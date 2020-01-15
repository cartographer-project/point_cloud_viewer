// Copyright 2018 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use crate::proto;
use crate::proto_grpc;
use crate::Color;
use cgmath::{PerspectiveFov, Point3, Quaternion, Rad, Vector3};
use collision::Aabb3;
use futures::sync::mpsc;
use futures::{Future, Sink, Stream};
use grpcio::{
    Environment, RpcContext, RpcStatus, RpcStatusCode, Server, ServerBuilder, ServerStreamingSink,
    UnarySink, WriteFlags,
};
use num_cpus;
use point_viewer::attributes::AttributeData;
use point_viewer::data_provider::DataProviderFactory;
use point_viewer::errors::*;
use point_viewer::iterator::{ParallelIterator, PointLocation, PointQuery};
use point_viewer::math::{Frustum, Isometry3};
use point_viewer::octree::{NodeId, Octree};
use point_viewer::PointsBatch;
use protobuf::Message;
use std::collections::HashMap;
use std::path::PathBuf;
use std::str::FromStr;
use std::sync::{Arc, RwLock};

struct OctreeServiceData {
    octree: Octree,
    meta: point_viewer::proto::Meta,
}

#[derive(Clone)]
struct OctreeService {
    location: PathBuf,
    data_cache: Arc<RwLock<HashMap<String, Arc<OctreeServiceData>>>>,
    factory: DataProviderFactory,
}

fn send_fail_stream<T>(ctx: &RpcContext, sink: ServerStreamingSink<T>, err_str: String) {
    let f = sink
        .fail(RpcStatus::new(RpcStatusCode::Internal, Some(err_str)))
        .map_err(move |err| eprintln!("Failed to reply: {:?}", err));
    ctx.spawn(f);
}

fn send_fail<T>(ctx: &RpcContext, sink: UnarySink<T>, err_str: String) {
    let f = sink
        .fail(RpcStatus::new(RpcStatusCode::Internal, Some(err_str)))
        .map_err(move |err| eprintln!("Failed to reply: {:?}", err));
    ctx.spawn(f);
}

impl proto_grpc::Octree for OctreeService {
    fn get_meta(
        &mut self,
        ctx: RpcContext,
        req: proto::GetMetaRequest,
        sink: UnarySink<proto::GetMetaReply>,
    ) {
        let mut resp = proto::GetMetaReply::new();
        let service_data = match self.get_service_data(&req.octree_id) {
            Ok(service_data) => service_data,
            Err(e) => return send_fail(&ctx, sink, e.to_string()),
        };
        resp.set_meta(service_data.meta.clone());
        let f = sink
            .success(resp)
            .map_err(move |e| println!("failed to reply {:?}: {:?}", req, e));
        ctx.spawn(f)
    }

    fn get_node_data(
        &mut self,
        ctx: RpcContext,
        req: proto::GetNodeDataRequest,
        sink: UnarySink<proto::GetNodeDataReply>,
    ) {
        let service_data = match self.get_service_data(&req.octree_id) {
            Ok(service_data) => service_data,
            Err(e) => return send_fail(&ctx, sink, e.to_string()),
        };
        let node_id = match NodeId::from_str(&req.id) {
            Ok(node_id) => node_id,
            Err(e) => return send_fail(&ctx, sink, e.to_string()),
        };
        let node_data = match service_data.octree.get_node_data(&node_id) {
            Ok(data) => data,
            Err(e) => return send_fail(&ctx, sink, e.to_string()),
        };
        let mut resp = proto::GetNodeDataReply::new();
        resp.mut_node()
            .set_position_encoding(node_data.meta.position_encoding.to_proto());
        resp.mut_node().set_num_points(node_data.meta.num_points);
        resp.set_position(node_data.position);
        resp.set_color(node_data.color);
        let f = sink
            .success(resp)
            .map_err(move |e| println!("failed to reply {:?}: {:?}", req, e));
        ctx.spawn(f)
    }

    fn get_points_in_frustum(
        &mut self,
        ctx: RpcContext,
        req: proto::GetPointsInFrustumRequest,
        resp: ServerStreamingSink<proto::PointsReply>,
    ) {
        let perspective = PerspectiveFov {
            fovy: Rad(req.fovy_rad),
            aspect: req.aspect,
            near: req.z_near,
            far: req.z_far,
        }
        .to_perspective();
        let rotation = {
            let q = req.rotation.unwrap();
            Quaternion::new(q.w, q.x, q.y, q.z)
        };

        let translation = {
            let t = req.translation.unwrap();
            Vector3::new(t.x, t.y, t.z)
        };

        let view_transform = Isometry3::<f64> {
            rotation,
            translation,
        };
        let frustum = Frustum::new(view_transform, perspective);
        let location = PointLocation::Frustum(frustum);
        self.stream_points_back_to_sink(location, &req.octree_id, &ctx, resp)
    }

    fn get_points_in_box(
        &mut self,
        ctx: RpcContext,
        req: proto::GetPointsInBoxRequest,
        resp: ServerStreamingSink<proto::PointsReply>,
    ) {
        let bounding_box = {
            let bounding_box = req.bounding_box.clone().unwrap();
            let min = bounding_box.min.unwrap();
            let max = bounding_box.max.unwrap();
            Aabb3::new(
                Point3::new(min.x, min.y, min.z),
                Point3::new(max.x, max.y, max.z),
            )
        };
        let location = PointLocation::Aabb(bounding_box);
        self.stream_points_back_to_sink(location, &req.octree_id, &ctx, resp)
    }

    fn get_all_points(
        &mut self,
        ctx: RpcContext,
        req: proto::GetAllPointsRequest,
        resp: ServerStreamingSink<proto::PointsReply>,
    ) {
        let location = PointLocation::AllPoints;
        self.stream_points_back_to_sink(location, &req.octree_id, &ctx, resp)
    }
}

impl OctreeService {
    fn stream_points_back_to_sink(
        &self,
        location: PointLocation,
        octree_id: &str,
        ctx: &RpcContext,
        resp: ServerStreamingSink<proto::PointsReply>,
    ) {
        use std::thread;

        let service_data = match self.get_service_data(octree_id) {
            Ok(service_data) => service_data,
            Err(e) => return send_fail_stream(&ctx, resp, e.to_string()),
        };

        // This creates a async-aware (tx, rx) pair that can wake up the event loop when new data
        // is piped through it.

        // We create a channel with a small buffer, which yields better performance than
        // fully blocking without requiring a ton of memory. This has not been carefully benchmarked
        // for best performance though.
        let buffer_size = 4;
        let (tx, rx) = mpsc::channel(buffer_size);
        thread::spawn(move || {
            // This is the secret sauce connecting an OS thread to a event-based receiver. Calling
            // wait() on this turns the event aware, i.e. async 'tx' into a blocking 'tx' that will
            // make this thread block when the event loop is not quick enough with piping out data.
            let mut tx = tx.wait();

            let mut reply = proto::PointsReply::new();
            let bytes_per_point = {
                let initial_proto_size = reply.compute_size();
                let mut v = point_viewer::proto::Vector3d::new();
                v.set_x(1.);
                v.set_y(1.);
                v.set_z(1.);
                reply.mut_positions().push(v);

                let mut v = point_viewer::proto::Color::new();
                v.set_red(1.);
                v.set_green(1.);
                v.set_blue(1.);
                v.set_alpha(1.);
                reply.mut_colors().push(v);

                reply.mut_intensities().push(1.);

                let final_proto_size = reply.compute_size();
                reply.mut_positions().clear();
                reply.mut_colors().clear();
                reply.mut_intensities().clear();
                final_proto_size - initial_proto_size
            };

            // Proto message must be below 4 MB.
            let max_message_size = 4 * 1024 * 1024;
            let num_points_per_batch: usize = max_message_size / bytes_per_point as usize;

            {
                // Extra scope to make sure that 'func' does not outlive 'reply'.
                // this function is currently not efficiently implemented
                let func = |p_data: PointsBatch| {
                    reply.positions = p_data
                        .position
                        .iter()
                        .map(|p| {
                            let mut v = point_viewer::proto::Vector3d::new();
                            v.set_x(p.x);
                            v.set_y(p.y);
                            v.set_z(p.z);
                            v
                        })
                        .collect();

                    reply.colors = match p_data.attributes.get(&"color".to_string()) {
                        Some(AttributeData::U8Vec3(data)) => data
                            .iter()
                            .map(|p| {
                                let rgb8: Color<u8> = crate::Color {
                                    red: p.x,
                                    green: p.y,
                                    blue: p.z,
                                    alpha: 255,
                                };
                                let rgb32: Color<f32> = crate::Color::to_f32(rgb8);
                                let mut v = point_viewer::proto::Color::new();
                                v.set_red(rgb32.red);
                                v.set_green(rgb32.green);
                                v.set_blue(rgb32.blue);
                                v.set_alpha(rgb32.alpha);
                                v
                            })
                            .collect(),
                        _ => {
                            return Err(std::io::Error::new(
                                std::io::ErrorKind::InvalidData,
                                "Color format is not u8",
                            )
                            .into());
                        }
                    };

                    reply.intensities = match p_data.attributes.get(&"intensity".to_string()) {
                        Some(AttributeData::F32(data)) => data.clone(),
                        _ => {
                            return Err(std::io::Error::new(
                                std::io::ErrorKind::InvalidData,
                                "Intensity format is not f32",
                            )
                            .into());
                        }
                    };

                    tx.send((reply.clone(), WriteFlags::default())).unwrap();
                    reply.mut_positions().clear();
                    reply.mut_colors().clear();
                    reply.mut_intensities().clear();
                    Ok(())
                };

                let octree_slice: &[Octree] = std::slice::from_ref(&service_data.octree);
                let point_query = PointQuery {
                    attributes: vec!["color", "intensity"],
                    location,
                    ..Default::default()
                };
                let mut parallel_iterator = ParallelIterator::new(
                    octree_slice,
                    &point_query,
                    num_points_per_batch,
                    num_cpus::get() - 1,
                    buffer_size,
                );
                // TODO(catevita): missing error handling for the thread
                let _result = parallel_iterator.try_for_each_batch(func);
            }
            tx.send((reply, WriteFlags::default())).unwrap();
        });

        let rx = rx.map_err(|_| grpcio::Error::RemoteStopped);
        let f = resp
            .send_all(rx)
            .map(|_| {})
            .map_err(|e| println!("failed to reply: {:?}", e));
        ctx.spawn(f)
    }

    fn get_service_data(&self, octree_id: &str) -> Result<Arc<OctreeServiceData>> {
        if let Some(service_data) = self.data_cache.read().unwrap().get(octree_id) {
            return Ok(Arc::clone(service_data));
        };
        let octree = Octree::from_data_provider(
            self.factory
                .generate_data_provider(self.location.join(&octree_id).to_string_lossy())?,
        )?;
        let meta = octree.to_meta_proto();
        let service_data = Arc::new(OctreeServiceData { octree, meta });
        self.data_cache
            .write()
            .unwrap()
            .insert(octree_id.to_string(), Arc::clone(&service_data));
        Ok(service_data)
    }
}

pub fn start_grpc_server(
    host: &str,
    port: u16,
    location: impl Into<PathBuf>,
    factory: DataProviderFactory,
) -> Server {
    let env = Arc::new(Environment::new(1));
    let data_cache = Arc::new(RwLock::new(HashMap::new()));

    let service = proto_grpc::create_octree(OctreeService {
        location: location.into(),
        data_cache,
        factory,
    });
    ServerBuilder::new(env)
        .register_service(service)
        .bind(host /* ip to bind to */, port)
        .build()
        .unwrap()
}
