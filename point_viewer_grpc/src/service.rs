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
use cgmath::{Decomposed, Matrix4, PerspectiveFov, Point3, Quaternion, Rad, Transform, Vector3};
use collision::Aabb3;
use futures::sync::mpsc;
use futures::{Future, Sink, Stream};
use grpcio::{
    Environment, RpcContext, Server, ServerBuilder, ServerStreamingSink, UnarySink, WriteFlags,
};
use point_viewer::octree::{NodeId, Octree, OctreeFactory};
use point_viewer::{InternalIterator, Point};
use protobuf::Message;
use std::collections::HashMap;
use std::path::PathBuf;
use std::str::FromStr;
use std::sync::{Arc, RwLock};

struct OctreeServiceData {
    octree: Box<Octree>,
    meta: point_viewer::proto::Meta,
}

#[derive(Clone)]
struct OctreeService {
    location: PathBuf,
    data_cache: Arc<RwLock<HashMap<String, Arc<OctreeServiceData>>>>,
    factory: OctreeFactory,
}

#[derive(Debug)]
enum OctreeQuery {
    Frustum(Matrix4<f32>),
    Box(Aabb3<f32>),
    FullPointcloud,
}

fn stream_points_back_to_sink(
    query: OctreeQuery,
    service_data: Arc<OctreeServiceData>,
    ctx: &RpcContext,
    resp: ServerStreamingSink<proto::PointsReply>,
) {
    use std::thread;

    // This creates a async-aware (tx, rx) pair that can wake up the event loop when new data
    // is piped through it.

    // We create a channel with a small buffer, which yields better performance than
    // fully blocking without requiring a ton of memory. This has not been carefully benchmarked
    // for best performance though.
    let (tx, rx) = mpsc::channel(4);
    thread::spawn(move || {
        // This is the secret sauce connecting an OS thread to a event-based receiver. Calling
        // wait() on this turns the event aware, i.e. async 'tx' into a blocking 'tx' that will
        // make this thread block when the event loop is not quick enough with piping out data.
        let mut tx = tx.wait();

        let mut reply = proto::PointsReply::new();
        let bytes_per_point = {
            let initial_proto_size = reply.compute_size();
            let mut v = point_viewer::proto::Vector3f::new();
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
        let mut reply_size = 0;

        {
            // Extra scope to make sure that 'func' does not outlive 'reply'.
            let func = |p: &Point| {
                {
                    let mut v = point_viewer::proto::Vector3f::new();
                    v.set_x(p.position.x);
                    v.set_y(p.position.y);
                    v.set_z(p.position.z);
                    reply.mut_positions().push(v);
                }

                {
                    let mut v = point_viewer::proto::Color::new();
                    let clr = p.color.to_f32();
                    v.set_red(clr.red);
                    v.set_green(clr.green);
                    v.set_blue(clr.blue);
                    v.set_alpha(clr.alpha);
                    reply.mut_colors().push(v);
                }

                if let Some(i) = p.intensity {
                    reply.mut_intensities().push(i);
                }

                reply_size += bytes_per_point;
                if reply_size > max_message_size - bytes_per_point {
                    tx.send((reply.clone(), WriteFlags::default())).unwrap();
                    reply.mut_positions().clear();
                    reply.mut_colors().clear();
                    reply.mut_intensities().clear();
                    reply_size = 0;
                }
            };
            match query {
                OctreeQuery::Box(bounding_box) => service_data
                    .octree
                    .points_in_box(&bounding_box)
                    .for_each(func),
                OctreeQuery::Frustum(frustum_matrix) => service_data
                    .octree
                    .points_in_frustum(&frustum_matrix)
                    .for_each(func),
                OctreeQuery::FullPointcloud => service_data.octree.all_points().for_each(func),
            };
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

impl proto_grpc::Octree for OctreeService {
    fn get_meta(
        &self,
        ctx: RpcContext,
        req: proto::GetMetaRequest,
        sink: UnarySink<proto::GetMetaReply>,
    ) {
        let mut resp = proto::GetMetaReply::new();
        resp.set_meta(self.get_service_data(&req.octree_id).meta.clone());
        let f = sink
            .success(resp)
            .map_err(move |e| println!("failed to reply {:?}: {:?}", req, e));
        ctx.spawn(f)
    }

    fn get_node_data(
        &self,
        ctx: RpcContext,
        req: proto::GetNodeDataRequest,
        sink: UnarySink<proto::GetNodeDataReply>,
    ) {
        let data = self
            .get_service_data(&req.octree_id)
            .octree
            .get_node_data(&NodeId::from_str(&req.id).unwrap())
            .unwrap();
        let mut resp = proto::GetNodeDataReply::new();
        resp.mut_node()
            .set_position_encoding(data.meta.position_encoding.to_proto());
        resp.mut_node().set_num_points(data.meta.num_points);
        resp.set_position(data.position);
        resp.set_color(data.color);
        let f = sink
            .success(resp)
            .map_err(move |e| println!("failed to reply {:?}: {:?}", req, e));
        ctx.spawn(f)
    }

    fn get_points_in_frustum(
        &self,
        ctx: RpcContext,
        req: proto::GetPointsInFrustumRequest,
        resp: ServerStreamingSink<proto::PointsReply>,
    ) {
        let projection_matrix = Matrix4::from(PerspectiveFov {
            fovy: Rad(req.fovy_rad as f32),
            aspect: (req.aspect as f32),
            near: (req.z_near as f32),
            far: (req.z_far as f32),
        });
        let rotation = {
            let q = req.rotation.unwrap();
            Quaternion::new(q.w, q.x, q.y, q.z)
        };

        let translation = {
            let t = req.translation.unwrap();
            Vector3::new(t.x, t.y, t.z)
        };

        let view_transform = Decomposed {
            scale: 1.0,
            rot: rotation,
            disp: translation,
        };
        let frustum_matrix = projection_matrix.concat(&view_transform.into());
        stream_points_back_to_sink(
            OctreeQuery::Frustum(frustum_matrix),
            self.get_service_data(&req.octree_id),
            &ctx,
            resp,
        )
    }

    fn get_points_in_box(
        &self,
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
        stream_points_back_to_sink(
            OctreeQuery::Box(bounding_box),
            self.get_service_data(&req.octree_id),
            &ctx,
            resp,
        )
    }

    fn get_all_points(
        &self,
        ctx: RpcContext,
        req: proto::GetAllPointsRequest,
        resp: ServerStreamingSink<proto::PointsReply>,
    ) {
        stream_points_back_to_sink(
            OctreeQuery::FullPointcloud,
            self.get_service_data(&req.octree_id),
            &ctx,
            resp,
        )
    }
}

impl OctreeService {
    pub fn get_service_data(&self, octree_id: &str) -> Arc<OctreeServiceData> {
        match self.data_cache.read().unwrap().get(octree_id) {
            None => {
                let octree = self
                    .factory
                    .generate_octree(self.location.join(&octree_id).to_string_lossy())
                    .unwrap();
                let meta = octree.to_meta_proto();
                let service_data = Arc::new(OctreeServiceData { octree, meta });
                self.data_cache
                    .write()
                    .unwrap()
                    .insert(octree_id.to_string(), Arc::clone(&service_data));
                service_data
            }
            Some(service_data) => Arc::clone(service_data),
        }
    }
}

pub fn start_grpc_server(
    host: &str,
    port: u16,
    location: impl Into<PathBuf>,
    factory: OctreeFactory,
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
