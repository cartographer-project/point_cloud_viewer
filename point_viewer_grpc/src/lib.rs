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

use crate::proto_grpc::OctreeClient;
use cgmath::Vector3;
use collision::Aabb3;
use futures::{Future, Stream};
use grpcio::{ChannelBuilder, EnvBuilder};
use point_viewer::color::Color;
use point_viewer::data_provider::{DataProvider, DataProviderFactoryResult};
use point_viewer::errors::*;
use point_viewer::proto::Meta;
use point_viewer::Point;
pub use point_viewer_grpc_proto_rust::proto;
pub use point_viewer_grpc_proto_rust::proto_grpc;
use std::collections::HashMap;
use std::io::{Cursor, Read};
use std::sync::Arc;

pub mod service;

pub struct GrpcOctreeDataProvider {
    client: OctreeClient,
    octree_id: String,
}

impl GrpcOctreeDataProvider {
    pub fn from_address(addr: &str) -> Result<Self> {
        let mut addr_parts = addr.trim_matches('/').splitn(2, '/');
        let addr = addr_parts.next().ok_or_else(|| "Invalid address.")?;
        let octree_id = addr_parts.next().unwrap_or_default().to_string();
        let env = Arc::new(EnvBuilder::new().build());
        let ch = ChannelBuilder::new(env)
            .max_receive_message_len(::std::i32::MAX)
            .connect(addr);
        let client = OctreeClient::new(ch);

        Ok(GrpcOctreeDataProvider { client, octree_id })
    }

    pub fn get_points_in_box(
        &self,
        bounding_box: &Aabb3<f64>,
        mut func: impl FnMut(&[Point]) -> bool,
    ) -> Result<()> {
        let mut req = proto::GetPointsInBoxRequest::new();
        req.set_octree_id(self.octree_id.clone());
        req.mut_bounding_box().mut_min().set_x(bounding_box.min.x);
        req.mut_bounding_box().mut_min().set_y(bounding_box.min.y);
        req.mut_bounding_box().mut_min().set_z(bounding_box.min.z);
        req.mut_bounding_box().mut_max().set_x(bounding_box.max.x);
        req.mut_bounding_box().mut_max().set_y(bounding_box.max.y);
        req.mut_bounding_box().mut_max().set_z(bounding_box.max.z);
        let replies = self
            .client
            .get_points_in_box(&req)
            .map_err(|_| point_viewer::errors::ErrorKind::Grpc)?;

        let mut points = Vec::new();
        let mut interrupted = false;
        let result = replies
            .for_each(|reply| {
                let last_num_points = points.len();
                for (p, color) in reply.positions.iter().zip(reply.colors.iter()) {
                    points.push(Point {
                        position: Vector3::new(p.x, p.y, p.z),
                        color: Color {
                            red: color.red,
                            green: color.green,
                            blue: color.blue,
                            alpha: color.alpha,
                        }
                        .to_u8(),
                        intensity: None,
                    });
                }

                if reply.intensities.len() == reply.positions.len() {
                    for (i, p) in reply.intensities.iter().zip(&mut points[last_num_points..]) {
                        p.intensity = Some(*i);
                    }
                }

                if !func(&points) {
                    interrupted = true;
                    return Err(grpcio::Error::QueueShutdown);
                }
                points.clear();
                Ok(())
            })
            .wait()
            .map_err(|_| point_viewer::errors::ErrorKind::Grpc);
        if result.is_err() && !interrupted {
            result?;
        }
        Ok(())
    }
}

impl DataProvider for GrpcOctreeDataProvider {
    fn meta_proto(&self) -> Result<Meta> {
        let mut req = proto::GetMetaRequest::new();
        req.set_octree_id(self.octree_id.clone());
        let reply = self
            .client
            .get_meta(&req)
            .map_err(|_| point_viewer::errors::ErrorKind::Grpc)?;
        Ok(reply.meta.unwrap())
    }

    fn data(
        &self,
        node_id: &str,
        node_attributes: &[&str],
    ) -> Result<HashMap<String, Box<dyn Read + Send>>> {
        let mut req = proto::GetNodeDataRequest::new();
        req.set_octree_id(self.octree_id.clone());
        req.set_id(node_id.to_string());
        let reply = self
            .client
            .get_node_data(&req)
            .map_err(|_| point_viewer::errors::ErrorKind::Grpc)?;
        let mut readers = HashMap::<String, Box<dyn Read + Send>>::new();
        for node_attribute in node_attributes {
            let reader: Box<dyn Read + Send> = match *node_attribute {
                "position" => Box::new(Cursor::new(reply.position.clone())),
                "color" => Box::new(Cursor::new(reply.color.clone())),
                _ => {
                    return Err("Unsupported node extension.".into());
                }
            };
            readers.insert(node_attribute.to_string(), reader);
        }
        Ok(readers)
    }
}

pub fn data_provider_from_grpc_address(addr: &str) -> DataProviderFactoryResult {
    let prefix = "grpc://";
    if !addr.starts_with(prefix) {
        return Err(format!("Invalid grpc address: it has to start with {}.", prefix).into());
    }
    let addr_no_prefix: &str = &addr[prefix.len()..];
    GrpcOctreeDataProvider::from_address(addr_no_prefix)
        .map(|provider| Box::new(provider) as Box<dyn DataProvider>)
}
