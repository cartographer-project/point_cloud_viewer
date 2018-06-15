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

extern crate cgmath;
extern crate collision;
extern crate futures;
extern crate grpcio;
extern crate point_viewer;
extern crate point_viewer_grpc_proto_rust;
extern crate protobuf;

use cgmath::{Matrix4, Point3};
use collision::Aabb3;
use futures::{Future, Stream};
use grpcio::{ChannelBuilder, EnvBuilder};
use point_viewer::errors::*;
use point_viewer::math::Cube;
use point_viewer::octree::{NodeData, NodeId, NodeMeta, Octree, OnDiskOctree, PositionEncoding};
pub use point_viewer_grpc_proto_rust::proto;
pub use point_viewer_grpc_proto_rust::proto_grpc;
use proto_grpc::OctreeClient;
use std::path::PathBuf;
use std::sync::Arc;

pub struct GrpcOctree {
    client: OctreeClient,
    octree: OnDiskOctree,
}

impl GrpcOctree {
    pub fn new(addr: &str) -> Self {
        let env = Arc::new(EnvBuilder::new().build());
        let ch = ChannelBuilder::new(env).max_receive_message_len(::std::usize::MAX).connect(addr);
        let client = OctreeClient::new(ch);

        let reply = client.get_meta(&proto::GetMetaRequest::new()).expect("rpc");
        // TODO(sirver): We pass a dummy directory and hope we never actually use it for anything.
        let octree = OnDiskOctree::from_meta(reply.meta.unwrap(), PathBuf::new()).unwrap();
        GrpcOctree { client, octree }
    }

    // TODO(tschiwietz): This function should return Result<> for error handling.
    pub fn get_points_in_box(&self, bounding_box: &Aabb3<f32>) -> Vec<Point3<f32>> {
        let mut req = proto::GetPointsInBoxRequest::new();
        req.mut_bounding_box().mut_min().set_x(bounding_box.min.x);
        req.mut_bounding_box().mut_min().set_y(bounding_box.min.y);
        req.mut_bounding_box().mut_min().set_z(bounding_box.min.z);
        req.mut_bounding_box().mut_max().set_x(bounding_box.max.x);
        req.mut_bounding_box().mut_max().set_y(bounding_box.max.y);
        req.mut_bounding_box().mut_max().set_z(bounding_box.max.z);
        let replies = self.client.get_points_in_box(&req).unwrap();
        let mut points = Vec::new();
        replies
            .for_each(|reply| {
                for point in reply.points.iter() {
                    points.push(Point3::new(point.x, point.y, point.z));
                }
                Ok(())
            })
            .wait()
            .unwrap();
        points
    }
}

impl Octree for GrpcOctree {
    fn get_visible_nodes(
        &self,
        projection_matrix: &Matrix4<f32>,
        width: i32,
        height: i32,
    ) -> Vec<NodeId> {
        self.octree
            .get_visible_nodes(projection_matrix, width, height)
    }

    fn get_node_data(&self, node_id: &NodeId) -> Result<NodeData> {
        let mut req = proto::GetNodeDataRequest::new();
        req.set_id(node_id.to_string());

        // TODO(sirver): This should most definitively not crash, but instead return an error.
        // Needs changes to the trait though.
        let reply = self.client.get_node_data(&req).expect("rpc");
        let node = reply.node.unwrap();
        let result = NodeData {
            position: reply.position,
            color: reply.color,
            meta: NodeMeta {
                num_points: node.num_points,
                position_encoding: PositionEncoding::from_proto(node.position_encoding).unwrap(),
                bounding_cube: node_id
                    .find_bounding_cube(&Cube::bounding(&self.octree.bounding_box())),
            },
        };
        Ok(result)
    }
}
