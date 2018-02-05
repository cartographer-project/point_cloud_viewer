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
extern crate protobuf;

include!(concat!(env!("OUT_DIR"), "/proto.rs"));
include!(concat!(env!("OUT_DIR"), "/proto_grpc.rs"));

use cgmath::{Matrix4, Point3};
use collision::Aabb3;
use futures::{Future, Stream};
use grpcio::{ChannelBuilder, EnvBuilder};
use point_viewer::errors::*;
use point_viewer::math::Cube;
use point_viewer::octree::{NodeData, NodeId, NodeMeta, Octree, PositionEncoding, UseLod,
                           VisibleNode};
use proto_grpc::OctreeClient;
use std::sync::Arc;

pub struct GrpcOctree {
    client: OctreeClient,
    root_bounding_cube: Cube,
}

impl GrpcOctree {
    pub fn new(addr: &str) -> Self {
        let env = Arc::new(EnvBuilder::new().build());
        let ch = ChannelBuilder::new(env).connect(addr);
        let client = OctreeClient::new(ch);

        let reply = client
            .get_root_bounding_cube(&proto::GetRootBoundingCubeRequest::new())
            .expect("rpc");
        let root_bounding_cube = {
            let proto = reply.bounding_cube.as_ref().unwrap();
            let min = proto.min.as_ref().unwrap();
            Cube::new(Point3::new(min.x, min.y, min.z), proto.edge_length)
        };

        GrpcOctree {
            client,
            root_bounding_cube,
        }
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
        replies.for_each(|reply| {
            for point in reply.points.iter() {
                points.push(Point3::new(point.x, point.y, point.z));
            }
            Ok(())
        }).wait().unwrap();
        points
    }
}

impl Octree for GrpcOctree {
    fn get_visible_nodes(
        &self,
        projection_matrix: &Matrix4<f32>,
        width: i32,
        height: i32,
        _: UseLod,
    ) -> Vec<VisibleNode> {
        // TODO(sirver): remove UseLod from the interface and leave this to the client.
        let mut req = proto::GetVisibleNodesRequest::new();
        req.mut_projection_matrix().extend_from_slice(&[
            projection_matrix.x[0],
            projection_matrix.x[1],
            projection_matrix.x[2],
            projection_matrix.x[3],
            projection_matrix.y[0],
            projection_matrix.y[1],
            projection_matrix.y[2],
            projection_matrix.y[3],
            projection_matrix.z[0],
            projection_matrix.z[1],
            projection_matrix.z[2],
            projection_matrix.z[3],
            projection_matrix.w[0],
            projection_matrix.w[1],
            projection_matrix.w[2],
            projection_matrix.w[3],
        ]);
        req.set_width(width);
        req.set_height(height);
        // TODO(sirver): This should most definitively not crash, but instead return an error.
        // Needs changes to the trait though.
        let reply = self.client.get_visible_nodes(&req).expect("rpc");

        let mut result = Vec::new();
        for node in &reply.node_ids {
            result.push(VisibleNode::new(
                NodeId::from_str(&node),
                1, /* level_of_detail */
            ));
        }
        result
    }

    fn get_node_data(&self, node_id: &NodeId, level_of_detail: i32) -> Result<NodeData> {
        assert_eq!(level_of_detail, 1);
        // TODO(sirver): We ignore 'level_of_detail'. Hoist out of the interface and let the client
        // deal with it.
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
                bounding_cube: node_id.find_bounding_cube(&self.root_bounding_cube),
            },
        };
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // #[test]
    // fn test_receive_points() {
    //     let octree = GrpcOctree::new("127.0.0.1:50051"); // data set 351
    //     let bounding_box = Aabb3::new(Point3::new(-10., -10., -10.), Point3::new(10., 10., 10.));
    //     let points = octree.get_points_in_box(&bounding_box);
    //     assert_eq!(points.len(), 1845651);
    // }
}
