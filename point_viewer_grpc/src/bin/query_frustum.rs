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
use cgmath::{Deg, EuclideanSpace, Point3, Rad};
use collision::{Aabb, Aabb3};
use futures::{Future, Stream};
use grpcio::{ChannelBuilder, EnvBuilder};
use point_viewer::color::Color;
use point_viewer::generation::build_octree;
use point_viewer::{InternalIterator, Point};
pub use point_viewer_grpc_proto_rust::proto::GetPointsInFrustumRequest;
pub use point_viewer_grpc_proto_rust::proto_grpc;
use std::sync::Arc;

struct Points {
    points: Vec<Point>,
}

impl InternalIterator for Points {
    fn for_each<F: FnMut(&Point)>(self, mut func: F) {
        for p in &self.points {
            func(p);
        }
    }
    fn size_hint(&self) -> Option<usize> {
        Some(self.points.len())
    }
}

fn main() {
    let env = Arc::new(EnvBuilder::new().build());
    let ch = ChannelBuilder::new(env)
        .max_receive_message_len(::std::i32::MAX)
        .connect("127.0.0.1:50051");
    let client = OctreeClient::new(ch);

    let mut request = GetPointsInFrustumRequest::new();
    request.mut_rotation().set_x(-0.302_828_07);
    request.mut_rotation().set_y(0.182_317_38);
    request.mut_rotation().set_z(0.482_488_93);
    request.mut_rotation().set_w(0.801_411_3);

    request.mut_translation().set_x(-0.791_012_76);
    request.mut_translation().set_y(-105.560_104);
    request.mut_translation().set_z(-132.893_23);

    request.set_fovy_rad(Rad::from(Deg(45.)).0);
    request.set_aspect(800. / 600.);
    request.set_z_near(0.1);
    request.set_z_far(10000.);

    let mut bounding_box = Aabb3::zero();

    let replies = client.get_points_in_frustum(&request).expect("rpc");
    let mut points = Vec::new();
    replies
        .for_each(|reply| {
            let last_num_points = points.len();
            for (position, color) in reply.positions.iter().zip(reply.colors.iter()) {
                let p = Point3::new(position.x, position.y, position.z);
                bounding_box = bounding_box.grow(p);
                points.push(Point {
                    position: p.to_vec(),
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
            Ok(())
        })
        .wait()
        .unwrap();
    let pool = scoped_pool::Pool::new(10);
    build_octree(&pool, "/tmp/octree", 0.001, bounding_box, Points { points });
}
