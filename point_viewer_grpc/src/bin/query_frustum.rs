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

use cgmath::{EuclideanSpace, Point3, Rad, Deg};
use collision::{Aabb3, Aabb};
use futures::{Future, Stream};
use grpcio::{ChannelBuilder, EnvBuilder};
pub use point_viewer_grpc_proto_rust::proto::GetPointsInFrustumRequest;
pub use point_viewer_grpc_proto_rust::proto_grpc;
use point_viewer::{InternalIterator, Point};
use point_viewer::color::Color;
use point_viewer::generation::build_octree;
use proto_grpc::OctreeClient;
use std::sync::Arc;

struct Points{
    points: Vec<Point3<f32>>
}

impl InternalIterator for Points {
    fn for_each<F: FnMut(&Point)>(self, mut func: F) {
        for p in self.points {
            func(&Point {
                position: p.to_vec(),
                color: Color{
                    red: 255,
                    green: 0,
                    blue: 255,
                    alpha: 255,
                },
                intensity: None,
            });
        }
    }
    fn size_hint(&self) -> Option<usize> {
        Some(self.points.len())
    }
}

fn main() {
    let env = Arc::new(EnvBuilder::new().build());
    let ch = ChannelBuilder::new(env)
        .max_receive_message_len(::std::usize::MAX)
        .connect("127.0.0.1:50051");
    let client = OctreeClient::new(ch);

    let mut request = GetPointsInFrustumRequest::new();
    request.mut_rotation().set_x(-0.30282807);
    request.mut_rotation().set_y(0.18231738 );
    request.mut_rotation().set_z(0.48248893  );
    request.mut_rotation().set_w(0.8014113  );

    request.mut_translation().set_x(-0.79101276);
    request.mut_translation().set_y(-105.560104);
    request.mut_translation().set_z(-132.89323  );

    request.set_fovy_rad(Rad::from(Deg(45.)).0);
    request.set_aspect(800./600.);
    request.set_z_near(0.1);
    request.set_z_far(10000.);

    let mut bounding_box = Aabb3::zero();

    let replies = client.get_points_in_frustum(&request).expect("rpc");
    let mut points = Vec::new();
    replies
        .for_each(|reply| {
            for point in reply.points.iter() {
                let p = Point3::new(point.x, point.y, point.z);
                bounding_box = bounding_box.grow(p);
                points.push(p);
            }
            Ok(())
        })
        .wait()
        .unwrap();
    build_octree("/tmp/octree", 0.001, bounding_box, Points{points});
}
