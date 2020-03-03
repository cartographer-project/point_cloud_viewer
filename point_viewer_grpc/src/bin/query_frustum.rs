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
use futures::{Future, Stream};
use grpcio::{ChannelBuilder, EnvBuilder};
use nalgebra::{Point3, Vector3};
use num_integer::div_ceil;
use point_viewer::attributes::AttributeData;
use point_viewer::color::Color;
use point_viewer::math::AABB;
use point_viewer::octree::build_octree;
use point_viewer::{NumberOfPoints, Point, PointsBatch, NUM_POINTS_PER_BATCH};
pub use point_viewer_grpc_proto_rust::proto::GetPointsInFrustumRequest;
pub use point_viewer_grpc_proto_rust::proto_grpc;
use std::collections::BTreeMap;
use std::sync::Arc;

struct Points {
    points: Vec<Point>,
    point_count: usize,
}

impl Points {
    fn new(points: Vec<Point>) -> Self {
        Points {
            points,
            point_count: 0,
        }
    }
}

impl NumberOfPoints for Points {
    fn num_points(&self) -> usize {
        self.points.len()
    }
}

impl Iterator for Points {
    type Item = PointsBatch;

    fn next(&mut self) -> Option<PointsBatch> {
        if self.point_count == self.points.len() {
            return None;
        }
        let batch_size = std::cmp::min(NUM_POINTS_PER_BATCH, self.points.len() - self.point_count);
        let mut position = Vec::with_capacity(batch_size);
        let mut color = Vec::with_capacity(batch_size);
        let mut intensity = None;
        for _ in 0..batch_size {
            let point = &self.points[self.point_count];
            position.push(point.position);
            color.push(Vector3::new(
                point.color.red,
                point.color.green,
                point.color.blue,
            ));
            if let Some(i) = point.intensity {
                if intensity.is_none() {
                    intensity = Some(Vec::with_capacity(batch_size));
                }
                intensity.as_mut().unwrap().push(i);
            }
            self.point_count += 1;
        }
        let mut attributes = BTreeMap::new();
        attributes.insert("color".to_string(), AttributeData::U8Vec3(color));
        if let Some(intensity) = intensity {
            attributes.insert("intensity".to_string(), AttributeData::F32(intensity));
        }
        Some(PointsBatch {
            position,
            attributes,
        })
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let num_batches = div_ceil(self.points.len(), NUM_POINTS_PER_BATCH);
        (num_batches, Some(num_batches))
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

    request.set_fovy_rad(std::f64::consts::FRAC_PI_4);
    request.set_aspect(800. / 600.);
    request.set_z_near(0.1);
    request.set_z_far(10000.);

    let mut bounding_box = AABB::zero();

    let replies = client.get_points_in_frustum(&request).expect("rpc");
    let mut points = Vec::new();
    replies
        .for_each(|reply| {
            let last_num_points = points.len();
            for (position, color) in reply.positions.iter().zip(reply.colors.iter()) {
                let p = Point3::new(position.x, position.y, position.z);
                bounding_box.grow(p);
                points.push(Point {
                    position: p,
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
    rayon::ThreadPoolBuilder::new()
        .num_threads(10)
        .build_global()
        .expect("Could not create thread pool.");
    build_octree(
        "/tmp/octree",
        0.001,
        bounding_box,
        Points::new(points),
        &["color"],
    );
}
