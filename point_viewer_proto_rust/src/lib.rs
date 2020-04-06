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

include!(concat!(env!("OUT_DIR"), "/proto.rs"));

impl From<&nalgebra::Point3<f64>> for proto::Vector3d {
    fn from(vec: &nalgebra::Point3<f64>) -> Self {
        let mut proto_vec = proto::Vector3d::new();
        proto_vec.set_x(vec.x);
        proto_vec.set_y(vec.y);
        proto_vec.set_z(vec.z);
        proto_vec
    }
}

impl From<proto::Vector3f> for proto::Vector3d {
    fn from(proto_vec_f: proto::Vector3f) -> Self {
        let mut proto_vec_d = proto::Vector3d::new();
        proto_vec_d.set_x(f64::from(proto_vec_f.x));
        proto_vec_d.set_y(f64::from(proto_vec_f.y));
        proto_vec_d.set_z(f64::from(proto_vec_f.z));
        proto_vec_d
    }
}

impl From<&proto::Vector3d> for nalgebra::Point3<f64> {
    fn from(proto_vec: &proto::Vector3d) -> Self {
        nalgebra::Point3::new(proto_vec.get_x(), proto_vec.get_y(), proto_vec.get_z())
    }
}
