// Copyright 2016 The Cartographer Authors
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

use cgmath::Matrix4;
use point_viewer::data_provider::DataProviderFactory;
use point_viewer::math::Isometry3;
use point_viewer::octree::Octree;
use point_viewer_grpc::data_provider_from_grpc_address;
use sdl_viewer::{opengl, run, Extension};
use std::rc::Rc;

struct NullExtension;

impl Extension for NullExtension {
    fn pre_init<'a, 'b>(app: clap::App<'a, 'b>) -> clap::App<'a, 'b> {
        app
    }

    fn new(_: &clap::ArgMatches, _: Rc<opengl::Gl>) -> Self {
        Self
    }

    fn local_from_global(_: &clap::ArgMatches, _: &Octree) -> Option<Isometry3<f64>> {
        None
    }

    fn camera_changed(&mut self, _: &Matrix4<f64>) {}

    fn draw(&mut self) {}
}

fn main() {
    let data_provider_factory =
        DataProviderFactory::new().register("grpc://", data_provider_from_grpc_address);
    // TODO(catevita): hide data provider factory details, simplify the run method interface
    run::<NullExtension>(data_provider_factory);
}
