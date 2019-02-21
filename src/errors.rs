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

use crate::octree;
use error_chain::*;
use std::io;

error_chain! {
    foreign_links {
        Io(io::Error);
    }

    errors {
        InvalidInput(msg: String) {
            description("The input file is not supported or invalid")
                display("{}", msg)
        }

        InvalidVersion(version: i32) {
            description("invalid octree version on disk")
            display(
            "Octree in this directory has a version of {}, the only supported version is {}. \
            The viewer might eventually be backwards compatible, but for now only \
            the currently created version is supported.", version, octree::CURRENT_VERSION)
        }

        NodeNotFound {
            description("The node does not exist.")
        }

        Grpc {
            description("Grpc request failed")
        }
    }
}
