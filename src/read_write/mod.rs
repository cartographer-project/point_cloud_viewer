// Copyright 2016 Google Inc.
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

mod codec;
pub use self::codec::{
    attribute_to_proto, decode, encode, fixpoint_decode, fixpoint_encode, vec3_encode,
    vec3_fixpoint_encode, Encoding, PositionEncoding,
};

mod input_file_iterator;
pub use self::input_file_iterator::{make_stream, InputFile, InputFileIterator};

mod node_iterator;
pub use self::node_iterator::{NodeIterator, NodeReader};

mod node_writer;
pub use self::node_writer::{DataWriter, NodeWriter, OpenMode, WriteEncoded, WriteLE, WriteLEPos};

mod ply;
pub use self::ply::{PlyIterator, PlyNodeWriter};

mod pts;
pub use self::pts::PtsIterator;

mod raw;
pub use self::raw::{RawNodeReader, RawNodeWriter};

mod s2;
pub use self::s2::{s2_cell_to_proto, s2_cloud_to_meta_proto, S2Splitter};
