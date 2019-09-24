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
    decode, encode, fixpoint_decode, fixpoint_encode, vec3_encode, vec3_fixpoint_encode, Encoding,
    PositionEncoding,
};

mod input_file_iterator;
pub use self::input_file_iterator::{make_stream, InputFile, InputFileIterator};

mod node_iterator;
pub use self::node_iterator::NodeIterator;

mod node_writer;
pub use self::node_writer::{DataWriter, NodeWriter, OpenMode, WriteEncoded, WriteLE, WriteLEPos};

mod ply;
pub use self::ply::{PlyIterator, PlyNodeWriter};

mod pts;
pub use self::pts::PtsIterator;

mod raw;
pub use self::raw::{RawNodeReader, RawNodeWriter};

mod s2;
pub use self::s2::S2Splitter;

use std::io::{BufReader, Read};

pub struct AttributeReader {
    pub data_type: crate::AttributeDataType,
    pub reader: BufReader<Box<dyn Read + Send>>,
}

/// We open a lot of files during our work. Sometimes users see errors with 'cannot open more
/// files'. This utility function attempt to increase the rlimits for the number of open files per
/// process here, but fails silently if we are not successful.
pub fn attempt_increasing_rlimit_to_max() {
    unsafe {
        let mut rl = libc::rlimit {
            rlim_cur: 0,
            rlim_max: 0,
        };
        libc::getrlimit(libc::RLIMIT_NOFILE, &mut rl);
        rl.rlim_cur = rl.rlim_max;
        libc::setrlimit(libc::RLIMIT_NOFILE, &rl);
    }
}
