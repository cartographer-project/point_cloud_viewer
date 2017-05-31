// This file is generated. Do not edit
// @generated

// https://github.com/Manishearth/rust-clippy/issues/702
#![allow(unknown_lints)]
#![allow(clippy)]

#![cfg_attr(rustfmt, rustfmt_skip)]

#![allow(box_pointers)]
#![allow(dead_code)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]
#![allow(trivial_casts)]
#![allow(unsafe_code)]
#![allow(unused_imports)]
#![allow(unused_results)]

use protobuf::Message as Message_imported_for_functions;
use protobuf::ProtobufEnum as ProtobufEnum_imported_for_functions;

#[derive(Clone,Default)]
pub struct Vector3f {
    // message fields
    x: ::std::option::Option<f32>,
    y: ::std::option::Option<f32>,
    z: ::std::option::Option<f32>,
    // special fields
    unknown_fields: ::protobuf::UnknownFields,
    cached_size: ::std::cell::Cell<u32>,
}

// see codegen.rs for the explanation why impl Sync explicitly
unsafe impl ::std::marker::Sync for Vector3f {}

impl Vector3f {
    pub fn new() -> Vector3f {
        ::std::default::Default::default()
    }

    pub fn default_instance() -> &'static Vector3f {
        static mut instance: ::protobuf::lazy::Lazy<Vector3f> = ::protobuf::lazy::Lazy {
            lock: ::protobuf::lazy::ONCE_INIT,
            ptr: 0 as *const Vector3f,
        };
        unsafe {
            instance.get(
                || {
                    Vector3f {
                        x: ::std::option::Option::None,
                        y: ::std::option::Option::None,
                        z: ::std::option::Option::None,
                        unknown_fields: ::protobuf::UnknownFields::new(),
                        cached_size: ::std::cell::Cell::new(0),
                    }
                }
            )
        }
    }

    // optional float x = 1;

    pub fn clear_x(&mut self) {
        self.x = ::std::option::Option::None;
    }

    pub fn has_x(&self) -> bool {
        self.x.is_some()
    }

    // Param is passed by value, moved
    pub fn set_x(&mut self, v: f32) {
        self.x = ::std::option::Option::Some(v);
    }

    pub fn get_x(&self) -> f32 {
        self.x.unwrap_or(0.)
    }

    // optional float y = 2;

    pub fn clear_y(&mut self) {
        self.y = ::std::option::Option::None;
    }

    pub fn has_y(&self) -> bool {
        self.y.is_some()
    }

    // Param is passed by value, moved
    pub fn set_y(&mut self, v: f32) {
        self.y = ::std::option::Option::Some(v);
    }

    pub fn get_y(&self) -> f32 {
        self.y.unwrap_or(0.)
    }

    // optional float z = 3;

    pub fn clear_z(&mut self) {
        self.z = ::std::option::Option::None;
    }

    pub fn has_z(&self) -> bool {
        self.z.is_some()
    }

    // Param is passed by value, moved
    pub fn set_z(&mut self, v: f32) {
        self.z = ::std::option::Option::Some(v);
    }

    pub fn get_z(&self) -> f32 {
        self.z.unwrap_or(0.)
    }
}

impl ::protobuf::Message for Vector3f {
    fn is_initialized(&self) -> bool {
        true
    }

    fn merge_from(
        &mut self,
        is: &mut ::protobuf::CodedInputStream,
    ) -> ::protobuf::ProtobufResult<()> {
        while !try!(is.eof()) {
            let (field_number, wire_type) = try!(is.read_tag_unpack());
            match field_number {
                1 => {
                    if wire_type != ::protobuf::wire_format::WireTypeFixed32 {
                        return ::std::result::Result::Err(
                            ::protobuf::rt::unexpected_wire_type(wire_type),
                        );
                    };
                    let tmp = try!(is.read_float());
                    self.x = ::std::option::Option::Some(tmp);
                }
                2 => {
                    if wire_type != ::protobuf::wire_format::WireTypeFixed32 {
                        return ::std::result::Result::Err(
                            ::protobuf::rt::unexpected_wire_type(wire_type),
                        );
                    };
                    let tmp = try!(is.read_float());
                    self.y = ::std::option::Option::Some(tmp);
                }
                3 => {
                    if wire_type != ::protobuf::wire_format::WireTypeFixed32 {
                        return ::std::result::Result::Err(
                            ::protobuf::rt::unexpected_wire_type(wire_type),
                        );
                    };
                    let tmp = try!(is.read_float());
                    self.z = ::std::option::Option::Some(tmp);
                }
                _ => {
                    try!(
                        ::protobuf::rt::read_unknown_or_skip_group(
                            field_number,
                            wire_type,
                            is,
                            self.mut_unknown_fields(),
                        )
                    );
                }
            };
        }
        ::std::result::Result::Ok(())
    }

    // Compute sizes of nested messages
    #[allow(unused_variables)]
    fn compute_size(&self) -> u32 {
        let mut my_size = 0;
        if self.x.is_some() {
            my_size += 5;
        };
        if self.y.is_some() {
            my_size += 5;
        };
        if self.z.is_some() {
            my_size += 5;
        };
        my_size += ::protobuf::rt::unknown_fields_size(self.get_unknown_fields());
        self.cached_size.set(my_size);
        my_size
    }

    fn write_to_with_cached_sizes(
        &self,
        os: &mut ::protobuf::CodedOutputStream,
    ) -> ::protobuf::ProtobufResult<()> {
        if let Some(v) = self.x {
            try!(os.write_float(1, v));
        };
        if let Some(v) = self.y {
            try!(os.write_float(2, v));
        };
        if let Some(v) = self.z {
            try!(os.write_float(3, v));
        };
        try!(os.write_unknown_fields(self.get_unknown_fields()));
        ::std::result::Result::Ok(())
    }

    fn get_cached_size(&self) -> u32 {
        self.cached_size.get()
    }

    fn get_unknown_fields(&self) -> &::protobuf::UnknownFields {
        &self.unknown_fields
    }

    fn mut_unknown_fields(&mut self) -> &mut ::protobuf::UnknownFields {
        &mut self.unknown_fields
    }

    fn type_id(&self) -> ::std::any::TypeId {
        ::std::any::TypeId::of::<Vector3f>()
    }

    fn as_any(&self) -> &::std::any::Any {
        self as &::std::any::Any
    }

    fn descriptor(&self) -> &'static ::protobuf::reflect::MessageDescriptor {
        ::protobuf::MessageStatic::descriptor_static(None::<Self>)
    }
}

impl ::protobuf::MessageStatic for Vector3f {
    fn new() -> Vector3f {
        Vector3f::new()
    }

    fn descriptor_static(_: ::std::option::Option<Vector3f>)
        -> &'static ::protobuf::reflect::MessageDescriptor {
        static mut descriptor: ::protobuf::lazy::Lazy<::protobuf::reflect::MessageDescriptor> =
            ::protobuf::lazy::Lazy {
                lock: ::protobuf::lazy::ONCE_INIT,
                ptr: 0 as *const ::protobuf::reflect::MessageDescriptor,
            };
        unsafe {
            descriptor.get(
                || {
                    let mut fields = ::std::vec::Vec::new();
                    fields.push(
                        ::protobuf::reflect::accessor::make_singular_f32_accessor(
                            "x",
                            Vector3f::has_x,
                            Vector3f::get_x,
                        )
                    );
                    fields.push(
                        ::protobuf::reflect::accessor::make_singular_f32_accessor(
                            "y",
                            Vector3f::has_y,
                            Vector3f::get_y,
                        )
                    );
                    fields.push(
                        ::protobuf::reflect::accessor::make_singular_f32_accessor(
                            "z",
                            Vector3f::has_z,
                            Vector3f::get_z,
                        )
                    );
                    ::protobuf::reflect::MessageDescriptor::new::<Vector3f>(
                        "Vector3f",
                        fields,
                        file_descriptor_proto(),
                    )
                }
            )
        }
    }
}

impl ::protobuf::Clear for Vector3f {
    fn clear(&mut self) {
        self.clear_x();
        self.clear_y();
        self.clear_z();
        self.unknown_fields.clear();
    }
}

impl ::std::cmp::PartialEq for Vector3f {
    fn eq(&self, other: &Vector3f) -> bool {
        self.x == other.x && self.y == other.y && self.z == other.z &&
        self.unknown_fields == other.unknown_fields
    }
}

impl ::std::fmt::Debug for Vector3f {
    fn fmt(&self, f: &mut ::std::fmt::Formatter) -> ::std::fmt::Result {
        ::protobuf::text_format::fmt(self, f)
    }
}

#[derive(Clone,Default)]
pub struct BoundingCube {
    // message fields
    min: ::protobuf::SingularPtrField<Vector3f>,
    edge_length: ::std::option::Option<f32>,
    // special fields
    unknown_fields: ::protobuf::UnknownFields,
    cached_size: ::std::cell::Cell<u32>,
}

// see codegen.rs for the explanation why impl Sync explicitly
unsafe impl ::std::marker::Sync for BoundingCube {}

impl BoundingCube {
    pub fn new() -> BoundingCube {
        ::std::default::Default::default()
    }

    pub fn default_instance() -> &'static BoundingCube {
        static mut instance: ::protobuf::lazy::Lazy<BoundingCube> = ::protobuf::lazy::Lazy {
            lock: ::protobuf::lazy::ONCE_INIT,
            ptr: 0 as *const BoundingCube,
        };
        unsafe {
            instance.get(
                || {
                    BoundingCube {
                        min: ::protobuf::SingularPtrField::none(),
                        edge_length: ::std::option::Option::None,
                        unknown_fields: ::protobuf::UnknownFields::new(),
                        cached_size: ::std::cell::Cell::new(0),
                    }
                }
            )
        }
    }

    // optional .point_cloud_viewer.proto.Vector3f min = 1;

    pub fn clear_min(&mut self) {
        self.min.clear();
    }

    pub fn has_min(&self) -> bool {
        self.min.is_some()
    }

    // Param is passed by value, moved
    pub fn set_min(&mut self, v: Vector3f) {
        self.min = ::protobuf::SingularPtrField::some(v);
    }

    // Mutable pointer to the field.
    // If field is not initialized, it is initialized with default value first.
    pub fn mut_min(&mut self) -> &mut Vector3f {
        if self.min.is_none() {
            self.min.set_default();
        };
        self.min.as_mut().unwrap()
    }

    // Take field
    pub fn take_min(&mut self) -> Vector3f {
        self.min.take().unwrap_or_else(|| Vector3f::new())
    }

    pub fn get_min(&self) -> &Vector3f {
        self.min
            .as_ref()
            .unwrap_or_else(|| Vector3f::default_instance())
    }

    // optional float edge_length = 2;

    pub fn clear_edge_length(&mut self) {
        self.edge_length = ::std::option::Option::None;
    }

    pub fn has_edge_length(&self) -> bool {
        self.edge_length.is_some()
    }

    // Param is passed by value, moved
    pub fn set_edge_length(&mut self, v: f32) {
        self.edge_length = ::std::option::Option::Some(v);
    }

    pub fn get_edge_length(&self) -> f32 {
        self.edge_length.unwrap_or(0.)
    }
}

impl ::protobuf::Message for BoundingCube {
    fn is_initialized(&self) -> bool {
        true
    }

    fn merge_from(
        &mut self,
        is: &mut ::protobuf::CodedInputStream,
    ) -> ::protobuf::ProtobufResult<()> {
        while !try!(is.eof()) {
            let (field_number, wire_type) = try!(is.read_tag_unpack());
            match field_number {
                1 => {
                    try!(::protobuf::rt::read_singular_message_into(wire_type, is, &mut self.min));
                }
                2 => {
                    if wire_type != ::protobuf::wire_format::WireTypeFixed32 {
                        return ::std::result::Result::Err(
                            ::protobuf::rt::unexpected_wire_type(wire_type),
                        );
                    };
                    let tmp = try!(is.read_float());
                    self.edge_length = ::std::option::Option::Some(tmp);
                }
                _ => {
                    try!(
                        ::protobuf::rt::read_unknown_or_skip_group(
                            field_number,
                            wire_type,
                            is,
                            self.mut_unknown_fields(),
                        )
                    );
                }
            };
        }
        ::std::result::Result::Ok(())
    }

    // Compute sizes of nested messages
    #[allow(unused_variables)]
    fn compute_size(&self) -> u32 {
        let mut my_size = 0;
        for value in &self.min {
            let len = value.compute_size();
            my_size += 1 + ::protobuf::rt::compute_raw_varint32_size(len) + len;
        }
        if self.edge_length.is_some() {
            my_size += 5;
        };
        my_size += ::protobuf::rt::unknown_fields_size(self.get_unknown_fields());
        self.cached_size.set(my_size);
        my_size
    }

    fn write_to_with_cached_sizes(
        &self,
        os: &mut ::protobuf::CodedOutputStream,
    ) -> ::protobuf::ProtobufResult<()> {
        if let Some(v) = self.min.as_ref() {
            try!(os.write_tag(1, ::protobuf::wire_format::WireTypeLengthDelimited));
            try!(os.write_raw_varint32(v.get_cached_size()));
            try!(v.write_to_with_cached_sizes(os));
        };
        if let Some(v) = self.edge_length {
            try!(os.write_float(2, v));
        };
        try!(os.write_unknown_fields(self.get_unknown_fields()));
        ::std::result::Result::Ok(())
    }

    fn get_cached_size(&self) -> u32 {
        self.cached_size.get()
    }

    fn get_unknown_fields(&self) -> &::protobuf::UnknownFields {
        &self.unknown_fields
    }

    fn mut_unknown_fields(&mut self) -> &mut ::protobuf::UnknownFields {
        &mut self.unknown_fields
    }

    fn type_id(&self) -> ::std::any::TypeId {
        ::std::any::TypeId::of::<BoundingCube>()
    }

    fn as_any(&self) -> &::std::any::Any {
        self as &::std::any::Any
    }

    fn descriptor(&self) -> &'static ::protobuf::reflect::MessageDescriptor {
        ::protobuf::MessageStatic::descriptor_static(None::<Self>)
    }
}

impl ::protobuf::MessageStatic for BoundingCube {
    fn new() -> BoundingCube {
        BoundingCube::new()
    }

    fn descriptor_static(_: ::std::option::Option<BoundingCube>)
        -> &'static ::protobuf::reflect::MessageDescriptor {
        static mut descriptor: ::protobuf::lazy::Lazy<::protobuf::reflect::MessageDescriptor> =
            ::protobuf::lazy::Lazy {
                lock: ::protobuf::lazy::ONCE_INIT,
                ptr: 0 as *const ::protobuf::reflect::MessageDescriptor,
            };
        unsafe {
            descriptor.get(
                || {
                    let mut fields = ::std::vec::Vec::new();
                    fields.push(
                        ::protobuf::reflect::accessor::make_singular_message_accessor(
                            "min",
                            BoundingCube::has_min,
                            BoundingCube::get_min,
                        )
                    );
                    fields.push(
                        ::protobuf::reflect::accessor::make_singular_f32_accessor(
                            "edge_length",
                            BoundingCube::has_edge_length,
                            BoundingCube::get_edge_length,
                        )
                    );
                    ::protobuf::reflect::MessageDescriptor::new::<BoundingCube>(
                        "BoundingCube",
                        fields,
                        file_descriptor_proto(),
                    )
                }
            )
        }
    }
}

impl ::protobuf::Clear for BoundingCube {
    fn clear(&mut self) {
        self.clear_min();
        self.clear_edge_length();
        self.unknown_fields.clear();
    }
}

impl ::std::cmp::PartialEq for BoundingCube {
    fn eq(&self, other: &BoundingCube) -> bool {
        self.min == other.min && self.edge_length == other.edge_length &&
        self.unknown_fields == other.unknown_fields
    }
}

impl ::std::fmt::Debug for BoundingCube {
    fn fmt(&self, f: &mut ::std::fmt::Formatter) -> ::std::fmt::Result {
        ::protobuf::text_format::fmt(self, f)
    }
}

#[derive(Clone,Default)]
pub struct Meta {
    // message fields
    version: ::std::option::Option<i32>,
    bounding_cube: ::protobuf::SingularPtrField<BoundingCube>,
    resolution: ::std::option::Option<f64>,
    // special fields
    unknown_fields: ::protobuf::UnknownFields,
    cached_size: ::std::cell::Cell<u32>,
}

// see codegen.rs for the explanation why impl Sync explicitly
unsafe impl ::std::marker::Sync for Meta {}

impl Meta {
    pub fn new() -> Meta {
        ::std::default::Default::default()
    }

    pub fn default_instance() -> &'static Meta {
        static mut instance: ::protobuf::lazy::Lazy<Meta> = ::protobuf::lazy::Lazy {
            lock: ::protobuf::lazy::ONCE_INIT,
            ptr: 0 as *const Meta,
        };
        unsafe {
            instance.get(
                || {
                    Meta {
                        version: ::std::option::Option::None,
                        bounding_cube: ::protobuf::SingularPtrField::none(),
                        resolution: ::std::option::Option::None,
                        unknown_fields: ::protobuf::UnknownFields::new(),
                        cached_size: ::std::cell::Cell::new(0),
                    }
                }
            )
        }
    }

    // optional int32 version = 1;

    pub fn clear_version(&mut self) {
        self.version = ::std::option::Option::None;
    }

    pub fn has_version(&self) -> bool {
        self.version.is_some()
    }

    // Param is passed by value, moved
    pub fn set_version(&mut self, v: i32) {
        self.version = ::std::option::Option::Some(v);
    }

    pub fn get_version(&self) -> i32 {
        self.version.unwrap_or(0)
    }

    // optional .point_cloud_viewer.proto.BoundingCube bounding_cube = 2;

    pub fn clear_bounding_cube(&mut self) {
        self.bounding_cube.clear();
    }

    pub fn has_bounding_cube(&self) -> bool {
        self.bounding_cube.is_some()
    }

    // Param is passed by value, moved
    pub fn set_bounding_cube(&mut self, v: BoundingCube) {
        self.bounding_cube = ::protobuf::SingularPtrField::some(v);
    }

    // Mutable pointer to the field.
    // If field is not initialized, it is initialized with default value first.
    pub fn mut_bounding_cube(&mut self) -> &mut BoundingCube {
        if self.bounding_cube.is_none() {
            self.bounding_cube.set_default();
        };
        self.bounding_cube.as_mut().unwrap()
    }

    // Take field
    pub fn take_bounding_cube(&mut self) -> BoundingCube {
        self.bounding_cube
            .take()
            .unwrap_or_else(|| BoundingCube::new())
    }

    pub fn get_bounding_cube(&self) -> &BoundingCube {
        self.bounding_cube
            .as_ref()
            .unwrap_or_else(|| BoundingCube::default_instance())
    }

    // optional double resolution = 3;

    pub fn clear_resolution(&mut self) {
        self.resolution = ::std::option::Option::None;
    }

    pub fn has_resolution(&self) -> bool {
        self.resolution.is_some()
    }

    // Param is passed by value, moved
    pub fn set_resolution(&mut self, v: f64) {
        self.resolution = ::std::option::Option::Some(v);
    }

    pub fn get_resolution(&self) -> f64 {
        self.resolution.unwrap_or(0.)
    }
}

impl ::protobuf::Message for Meta {
    fn is_initialized(&self) -> bool {
        true
    }

    fn merge_from(
        &mut self,
        is: &mut ::protobuf::CodedInputStream,
    ) -> ::protobuf::ProtobufResult<()> {
        while !try!(is.eof()) {
            let (field_number, wire_type) = try!(is.read_tag_unpack());
            match field_number {
                1 => {
                    if wire_type != ::protobuf::wire_format::WireTypeVarint {
                        return ::std::result::Result::Err(
                            ::protobuf::rt::unexpected_wire_type(wire_type),
                        );
                    };
                    let tmp = try!(is.read_int32());
                    self.version = ::std::option::Option::Some(tmp);
                }
                2 => {
                    try!(
                        ::protobuf::rt::read_singular_message_into(
                            wire_type,
                            is,
                            &mut self.bounding_cube,
                        )
                    );
                }
                3 => {
                    if wire_type != ::protobuf::wire_format::WireTypeFixed64 {
                        return ::std::result::Result::Err(
                            ::protobuf::rt::unexpected_wire_type(wire_type),
                        );
                    };
                    let tmp = try!(is.read_double());
                    self.resolution = ::std::option::Option::Some(tmp);
                }
                _ => {
                    try!(
                        ::protobuf::rt::read_unknown_or_skip_group(
                            field_number,
                            wire_type,
                            is,
                            self.mut_unknown_fields(),
                        )
                    );
                }
            };
        }
        ::std::result::Result::Ok(())
    }

    // Compute sizes of nested messages
    #[allow(unused_variables)]
    fn compute_size(&self) -> u32 {
        let mut my_size = 0;
        for value in &self.version {
            my_size +=
                ::protobuf::rt::value_size(1, *value, ::protobuf::wire_format::WireTypeVarint);
        }
        for value in &self.bounding_cube {
            let len = value.compute_size();
            my_size += 1 + ::protobuf::rt::compute_raw_varint32_size(len) + len;
        }
        if self.resolution.is_some() {
            my_size += 9;
        };
        my_size += ::protobuf::rt::unknown_fields_size(self.get_unknown_fields());
        self.cached_size.set(my_size);
        my_size
    }

    fn write_to_with_cached_sizes(
        &self,
        os: &mut ::protobuf::CodedOutputStream,
    ) -> ::protobuf::ProtobufResult<()> {
        if let Some(v) = self.version {
            try!(os.write_int32(1, v));
        };
        if let Some(v) = self.bounding_cube.as_ref() {
            try!(os.write_tag(2, ::protobuf::wire_format::WireTypeLengthDelimited));
            try!(os.write_raw_varint32(v.get_cached_size()));
            try!(v.write_to_with_cached_sizes(os));
        };
        if let Some(v) = self.resolution {
            try!(os.write_double(3, v));
        };
        try!(os.write_unknown_fields(self.get_unknown_fields()));
        ::std::result::Result::Ok(())
    }

    fn get_cached_size(&self) -> u32 {
        self.cached_size.get()
    }

    fn get_unknown_fields(&self) -> &::protobuf::UnknownFields {
        &self.unknown_fields
    }

    fn mut_unknown_fields(&mut self) -> &mut ::protobuf::UnknownFields {
        &mut self.unknown_fields
    }

    fn type_id(&self) -> ::std::any::TypeId {
        ::std::any::TypeId::of::<Meta>()
    }

    fn as_any(&self) -> &::std::any::Any {
        self as &::std::any::Any
    }

    fn descriptor(&self) -> &'static ::protobuf::reflect::MessageDescriptor {
        ::protobuf::MessageStatic::descriptor_static(None::<Self>)
    }
}

impl ::protobuf::MessageStatic for Meta {
    fn new() -> Meta {
        Meta::new()
    }

    fn descriptor_static(_: ::std::option::Option<Meta>)
        -> &'static ::protobuf::reflect::MessageDescriptor {
        static mut descriptor: ::protobuf::lazy::Lazy<::protobuf::reflect::MessageDescriptor> =
            ::protobuf::lazy::Lazy {
                lock: ::protobuf::lazy::ONCE_INIT,
                ptr: 0 as *const ::protobuf::reflect::MessageDescriptor,
            };
        unsafe {
            descriptor.get(
                || {
                    let mut fields = ::std::vec::Vec::new();
                    fields.push(
                        ::protobuf::reflect::accessor::make_singular_i32_accessor(
                            "version",
                            Meta::has_version,
                            Meta::get_version,
                        )
                    );
                    fields.push(
                        ::protobuf::reflect::accessor::make_singular_message_accessor(
                            "bounding_cube",
                            Meta::has_bounding_cube,
                            Meta::get_bounding_cube,
                        )
                    );
                    fields.push(
                        ::protobuf::reflect::accessor::make_singular_f64_accessor(
                            "resolution",
                            Meta::has_resolution,
                            Meta::get_resolution,
                        )
                    );
                    ::protobuf::reflect::MessageDescriptor::new::<Meta>(
                        "Meta",
                        fields,
                        file_descriptor_proto(),
                    )
                }
            )
        }
    }
}

impl ::protobuf::Clear for Meta {
    fn clear(&mut self) {
        self.clear_version();
        self.clear_bounding_cube();
        self.clear_resolution();
        self.unknown_fields.clear();
    }
}

impl ::std::cmp::PartialEq for Meta {
    fn eq(&self, other: &Meta) -> bool {
        self.version == other.version && self.bounding_cube == other.bounding_cube &&
        self.resolution == other.resolution && self.unknown_fields == other.unknown_fields
    }
}

impl ::std::fmt::Debug for Meta {
    fn fmt(&self, f: &mut ::std::fmt::Formatter) -> ::std::fmt::Result {
        ::protobuf::text_format::fmt(self, f)
    }
}

#[derive(Clone,Default)]
pub struct Node {
    // message fields
    bounding_cube: ::protobuf::SingularPtrField<BoundingCube>,
    position_encoding: ::std::option::Option<Node_PositionEncoding>,
    num_points: ::std::option::Option<i64>,
    has_color: ::std::option::Option<bool>,
    // special fields
    unknown_fields: ::protobuf::UnknownFields,
    cached_size: ::std::cell::Cell<u32>,
}

// see codegen.rs for the explanation why impl Sync explicitly
unsafe impl ::std::marker::Sync for Node {}

impl Node {
    pub fn new() -> Node {
        ::std::default::Default::default()
    }

    pub fn default_instance() -> &'static Node {
        static mut instance: ::protobuf::lazy::Lazy<Node> = ::protobuf::lazy::Lazy {
            lock: ::protobuf::lazy::ONCE_INIT,
            ptr: 0 as *const Node,
        };
        unsafe {
            instance.get(
                || {
                    Node {
                        bounding_cube: ::protobuf::SingularPtrField::none(),
                        position_encoding: ::std::option::Option::None,
                        num_points: ::std::option::Option::None,
                        has_color: ::std::option::Option::None,
                        unknown_fields: ::protobuf::UnknownFields::new(),
                        cached_size: ::std::cell::Cell::new(0),
                    }
                }
            )
        }
    }

    // optional .point_cloud_viewer.proto.BoundingCube bounding_cube = 1;

    pub fn clear_bounding_cube(&mut self) {
        self.bounding_cube.clear();
    }

    pub fn has_bounding_cube(&self) -> bool {
        self.bounding_cube.is_some()
    }

    // Param is passed by value, moved
    pub fn set_bounding_cube(&mut self, v: BoundingCube) {
        self.bounding_cube = ::protobuf::SingularPtrField::some(v);
    }

    // Mutable pointer to the field.
    // If field is not initialized, it is initialized with default value first.
    pub fn mut_bounding_cube(&mut self) -> &mut BoundingCube {
        if self.bounding_cube.is_none() {
            self.bounding_cube.set_default();
        };
        self.bounding_cube.as_mut().unwrap()
    }

    // Take field
    pub fn take_bounding_cube(&mut self) -> BoundingCube {
        self.bounding_cube
            .take()
            .unwrap_or_else(|| BoundingCube::new())
    }

    pub fn get_bounding_cube(&self) -> &BoundingCube {
        self.bounding_cube
            .as_ref()
            .unwrap_or_else(|| BoundingCube::default_instance())
    }

    // optional .point_cloud_viewer.proto.Node.PositionEncoding position_encoding = 2;

    pub fn clear_position_encoding(&mut self) {
        self.position_encoding = ::std::option::Option::None;
    }

    pub fn has_position_encoding(&self) -> bool {
        self.position_encoding.is_some()
    }

    // Param is passed by value, moved
    pub fn set_position_encoding(&mut self, v: Node_PositionEncoding) {
        self.position_encoding = ::std::option::Option::Some(v);
    }

    pub fn get_position_encoding(&self) -> Node_PositionEncoding {
        self.position_encoding
            .unwrap_or(Node_PositionEncoding::Uint8)
    }

    // optional int64 num_points = 3;

    pub fn clear_num_points(&mut self) {
        self.num_points = ::std::option::Option::None;
    }

    pub fn has_num_points(&self) -> bool {
        self.num_points.is_some()
    }

    // Param is passed by value, moved
    pub fn set_num_points(&mut self, v: i64) {
        self.num_points = ::std::option::Option::Some(v);
    }

    pub fn get_num_points(&self) -> i64 {
        self.num_points.unwrap_or(0)
    }

    // optional bool has_color = 4;

    pub fn clear_has_color(&mut self) {
        self.has_color = ::std::option::Option::None;
    }

    pub fn has_has_color(&self) -> bool {
        self.has_color.is_some()
    }

    // Param is passed by value, moved
    pub fn set_has_color(&mut self, v: bool) {
        self.has_color = ::std::option::Option::Some(v);
    }

    pub fn get_has_color(&self) -> bool {
        self.has_color.unwrap_or(false)
    }
}

impl ::protobuf::Message for Node {
    fn is_initialized(&self) -> bool {
        true
    }

    fn merge_from(
        &mut self,
        is: &mut ::protobuf::CodedInputStream,
    ) -> ::protobuf::ProtobufResult<()> {
        while !try!(is.eof()) {
            let (field_number, wire_type) = try!(is.read_tag_unpack());
            match field_number {
                1 => {
                    try!(
                        ::protobuf::rt::read_singular_message_into(
                            wire_type,
                            is,
                            &mut self.bounding_cube,
                        )
                    );
                }
                2 => {
                    if wire_type != ::protobuf::wire_format::WireTypeVarint {
                        return ::std::result::Result::Err(
                            ::protobuf::rt::unexpected_wire_type(wire_type),
                        );
                    };
                    let tmp = try!(is.read_enum());
                    self.position_encoding = ::std::option::Option::Some(tmp);
                }
                3 => {
                    if wire_type != ::protobuf::wire_format::WireTypeVarint {
                        return ::std::result::Result::Err(
                            ::protobuf::rt::unexpected_wire_type(wire_type),
                        );
                    };
                    let tmp = try!(is.read_int64());
                    self.num_points = ::std::option::Option::Some(tmp);
                }
                4 => {
                    if wire_type != ::protobuf::wire_format::WireTypeVarint {
                        return ::std::result::Result::Err(
                            ::protobuf::rt::unexpected_wire_type(wire_type),
                        );
                    };
                    let tmp = try!(is.read_bool());
                    self.has_color = ::std::option::Option::Some(tmp);
                }
                _ => {
                    try!(
                        ::protobuf::rt::read_unknown_or_skip_group(
                            field_number,
                            wire_type,
                            is,
                            self.mut_unknown_fields(),
                        )
                    );
                }
            };
        }
        ::std::result::Result::Ok(())
    }

    // Compute sizes of nested messages
    #[allow(unused_variables)]
    fn compute_size(&self) -> u32 {
        let mut my_size = 0;
        for value in &self.bounding_cube {
            let len = value.compute_size();
            my_size += 1 + ::protobuf::rt::compute_raw_varint32_size(len) + len;
        }
        for value in &self.position_encoding {
            my_size += ::protobuf::rt::enum_size(2, *value);
        }
        for value in &self.num_points {
            my_size +=
                ::protobuf::rt::value_size(3, *value, ::protobuf::wire_format::WireTypeVarint);
        }
        if self.has_color.is_some() {
            my_size += 2;
        };
        my_size += ::protobuf::rt::unknown_fields_size(self.get_unknown_fields());
        self.cached_size.set(my_size);
        my_size
    }

    fn write_to_with_cached_sizes(
        &self,
        os: &mut ::protobuf::CodedOutputStream,
    ) -> ::protobuf::ProtobufResult<()> {
        if let Some(v) = self.bounding_cube.as_ref() {
            try!(os.write_tag(1, ::protobuf::wire_format::WireTypeLengthDelimited));
            try!(os.write_raw_varint32(v.get_cached_size()));
            try!(v.write_to_with_cached_sizes(os));
        };
        if let Some(v) = self.position_encoding {
            try!(os.write_enum(2, v.value()));
        };
        if let Some(v) = self.num_points {
            try!(os.write_int64(3, v));
        };
        if let Some(v) = self.has_color {
            try!(os.write_bool(4, v));
        };
        try!(os.write_unknown_fields(self.get_unknown_fields()));
        ::std::result::Result::Ok(())
    }

    fn get_cached_size(&self) -> u32 {
        self.cached_size.get()
    }

    fn get_unknown_fields(&self) -> &::protobuf::UnknownFields {
        &self.unknown_fields
    }

    fn mut_unknown_fields(&mut self) -> &mut ::protobuf::UnknownFields {
        &mut self.unknown_fields
    }

    fn type_id(&self) -> ::std::any::TypeId {
        ::std::any::TypeId::of::<Node>()
    }

    fn as_any(&self) -> &::std::any::Any {
        self as &::std::any::Any
    }

    fn descriptor(&self) -> &'static ::protobuf::reflect::MessageDescriptor {
        ::protobuf::MessageStatic::descriptor_static(None::<Self>)
    }
}

impl ::protobuf::MessageStatic for Node {
    fn new() -> Node {
        Node::new()
    }

    fn descriptor_static(_: ::std::option::Option<Node>)
        -> &'static ::protobuf::reflect::MessageDescriptor {
        static mut descriptor: ::protobuf::lazy::Lazy<::protobuf::reflect::MessageDescriptor> =
            ::protobuf::lazy::Lazy {
                lock: ::protobuf::lazy::ONCE_INIT,
                ptr: 0 as *const ::protobuf::reflect::MessageDescriptor,
            };
        unsafe {
            descriptor.get(
                || {
                    let mut fields = ::std::vec::Vec::new();
                    fields.push(
                        ::protobuf::reflect::accessor::make_singular_message_accessor(
                            "bounding_cube",
                            Node::has_bounding_cube,
                            Node::get_bounding_cube,
                        )
                    );
                    fields.push(
                        ::protobuf::reflect::accessor::make_singular_enum_accessor(
                            "position_encoding",
                            Node::has_position_encoding,
                            Node::get_position_encoding,
                        )
                    );
                    fields.push(
                        ::protobuf::reflect::accessor::make_singular_i64_accessor(
                            "num_points",
                            Node::has_num_points,
                            Node::get_num_points,
                        )
                    );
                    fields.push(
                        ::protobuf::reflect::accessor::make_singular_bool_accessor(
                            "has_color",
                            Node::has_has_color,
                            Node::get_has_color,
                        )
                    );
                    ::protobuf::reflect::MessageDescriptor::new::<Node>(
                        "Node",
                        fields,
                        file_descriptor_proto(),
                    )
                }
            )
        }
    }
}

impl ::protobuf::Clear for Node {
    fn clear(&mut self) {
        self.clear_bounding_cube();
        self.clear_position_encoding();
        self.clear_num_points();
        self.clear_has_color();
        self.unknown_fields.clear();
    }
}

impl ::std::cmp::PartialEq for Node {
    fn eq(&self, other: &Node) -> bool {
        self.bounding_cube == other.bounding_cube &&
        self.position_encoding == other.position_encoding &&
        self.num_points == other.num_points && self.has_color == other.has_color &&
        self.unknown_fields == other.unknown_fields
    }
}

impl ::std::fmt::Debug for Node {
    fn fmt(&self, f: &mut ::std::fmt::Formatter) -> ::std::fmt::Result {
        ::protobuf::text_format::fmt(self, f)
    }
}

#[derive(Clone,PartialEq,Eq,Debug,Hash)]
pub enum Node_PositionEncoding {
    Uint8 = 1,
    Uint16 = 2,
    Float32 = 3,
}

impl ::protobuf::ProtobufEnum for Node_PositionEncoding {
    fn value(&self) -> i32 {
        *self as i32
    }

    fn from_i32(value: i32) -> ::std::option::Option<Node_PositionEncoding> {
        match value {
            1 => ::std::option::Option::Some(Node_PositionEncoding::Uint8),
            2 => ::std::option::Option::Some(Node_PositionEncoding::Uint16),
            3 => ::std::option::Option::Some(Node_PositionEncoding::Float32),
            _ => ::std::option::Option::None,
        }
    }

    fn values() -> &'static [Self] {
        static values: &'static [Node_PositionEncoding] = &[
            Node_PositionEncoding::Uint8,
            Node_PositionEncoding::Uint16,
            Node_PositionEncoding::Float32,
        ];
        values
    }

    fn enum_descriptor_static(_: Option<Node_PositionEncoding>)
        -> &'static ::protobuf::reflect::EnumDescriptor {
        static mut descriptor: ::protobuf::lazy::Lazy<::protobuf::reflect::EnumDescriptor> =
            ::protobuf::lazy::Lazy {
                lock: ::protobuf::lazy::ONCE_INIT,
                ptr: 0 as *const ::protobuf::reflect::EnumDescriptor,
            };
        unsafe {
            descriptor.get(
                || {
                    ::protobuf::reflect::EnumDescriptor::new(
                        "Node_PositionEncoding",
                        file_descriptor_proto(),
                    )
                }
            )
        }
    }
}

impl ::std::marker::Copy for Node_PositionEncoding {}

static file_descriptor_proto_data: &'static [u8] = &[
    0x0a,
    0x0f,
    0x73,
    0x72,
    0x63,
    0x2f,
    0x70,
    0x72,
    0x6f,
    0x74,
    0x6f,
    0x2e,
    0x70,
    0x72,
    0x6f,
    0x74,
    0x6f,
    0x12,
    0x18,
    0x70,
    0x6f,
    0x69,
    0x6e,
    0x74,
    0x5f,
    0x63,
    0x6c,
    0x6f,
    0x75,
    0x64,
    0x5f,
    0x76,
    0x69,
    0x65,
    0x77,
    0x65,
    0x72,
    0x2e,
    0x70,
    0x72,
    0x6f,
    0x74,
    0x6f,
    0x22,
    0x2b,
    0x0a,
    0x08,
    0x56,
    0x65,
    0x63,
    0x74,
    0x6f,
    0x72,
    0x33,
    0x66,
    0x12,
    0x09,
    0x0a,
    0x01,
    0x78,
    0x18,
    0x01,
    0x20,
    0x01,
    0x28,
    0x02,
    0x12,
    0x09,
    0x0a,
    0x01,
    0x79,
    0x18,
    0x02,
    0x20,
    0x01,
    0x28,
    0x02,
    0x12,
    0x09,
    0x0a,
    0x01,
    0x7a,
    0x18,
    0x03,
    0x20,
    0x01,
    0x28,
    0x02,
    0x22,
    0x54,
    0x0a,
    0x0c,
    0x42,
    0x6f,
    0x75,
    0x6e,
    0x64,
    0x69,
    0x6e,
    0x67,
    0x43,
    0x75,
    0x62,
    0x65,
    0x12,
    0x2f,
    0x0a,
    0x03,
    0x6d,
    0x69,
    0x6e,
    0x18,
    0x01,
    0x20,
    0x01,
    0x28,
    0x0b,
    0x32,
    0x22,
    0x2e,
    0x70,
    0x6f,
    0x69,
    0x6e,
    0x74,
    0x5f,
    0x63,
    0x6c,
    0x6f,
    0x75,
    0x64,
    0x5f,
    0x76,
    0x69,
    0x65,
    0x77,
    0x65,
    0x72,
    0x2e,
    0x70,
    0x72,
    0x6f,
    0x74,
    0x6f,
    0x2e,
    0x56,
    0x65,
    0x63,
    0x74,
    0x6f,
    0x72,
    0x33,
    0x66,
    0x12,
    0x13,
    0x0a,
    0x0b,
    0x65,
    0x64,
    0x67,
    0x65,
    0x5f,
    0x6c,
    0x65,
    0x6e,
    0x67,
    0x74,
    0x68,
    0x18,
    0x02,
    0x20,
    0x01,
    0x28,
    0x02,
    0x22,
    0x6a,
    0x0a,
    0x04,
    0x4d,
    0x65,
    0x74,
    0x61,
    0x12,
    0x0f,
    0x0a,
    0x07,
    0x76,
    0x65,
    0x72,
    0x73,
    0x69,
    0x6f,
    0x6e,
    0x18,
    0x01,
    0x20,
    0x01,
    0x28,
    0x05,
    0x12,
    0x3d,
    0x0a,
    0x0d,
    0x62,
    0x6f,
    0x75,
    0x6e,
    0x64,
    0x69,
    0x6e,
    0x67,
    0x5f,
    0x63,
    0x75,
    0x62,
    0x65,
    0x18,
    0x02,
    0x20,
    0x01,
    0x28,
    0x0b,
    0x32,
    0x26,
    0x2e,
    0x70,
    0x6f,
    0x69,
    0x6e,
    0x74,
    0x5f,
    0x63,
    0x6c,
    0x6f,
    0x75,
    0x64,
    0x5f,
    0x76,
    0x69,
    0x65,
    0x77,
    0x65,
    0x72,
    0x2e,
    0x70,
    0x72,
    0x6f,
    0x74,
    0x6f,
    0x2e,
    0x42,
    0x6f,
    0x75,
    0x6e,
    0x64,
    0x69,
    0x6e,
    0x67,
    0x43,
    0x75,
    0x62,
    0x65,
    0x12,
    0x12,
    0x0a,
    0x0a,
    0x72,
    0x65,
    0x73,
    0x6f,
    0x6c,
    0x75,
    0x74,
    0x69,
    0x6f,
    0x6e,
    0x18,
    0x03,
    0x20,
    0x01,
    0x28,
    0x01,
    0x22,
    0xf0,
    0x01,
    0x0a,
    0x04,
    0x4e,
    0x6f,
    0x64,
    0x65,
    0x12,
    0x3d,
    0x0a,
    0x0d,
    0x62,
    0x6f,
    0x75,
    0x6e,
    0x64,
    0x69,
    0x6e,
    0x67,
    0x5f,
    0x63,
    0x75,
    0x62,
    0x65,
    0x18,
    0x01,
    0x20,
    0x01,
    0x28,
    0x0b,
    0x32,
    0x26,
    0x2e,
    0x70,
    0x6f,
    0x69,
    0x6e,
    0x74,
    0x5f,
    0x63,
    0x6c,
    0x6f,
    0x75,
    0x64,
    0x5f,
    0x76,
    0x69,
    0x65,
    0x77,
    0x65,
    0x72,
    0x2e,
    0x70,
    0x72,
    0x6f,
    0x74,
    0x6f,
    0x2e,
    0x42,
    0x6f,
    0x75,
    0x6e,
    0x64,
    0x69,
    0x6e,
    0x67,
    0x43,
    0x75,
    0x62,
    0x65,
    0x12,
    0x4a,
    0x0a,
    0x11,
    0x70,
    0x6f,
    0x73,
    0x69,
    0x74,
    0x69,
    0x6f,
    0x6e,
    0x5f,
    0x65,
    0x6e,
    0x63,
    0x6f,
    0x64,
    0x69,
    0x6e,
    0x67,
    0x18,
    0x02,
    0x20,
    0x01,
    0x28,
    0x0e,
    0x32,
    0x2f,
    0x2e,
    0x70,
    0x6f,
    0x69,
    0x6e,
    0x74,
    0x5f,
    0x63,
    0x6c,
    0x6f,
    0x75,
    0x64,
    0x5f,
    0x76,
    0x69,
    0x65,
    0x77,
    0x65,
    0x72,
    0x2e,
    0x70,
    0x72,
    0x6f,
    0x74,
    0x6f,
    0x2e,
    0x4e,
    0x6f,
    0x64,
    0x65,
    0x2e,
    0x50,
    0x6f,
    0x73,
    0x69,
    0x74,
    0x69,
    0x6f,
    0x6e,
    0x45,
    0x6e,
    0x63,
    0x6f,
    0x64,
    0x69,
    0x6e,
    0x67,
    0x12,
    0x12,
    0x0a,
    0x0a,
    0x6e,
    0x75,
    0x6d,
    0x5f,
    0x70,
    0x6f,
    0x69,
    0x6e,
    0x74,
    0x73,
    0x18,
    0x03,
    0x20,
    0x01,
    0x28,
    0x03,
    0x12,
    0x11,
    0x0a,
    0x09,
    0x68,
    0x61,
    0x73,
    0x5f,
    0x63,
    0x6f,
    0x6c,
    0x6f,
    0x72,
    0x18,
    0x04,
    0x20,
    0x01,
    0x28,
    0x08,
    0x22,
    0x36,
    0x0a,
    0x10,
    0x50,
    0x6f,
    0x73,
    0x69,
    0x74,
    0x69,
    0x6f,
    0x6e,
    0x45,
    0x6e,
    0x63,
    0x6f,
    0x64,
    0x69,
    0x6e,
    0x67,
    0x12,
    0x09,
    0x0a,
    0x05,
    0x55,
    0x69,
    0x6e,
    0x74,
    0x38,
    0x10,
    0x01,
    0x12,
    0x0a,
    0x0a,
    0x06,
    0x55,
    0x69,
    0x6e,
    0x74,
    0x31,
    0x36,
    0x10,
    0x02,
    0x12,
    0x0b,
    0x0a,
    0x07,
    0x46,
    0x6c,
    0x6f,
    0x61,
    0x74,
    0x33,
    0x32,
    0x10,
    0x03,
    0x4a,
    0xd7,
    0x08,
    0x0a,
    0x06,
    0x12,
    0x04,
    0x10,
    0x00,
    0x30,
    0x01,
    0x0a,
    0x08,
    0x0a,
    0x01,
    0x02,
    0x12,
    0x03,
    0x12,
    0x08,
    0x20,
    0x0a,
    0x0a,
    0x0a,
    0x02,
    0x04,
    0x00,
    0x12,
    0x04,
    0x14,
    0x00,
    0x18,
    0x01,
    0x0a,
    0x0a,
    0x0a,
    0x03,
    0x04,
    0x00,
    0x01,
    0x12,
    0x03,
    0x14,
    0x08,
    0x10,
    0x0a,
    0x0b,
    0x0a,
    0x04,
    0x04,
    0x00,
    0x02,
    0x00,
    0x12,
    0x03,
    0x15,
    0x02,
    0x17,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x00,
    0x02,
    0x00,
    0x04,
    0x12,
    0x03,
    0x15,
    0x02,
    0x0a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x00,
    0x02,
    0x00,
    0x05,
    0x12,
    0x03,
    0x15,
    0x0b,
    0x10,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x00,
    0x02,
    0x00,
    0x01,
    0x12,
    0x03,
    0x15,
    0x11,
    0x12,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x00,
    0x02,
    0x00,
    0x03,
    0x12,
    0x03,
    0x15,
    0x15,
    0x16,
    0x0a,
    0x0b,
    0x0a,
    0x04,
    0x04,
    0x00,
    0x02,
    0x01,
    0x12,
    0x03,
    0x16,
    0x02,
    0x17,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x00,
    0x02,
    0x01,
    0x04,
    0x12,
    0x03,
    0x16,
    0x02,
    0x0a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x00,
    0x02,
    0x01,
    0x05,
    0x12,
    0x03,
    0x16,
    0x0b,
    0x10,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x00,
    0x02,
    0x01,
    0x01,
    0x12,
    0x03,
    0x16,
    0x11,
    0x12,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x00,
    0x02,
    0x01,
    0x03,
    0x12,
    0x03,
    0x16,
    0x15,
    0x16,
    0x0a,
    0x0b,
    0x0a,
    0x04,
    0x04,
    0x00,
    0x02,
    0x02,
    0x12,
    0x03,
    0x17,
    0x02,
    0x17,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x00,
    0x02,
    0x02,
    0x04,
    0x12,
    0x03,
    0x17,
    0x02,
    0x0a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x00,
    0x02,
    0x02,
    0x05,
    0x12,
    0x03,
    0x17,
    0x0b,
    0x10,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x00,
    0x02,
    0x02,
    0x01,
    0x12,
    0x03,
    0x17,
    0x11,
    0x12,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x00,
    0x02,
    0x02,
    0x03,
    0x12,
    0x03,
    0x17,
    0x15,
    0x16,
    0x0a,
    0x0a,
    0x0a,
    0x02,
    0x04,
    0x01,
    0x12,
    0x04,
    0x1a,
    0x00,
    0x1d,
    0x01,
    0x0a,
    0x0a,
    0x0a,
    0x03,
    0x04,
    0x01,
    0x01,
    0x12,
    0x03,
    0x1a,
    0x08,
    0x14,
    0x0a,
    0x0b,
    0x0a,
    0x04,
    0x04,
    0x01,
    0x02,
    0x00,
    0x12,
    0x03,
    0x1b,
    0x02,
    0x1c,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x01,
    0x02,
    0x00,
    0x04,
    0x12,
    0x03,
    0x1b,
    0x02,
    0x0a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x01,
    0x02,
    0x00,
    0x06,
    0x12,
    0x03,
    0x1b,
    0x0b,
    0x13,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x01,
    0x02,
    0x00,
    0x01,
    0x12,
    0x03,
    0x1b,
    0x14,
    0x17,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x01,
    0x02,
    0x00,
    0x03,
    0x12,
    0x03,
    0x1b,
    0x1a,
    0x1b,
    0x0a,
    0x0b,
    0x0a,
    0x04,
    0x04,
    0x01,
    0x02,
    0x01,
    0x12,
    0x03,
    0x1c,
    0x02,
    0x21,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x01,
    0x02,
    0x01,
    0x04,
    0x12,
    0x03,
    0x1c,
    0x02,
    0x0a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x01,
    0x02,
    0x01,
    0x05,
    0x12,
    0x03,
    0x1c,
    0x0b,
    0x10,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x01,
    0x02,
    0x01,
    0x01,
    0x12,
    0x03,
    0x1c,
    0x11,
    0x1c,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x01,
    0x02,
    0x01,
    0x03,
    0x12,
    0x03,
    0x1c,
    0x1f,
    0x20,
    0x0a,
    0x0a,
    0x0a,
    0x02,
    0x04,
    0x02,
    0x12,
    0x04,
    0x1f,
    0x00,
    0x23,
    0x01,
    0x0a,
    0x0a,
    0x0a,
    0x03,
    0x04,
    0x02,
    0x01,
    0x12,
    0x03,
    0x1f,
    0x08,
    0x0c,
    0x0a,
    0x0b,
    0x0a,
    0x04,
    0x04,
    0x02,
    0x02,
    0x00,
    0x12,
    0x03,
    0x20,
    0x02,
    0x1d,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x02,
    0x02,
    0x00,
    0x04,
    0x12,
    0x03,
    0x20,
    0x02,
    0x0a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x02,
    0x02,
    0x00,
    0x05,
    0x12,
    0x03,
    0x20,
    0x0b,
    0x10,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x02,
    0x02,
    0x00,
    0x01,
    0x12,
    0x03,
    0x20,
    0x11,
    0x18,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x02,
    0x02,
    0x00,
    0x03,
    0x12,
    0x03,
    0x20,
    0x1b,
    0x1c,
    0x0a,
    0x0b,
    0x0a,
    0x04,
    0x04,
    0x02,
    0x02,
    0x01,
    0x12,
    0x03,
    0x21,
    0x02,
    0x2a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x02,
    0x02,
    0x01,
    0x04,
    0x12,
    0x03,
    0x21,
    0x02,
    0x0a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x02,
    0x02,
    0x01,
    0x06,
    0x12,
    0x03,
    0x21,
    0x0b,
    0x17,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x02,
    0x02,
    0x01,
    0x01,
    0x12,
    0x03,
    0x21,
    0x18,
    0x25,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x02,
    0x02,
    0x01,
    0x03,
    0x12,
    0x03,
    0x21,
    0x28,
    0x29,
    0x0a,
    0x0b,
    0x0a,
    0x04,
    0x04,
    0x02,
    0x02,
    0x02,
    0x12,
    0x03,
    0x22,
    0x02,
    0x21,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x02,
    0x02,
    0x02,
    0x04,
    0x12,
    0x03,
    0x22,
    0x02,
    0x0a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x02,
    0x02,
    0x02,
    0x05,
    0x12,
    0x03,
    0x22,
    0x0b,
    0x11,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x02,
    0x02,
    0x02,
    0x01,
    0x12,
    0x03,
    0x22,
    0x12,
    0x1c,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x02,
    0x02,
    0x02,
    0x03,
    0x12,
    0x03,
    0x22,
    0x1f,
    0x20,
    0x0a,
    0x0a,
    0x0a,
    0x02,
    0x04,
    0x03,
    0x12,
    0x04,
    0x25,
    0x00,
    0x30,
    0x01,
    0x0a,
    0x0a,
    0x0a,
    0x03,
    0x04,
    0x03,
    0x01,
    0x12,
    0x03,
    0x25,
    0x08,
    0x0c,
    0x0a,
    0x0c,
    0x0a,
    0x04,
    0x04,
    0x03,
    0x04,
    0x00,
    0x12,
    0x04,
    0x26,
    0x02,
    0x2a,
    0x03,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x04,
    0x00,
    0x01,
    0x12,
    0x03,
    0x26,
    0x07,
    0x17,
    0x0a,
    0x0d,
    0x0a,
    0x06,
    0x04,
    0x03,
    0x04,
    0x00,
    0x02,
    0x00,
    0x12,
    0x03,
    0x27,
    0x04,
    0x0e,
    0x0a,
    0x0e,
    0x0a,
    0x07,
    0x04,
    0x03,
    0x04,
    0x00,
    0x02,
    0x00,
    0x01,
    0x12,
    0x03,
    0x27,
    0x04,
    0x09,
    0x0a,
    0x0e,
    0x0a,
    0x07,
    0x04,
    0x03,
    0x04,
    0x00,
    0x02,
    0x00,
    0x02,
    0x12,
    0x03,
    0x27,
    0x0c,
    0x0d,
    0x0a,
    0x0d,
    0x0a,
    0x06,
    0x04,
    0x03,
    0x04,
    0x00,
    0x02,
    0x01,
    0x12,
    0x03,
    0x28,
    0x04,
    0x0f,
    0x0a,
    0x0e,
    0x0a,
    0x07,
    0x04,
    0x03,
    0x04,
    0x00,
    0x02,
    0x01,
    0x01,
    0x12,
    0x03,
    0x28,
    0x04,
    0x0a,
    0x0a,
    0x0e,
    0x0a,
    0x07,
    0x04,
    0x03,
    0x04,
    0x00,
    0x02,
    0x01,
    0x02,
    0x12,
    0x03,
    0x28,
    0x0d,
    0x0e,
    0x0a,
    0x0d,
    0x0a,
    0x06,
    0x04,
    0x03,
    0x04,
    0x00,
    0x02,
    0x02,
    0x12,
    0x03,
    0x29,
    0x04,
    0x10,
    0x0a,
    0x0e,
    0x0a,
    0x07,
    0x04,
    0x03,
    0x04,
    0x00,
    0x02,
    0x02,
    0x01,
    0x12,
    0x03,
    0x29,
    0x04,
    0x0b,
    0x0a,
    0x0e,
    0x0a,
    0x07,
    0x04,
    0x03,
    0x04,
    0x00,
    0x02,
    0x02,
    0x02,
    0x12,
    0x03,
    0x29,
    0x0e,
    0x0f,
    0x0a,
    0x0b,
    0x0a,
    0x04,
    0x04,
    0x03,
    0x02,
    0x00,
    0x12,
    0x03,
    0x2c,
    0x02,
    0x2a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x00,
    0x04,
    0x12,
    0x03,
    0x2c,
    0x02,
    0x0a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x00,
    0x06,
    0x12,
    0x03,
    0x2c,
    0x0b,
    0x17,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x00,
    0x01,
    0x12,
    0x03,
    0x2c,
    0x18,
    0x25,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x00,
    0x03,
    0x12,
    0x03,
    0x2c,
    0x28,
    0x29,
    0x0a,
    0x0b,
    0x0a,
    0x04,
    0x04,
    0x03,
    0x02,
    0x01,
    0x12,
    0x03,
    0x2d,
    0x02,
    0x32,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x01,
    0x04,
    0x12,
    0x03,
    0x2d,
    0x02,
    0x0a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x01,
    0x06,
    0x12,
    0x03,
    0x2d,
    0x0b,
    0x1b,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x01,
    0x01,
    0x12,
    0x03,
    0x2d,
    0x1c,
    0x2d,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x01,
    0x03,
    0x12,
    0x03,
    0x2d,
    0x30,
    0x31,
    0x0a,
    0x0b,
    0x0a,
    0x04,
    0x04,
    0x03,
    0x02,
    0x02,
    0x12,
    0x03,
    0x2e,
    0x02,
    0x20,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x02,
    0x04,
    0x12,
    0x03,
    0x2e,
    0x02,
    0x0a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x02,
    0x05,
    0x12,
    0x03,
    0x2e,
    0x0b,
    0x10,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x02,
    0x01,
    0x12,
    0x03,
    0x2e,
    0x11,
    0x1b,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x02,
    0x03,
    0x12,
    0x03,
    0x2e,
    0x1e,
    0x1f,
    0x0a,
    0x0b,
    0x0a,
    0x04,
    0x04,
    0x03,
    0x02,
    0x03,
    0x12,
    0x03,
    0x2f,
    0x02,
    0x1e,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x03,
    0x04,
    0x12,
    0x03,
    0x2f,
    0x02,
    0x0a,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x03,
    0x05,
    0x12,
    0x03,
    0x2f,
    0x0b,
    0x0f,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x03,
    0x01,
    0x12,
    0x03,
    0x2f,
    0x10,
    0x19,
    0x0a,
    0x0c,
    0x0a,
    0x05,
    0x04,
    0x03,
    0x02,
    0x03,
    0x03,
    0x12,
    0x03,
    0x2f,
    0x1c,
    0x1d,
];

static mut file_descriptor_proto_lazy: ::protobuf::lazy::Lazy<::protobuf::descriptor::FileDescriptorProto> = ::protobuf::lazy::Lazy {
    lock: ::protobuf::lazy::ONCE_INIT,
    ptr: 0 as *const ::protobuf::descriptor::FileDescriptorProto,
};

fn parse_descriptor_proto() -> ::protobuf::descriptor::FileDescriptorProto {
    ::protobuf::parse_from_bytes(file_descriptor_proto_data).unwrap()
}

pub fn file_descriptor_proto() -> &'static ::protobuf::descriptor::FileDescriptorProto {
    unsafe { file_descriptor_proto_lazy.get(|| parse_descriptor_proto()) }
}
