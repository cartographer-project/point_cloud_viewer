fn main() {
    proto_helper::ProtoBuilder::new()
        .protoc_search_path("../target/protobuf/bin")
        .add_file("src/proto.proto")
        .add_transformation(proto_helper::wrap_in_module("proto"))
        .run();
}
