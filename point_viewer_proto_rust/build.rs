fn main() {
    proto_helper::build_script::ProtoBuilder::new()
        .protoc_search_path("../target/protobuf/bin")
        .add_file("src/proto.proto")
        .run();
}
