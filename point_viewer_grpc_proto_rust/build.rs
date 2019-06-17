fn main() {
    proto_helper::ProtoBuilder::new()
        .protoc_search_path("../target/protobuf/bin")
        .add_file("src/proto.proto")
        .add_import_path("..")
        .grpc(true)
        .add_transformation(Box::new(|c: String| {
            c.replace("super::proto", "::point_viewer::proto")
        }))
        .add_transformation(proto_helper::wrap_in_module("proto"))
        .add_grpc_transformation(proto_helper::wrap_in_module("proto_grpc"))
        .run();
}
