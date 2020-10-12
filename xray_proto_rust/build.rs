use protobuf_provider::PROTOBUF_BIN_PATH;
use std::env;
use std::fs::File;
use std::io::{Read, Write};
use std::path::Path;

fn main() {
    println!("cargo:rerun-if-changed=src/proto.proto");

    let out_dir = env::var("OUT_DIR").unwrap();
    protoc_rust::Codegen::new()
        .protoc_path(Path::new(PROTOBUF_BIN_PATH).join("protoc"))
        .out_dir(&out_dir)
        .input("src/proto.proto")
        .run()
        .expect("protoc");

    // Work around
    // https://github.com/stepancheg/rust-protobuf/issues/117
    // https://github.com/rust-lang/rust/issues/18810.
    // We open the file, add 'mod proto { }' around the contents and write it back. This allows us
    // to include! the file in lib.rs and have a proper proto module.
    let proto_path = Path::new(&out_dir).join("proto.rs");
    let mut contents = String::new();
    File::open(&proto_path)
        .unwrap()
        .read_to_string(&mut contents)
        .unwrap();
    let new_contents = format!(
        "#[rustfmt::skip]\npub mod proto {{\n{}\n}}",
        contents.replace("#![rustfmt::skip]\n", "")
    );

    File::create(&proto_path)
        .unwrap()
        .write_all(new_contents.as_bytes())
        .unwrap();
}
