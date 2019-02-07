extern crate protoc_grpcio;

use std::env;
use std::fs::File;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

// Finds the absolute path to the root of the repository.
fn find_git_repo_root() -> PathBuf {
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let mut path = Path::new(&manifest_dir);
    while !path.join(".git").exists() {
        path = path.parent().unwrap();
    }
    return path.to_owned();
}

// Opens a file, calls 'func' with its contents and writes the new content back.
fn inplace_modify_file<F: FnOnce(String) -> String>(path: &Path, func: F) {
    let mut contents = String::new();
    File::open(&path)
        .unwrap()
        .read_to_string(&mut contents)
        .unwrap();
    let new_contents = func(contents);
    File::create(&path)
        .unwrap()
        .write_all(new_contents.as_bytes())
        .unwrap();
}

fn wrap_in_module(contents: String, mod_name: &str) -> String {
    // TODO(sirver): Replicated from root crate. Pull out a build-proto crate?
    // Work around
    // https://github.com/stepancheg/rust-protobuf/issues/117
    // https://github.com/rust-lang/rust/issues/18810.
    // We open the file, add 'mod proto { }' around the contents and write it back. This allows us
    // to include! the file in lib.rs and have a proper proto module.
    format!("pub mod {} {{\n{}\n}}", mod_name, contents)
}

fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();

    println!("cargo:rerun-if-changed=point_viewer_grpc_proto_rust/src/proto.proto");

    let git_repo_root = find_git_repo_root();
    protoc_grpcio::compile_grpc_protos(
        &["point_viewer_grpc_proto_rust/src/proto.proto"],
        &[git_repo_root.clone()],
        &out_dir,
    )
    .expect("Failed to compile gRPC definitions!");

    inplace_modify_file(&Path::new(&out_dir).join("proto.rs"), |c| {
        // Work around https://github.com/stepancheg/rust-protobuf/issues/260. The protobuf plugin
        // apparently gets confused that there is point_viewer::octree::proto and
        // point_viewer_grpc::proto and believes the two proto namespaces are the same. We fully
        // qualify the paths to avoid this issue.
        let new_content = c.replace("super::proto", "::point_viewer::proto");
        wrap_in_module(new_content, "proto")
    });
    inplace_modify_file(&Path::new(&out_dir).join("proto_grpc.rs"), |c| {
        wrap_in_module(c, "proto_grpc")
    });
}
