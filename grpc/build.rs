use std::env;
use std::fs::File;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process;

// Finds the absolute path to the root of the repository.
fn find_git_repo_root() -> PathBuf {
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let mut path = Path::new(&manifest_dir);
    while !path.join(".git").exists() {
        path = path.parent().unwrap();
    }
    return path.to_owned();
}

// Finds 'exe_name' in $PATH and returns its full path.
// From https://stackoverflow.com/a/37499032.
fn find_executable<P>(exe_name: P) -> Option<PathBuf>
where
    P: AsRef<Path>,
{
    env::var_os("PATH").and_then(|paths| {
        env::split_paths(&paths)
            .filter_map(|dir| {
                let full_path = dir.join(&exe_name);
                if full_path.is_file() {
                    Some(full_path)
                } else {
                    None
                }
            })
            .next()
    })
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

    println!("cargo:rerun-if-changed=src/proto.proto");

    let mut cmd = process::Command::new("protoc");
    let plugin = find_executable("grpc_rust_plugin")
        .unwrap()
        .to_string_lossy()
        .into_owned();

    let root_path = find_git_repo_root();
    cmd.stdin(process::Stdio::null());
    cmd.args(&[
        format!("-I{}", root_path.to_string_lossy()),
        format!("--rust_out={}", out_dir),
        format!("--grpc_out={}", out_dir),
        format!("--plugin=protoc-gen-grpc={}", plugin),
        root_path
            .join("grpc/src/proto.proto")
            .to_string_lossy()
            .into_owned(),
    ]);

    let mut child = cmd.spawn().unwrap();
    if !child.wait().unwrap().success() {
        panic!("protoc exited with non-zero exit code");
    }

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
