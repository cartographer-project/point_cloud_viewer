use std::fs::File;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

#[derive(Default)]
pub struct ProtoBuilder {
    protoc_search_path: Option<PathBuf>,
    files: Vec<PathBuf>,
    import_paths: Vec<PathBuf>,
    generate_grpc: bool,
    transformations: Vec<Box<Fn(String) -> String>>,
    grpc_transformations: Vec<Box<Fn(String) -> String>>,
}

impl ProtoBuilder {
    // The path can be relative to the build script.
    pub fn new() -> Self {
        Self {
            protoc_search_path: None,
            files: vec![],
            import_paths: vec![],
            generate_grpc: false,
            transformations: vec![],
            grpc_transformations: vec![],
        }
    }

    // The path to the directory where protoc is located. If this is not set, PATH is used.
    pub fn protoc_search_path<P: AsRef<Path>>(&mut self, protoc_search_path: P) -> &mut Self {
        self.protoc_search_path = Some(protoc_search_path.as_ref().to_owned());
        self
    }

    // Register a file for compilation. The path must be relative to the build script.
    pub fn add_file<P: AsRef<Path>>(&mut self, file_path: P) -> &mut Self {
        self.files.push(file_path.as_ref().to_owned());
        self
    }

    pub fn add_import_path<P: AsRef<Path>>(&mut self, import_path: P) -> &mut Self {
        self.import_paths.push(import_path.as_ref().to_owned());
        self
    }

    pub fn grpc(&mut self, generate_grpc: bool) -> &mut Self {
        self.generate_grpc = generate_grpc;
        self
    }

    pub fn add_transformation(&mut self, f: Box<Fn(String) -> String>) -> &mut Self {
        self.transformations.push(f);
        self
    }

    pub fn add_grpc_transformation(&mut self, f: Box<Fn(String) -> String>) -> &mut Self {
        self.grpc_transformations.push(f);
        self
    }

    pub fn run(&mut self) {
        let old_path = std::env::var("PATH");
        if let Some(search_path) = &self.protoc_search_path {
            std::env::set_var("PATH", search_path);
        }
        for imp in self.import_paths.iter_mut() {
            std::mem::replace(imp, std::fs::canonicalize(&imp).unwrap());
        }
        self.files.iter().for_each(|file| {
            println!("cargo:rerun-if-changed={}", file.display());
            compile_proto(
                &file,
                &self.import_paths,
                self.generate_grpc,
                &self.transformations,
                &self.grpc_transformations,
            );
        });
        let _ = old_path.map(|pth| std::env::set_var("PATH", pth));
    }
}

// Compile a protobuf. The filepath is relative to the crate root.
fn compile_proto(
    protobuf_file: &Path,
    import_paths: &[PathBuf],
    generate_grpc: bool,
    transformations: &[Box<Fn(String) -> String>],
    grpc_transformations: &[Box<Fn(String) -> String>],
) {
    // Create parent directories
    let in_dir = protobuf_file.parent().unwrap();
    let out_dir: String = std::env::var("OUT_DIR").unwrap();
    let out_dir: &Path = &Path::new(&out_dir).join(in_dir);
    std::fs::create_dir_all(out_dir).expect("Could not create dir");

    // There are different options here:
    // protoc-rust (recommended): Invokes protoc programmatically
    // protobuf-codegen-pure (Alpha): Pure rust protobuf parser and code generator
    // protoc: General-purpose wrapper around protoc, could be used to generate CPP code
    // For gRPC:
    // protoc-rust-grpc (in the protoc-grpc-rust repo): Also invokes protoc programmatically
    // protoc-grpcio (in the protoc-grpcio repo)

    let protobuf_name: &Path = protobuf_file.file_stem().unwrap().as_ref();
    let includes: Vec<&str> = import_paths.iter().map(|p| p.to_str().unwrap()).collect();

    if generate_grpc {
        // This generates both protobuf and grpc definitions
        protoc_grpcio::compile_grpc_protos(&[protobuf_file], includes.as_slice(), out_dir)
            .expect("Error running protoc (with grpc)");
        let out_path = out_dir.join(format!("{}.rs", protobuf_name.display()));
        inplace_modify_file(&out_path, |code| {
            let code = replace_clippy(code);
            transformations.iter().fold(code, |code, tr| tr(code))
        });
        let grpc_out_path = out_dir.join(format!("{}_grpc.rs", protobuf_name.display()));
        inplace_modify_file(&grpc_out_path, |code| {
            let code = replace_clippy(code);
            grpc_transformations.iter().fold(code, |code, tr| tr(code))
        });
    } else {
        let args = protoc_rust::Args {
            out_dir: out_dir.to_str().unwrap(),
            includes: includes.as_slice(),
            input: &[protobuf_file.to_str().unwrap()],
            ..Default::default()
        };
        protoc_rust::run(args).expect("Error running protoc");
        let out_path = out_dir.join(format!("{}.rs", protobuf_name.display()));
        inplace_modify_file(&out_path, |code| {
            let code = replace_clippy(code);
            transformations.iter().fold(code, |code, tr| tr(code))
        });
    }
}

// Work around
// https://github.com/stepancheg/rust-protobuf/issues/117
// https://github.com/rust-lang/rust/issues/18810.
// We open the file, add 'mod proto { }' around the contents and write it back. This allows us
// to include! the file in lib.rs and have a proper proto module.
pub fn wrap_in_module(mod_name: &'static str) -> Box<Fn(String) -> String> {
    Box::new(move |code| format!("pub mod {} {{\n{}\n}}", mod_name, code))
}

// This is a workaround for https://github.com/stepancheg/rust-protobuf/issues/331
fn replace_clippy(contents: String) -> String {
    contents.replace("#![allow(clippy)]", "#![allow(clippy::all)]")
}

// Opens a file, calls 'func' with its contents and writes the new content back.
fn inplace_modify_file<F: FnOnce(String) -> String>(path: &Path, func: F) {
    let mut contents = String::new();
    File::open(&path)
        .unwrap_or_else(|_| panic!("Could not open {}", path.display()))
        .read_to_string(&mut contents)
        .unwrap();
    let new_contents = func(contents);
    File::create(&path)
        .unwrap()
        .write_all(new_contents.as_bytes())
        .unwrap();
}
