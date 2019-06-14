pub mod build_script {

    use std::fs::File;
    use std::io::{Read, Write};
    use std::path::Path;

    pub struct ProtoBuilder<'a> {
        protoc_search_path: Option<&'a Path>,
        files: Vec<&'a Path>,
        generate_grpc: bool,
    }

    impl<'a> ProtoBuilder<'a> {
        // The given path to protoc can be relative to the build script.
        pub fn new(protoc_search_path: Option<&'a Path>) -> Self {
            Self {
                protoc_search_path,
                files: vec![],
                generate_grpc: false,
            }
        }

        // Register a file for compilation. The path must be relative to the build script.
        pub fn file(&mut self, file_path: &'a Path) -> &mut Self {
            self.files.push(file_path);
            self
        }

        pub fn run(self) {
            let old_path = std::env::var("PATH");
            if let Some(search_path) = self.protoc_search_path {
                std::env::set_var("PATH", search_path);
            }
            self.files.iter().for_each(|file| {
                println!("cargo:rerun-if-changed={}", file.display());
                compile_proto(file, self.generate_grpc);
            });
            std::env::set_var("PATH", old_path.unwrap_or("".to_string()));
        }
    }

    // Compile a protobuf. The filepath is relative to the crate root.
    fn compile_proto(protobuf_file: &Path, generate_grpc: bool) {
        // Create parent directories
        let in_dir = protobuf_file.parent().unwrap();
        std::fs::create_dir_all(in_dir).expect("Could not create dir");

        // There are different options here:
        // protoc-rust (recommended): Invokes protoc programmatically
        // protobuf-codegen-pure (Alpha): Pure rust protobuf parser and code generator
        // protoc: General-purpose wrapper around protoc, could be used to generate CPP code
        // For gRPC:
        // protoc-rust-grpc (in the protoc-grpc-rust repo): Also invokes protoc programmatically
        // protoc-grpcio (in the protoc-grpcio repo)

        let out_dir: String = std::env::var("OUT_DIR").unwrap();
        let out_dir: &Path = Path::new(&out_dir);
        let protobuf_name: &Path = protobuf_file.file_stem().unwrap().as_ref();
        let includes: &[&str] = &[];

        if generate_grpc {
            // This generates both protobuf and grpc definitions
            protoc_grpcio::compile_grpc_protos(&[protobuf_file], includes, out_dir)
                .expect("Error running protoc (with grpc)");
            let out_path = out_dir.with_file_name(format!("{}_grpc.rs", protobuf_name.display()));
            inplace_modify_file(&out_path, |code| wrap_in_module(code, "grpc"));
        } else {
            let args = protoc_rust::Args {
                out_dir: out_dir.to_str().unwrap(),
                includes: includes,
                input: &[protobuf_file.to_str().unwrap()],
                ..Default::default()
            };
            protoc_rust::run(args).expect("Error running protoc");
            let out_path = out_dir.with_file_name(format!("{}.rs", protobuf_name.display()));
            inplace_modify_file(&out_path, |code| {
            	let code = replace_clippy(code);
            	wrap_in_module(code, "proto")
            });
        }
    }

    // This is a workaround for https://github.com/stepancheg/rust-protobuf/issues/331
    fn replace_clippy(contents: String) -> String {
        contents.replace("#![allow(clippy)]", "#![allow(clippy::all)]")
    }

    // Work around
    // https://github.com/stepancheg/rust-protobuf/issues/117
    // https://github.com/rust-lang/rust/issues/18810.
    // We open the file, add 'mod proto { }' around the contents and write it back. This allows us
    // to include! the file in lib.rs and have a proper proto module.
    fn wrap_in_module(contents: String, mod_name: &str) -> String {
        format!("pub mod {} {{\n{}\n}}", mod_name, contents)
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

}

pub mod library {
    // Macro to include protobuf from path relative to crate root
}
