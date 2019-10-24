use gl_generator::{Api, Fallbacks, Profile, Registry, StructGenerator};
use std::env;
use std::fs::File;
use std::path::Path;

fn main() {
    let dest = env::var("OUT_DIR").unwrap();
    let mut file = File::create(&Path::new(&dest).join("bindings.rs")).unwrap();

    Registry::new(Api::Gl, (4, 1), Profile::Core, Fallbacks::All, [])
        .write_bindings(StructGenerator, &mut file)
        .unwrap();

    // See https://github.com/rust-lang/rust-clippy/pull/4535
    if version_check::is_min_date("2019-09-19").unwrap_or(false) {
        println!("cargo:rustc-cfg=clippy_has_missing_safety_doc");
    }
}
