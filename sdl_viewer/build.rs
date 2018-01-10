extern crate gl_generator;

use gl_generator::{Api, Fallbacks, StructGenerator, Profile, Registry};
use std::env;
use std::fs::File;
use std::path::Path;

fn main() {
    let dest = env::var("OUT_DIR").unwrap();
    let mut file = File::create(&Path::new(&dest).join("bindings.rs")).unwrap();

    Registry::new(Api::Gl, (3, 2), Profile::Core, Fallbacks::All, [])
        .write_bindings(StructGenerator, &mut file)
        .unwrap();
}
