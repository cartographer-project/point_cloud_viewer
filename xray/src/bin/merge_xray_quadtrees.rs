use std::path::{PathBuf};
use structopt::StructOpt;

#[derive(StructOpt, Debug)]
#[structopt(name = "merge_xray_quadtrees")]
/// Merge partial xray octrees. We assume that the root
/// of each octree belongs to the same level of the final
/// octree. 
struct CommandlineArguments {
    /// Directory with partial xray quadtrees.
    #[structopt(parse(from_os_str))]
    directory: PathBuf,
    /*
    /// Paths to each quadtree's metadata protobuf file.
    metadata_proto_filenames: Vector<Pathbuf>
    */
}

fn main() {
    let _args = CommandlineArguments::from_args();
    unimplemented!();
}