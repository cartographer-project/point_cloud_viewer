use std::path::PathBuf;
use structopt::StructOpt;

#[derive(StructOpt, Debug)]
#[structopt(name = "merge_xray_quadtrees")]
/// Merge partial xray quadtrees. We assume that the root
/// of each quadtree belongs to the same level of the final
/// quadtree.
struct CommandlineArguments {
    /// Directory where to write the merged quadtree. Does *not*
    /// have to be disjoint from input_directories.
    #[structopt(parse(from_os_str), long = "output_directory")]
    output_directory: PathBuf,
    /// Directories with, possibly multiple, partial xray quadtrees.
    #[structopt(parse(from_os_str))]
    input_directories: Vec<PathBuf>,
}

fn main() {
    let args = CommandlineArguments::from_args();
    dbg!(args);
    unimplemented!();
}
