use quadtree::NodeId;
use std::collections::HashSet;
use std::io::Cursor;
use std::path::{Path, PathBuf};
use structopt::StructOpt;
use xray_proto_rust::proto;

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

fn read_metadata(path: &Path) -> proto::Meta {
    let data = std::fs::read(&path).expect("Cannot open file");
    protobuf::parse_from_reader::<proto::Meta>(&mut Cursor::new(data))
        .expect("Could not read meta proto")
}

fn read_metadata_from_directory(directory: &Path) -> Vec<proto::Meta> {
    globwalk::GlobWalkerBuilder::from_patterns(directory, &["meta*.pb"])
        .build()
        .expect("Failed to build GlobWalker")
        .into_iter()
        .filter_map(Result::ok)
        .map(|dir_entry| read_metadata(dir_entry.path()))
        .collect()
}

fn read_metadata_from_directories(directories: &Vec<PathBuf>) -> Vec<proto::Meta> {
    directories
        .iter()
        .map(|directory| read_metadata_from_directory(&directory))
        .flatten()
        .collect()
}

fn get_root_node(meta: &proto::Meta) -> Option<NodeId> {
    meta.get_nodes()
        .iter()
        .map(NodeId::from)
        .find(|node| node.parent_id().is_none())
}

fn get_root_nodes(meta: &Vec<proto::Meta>) -> Option<Vec<NodeId>> {
    meta.iter().map(get_root_node).collect()
}

fn validate_metadata(metadata: &Vec<proto::Meta>) -> Vec<NodeId> {
    let root_nodes = get_root_nodes(metadata).expect("One of the quadtrees is empty.");
    let mut levels = HashSet::new();
    let mut indices = HashSet::new();
    for root_node in &root_nodes {
        levels.insert(root_node.level());
        indices.insert(root_node.index());
    }
    assert_eq!(levels.len(), 1, "Not all roots have the same levels.");
    assert_eq!(
        indices.len(),
        root_nodes.len(),
        "Not all roots have unique indices."
    );
    root_nodes
}

fn main() {
    let args = CommandlineArguments::from_args();
    let metadata = read_metadata_from_directories(&args.input_directories);
    let root_nodes = validate_metadata(&metadata);
    unimplemented!();
}
