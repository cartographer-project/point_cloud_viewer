use fnv::FnvHashSet;
use point_viewer::color::Color;
use protobuf::Message;
use quadtree::NodeId;
use std::fs::File;
use std::io::{BufWriter, Cursor};
use std::path::{Path, PathBuf};
use structopt::StructOpt;
use xray::generation;
use xray_proto_rust::proto;

#[derive(StructOpt, Debug)]
#[structopt(name = "merge_xray_quadtrees")]
/// Merge partial xray quadtrees. We assume that the root
/// of each quadtree belongs to the same level of the final
/// quadtree.
struct CommandlineArguments {
    /// Directory where to write the merged quadtree. Does *not*
    /// have to be disjoint from input_directories.
    #[structopt(parse(from_os_str), long)]
    output_directory: PathBuf,
    /// Tile background color.
    #[structopt(default_value = "white", long)]
    tile_background_color: generation::TileBackgroundColorArgument,
    /// Directories with, possibly multiple, partial xray quadtrees.
    #[structopt(parse(from_os_str))]
    input_directories: Vec<PathBuf>,
}

fn copy_images(input_directory: &Path, output_directory: &Path) {
    if input_directory == output_directory {
        return;
    }
    globwalk::GlobWalkerBuilder::new(
        input_directory,
        &format!("*.{}", xray::IMAGE_FILE_EXTENSION),
    )
    .build()
    .expect("Failed to build GlobWalker")
    .filter_map(Result::ok)
    .for_each(|dir_entry| {
        std::fs::copy(
            dir_entry.path(),
            output_directory.join(dir_entry.file_name()),
        )
        .expect("Failed to copy the file.");
    })
}

fn copy_all_images(input_directories: &Vec<PathBuf>, output_directory: &Path) {
    for input_directory in input_directories {
        copy_images(input_directory, output_directory);
    }
}

fn read_metadata(path: &Path) -> proto::Meta {
    let data = std::fs::read(&path).expect("Cannot open file");
    protobuf::parse_from_reader::<proto::Meta>(&mut Cursor::new(data))
        .expect("Could not read meta proto")
}

fn read_metadata_from_directory(directory: &Path) -> Vec<proto::Meta> {
    globwalk::GlobWalkerBuilder::new(directory, "meta*.pb")
        .build()
        .expect("Failed to build GlobWalker")
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
        .min_by_key(|node| node.level())
}

fn get_root_nodes(meta: &Vec<proto::Meta>) -> Option<FnvHashSet<NodeId>> {
    meta.iter().map(get_root_node).collect()
}

struct Metadata {
    root_nodes: FnvHashSet<NodeId>,
    nodes: FnvHashSet<NodeId>,
    level: u8,
    deepest_level: u8,
    tile_size: u32,
    bounding_rect: proto::Rect,
}

// There must be at least one element in the iterator.
fn check_all_the_same<I, V>(mut iterator: I) -> Option<V>
where
    I: Iterator<Item = V>,
    V: std::cmp::PartialEq,
{
    let first = iterator.next().expect("Empty iterator");
    if iterator.all(|element| element == first) {
        Some(first)
    } else {
        None
    }
}

fn validate_metadata(metadata: &Vec<proto::Meta>) -> Metadata {
    assert!(!metadata.is_empty(), "No meta.pb files found.");
    let root_nodes = get_root_nodes(metadata).expect("One of the quadtrees is empty.");
    assert_eq!(
        metadata.len(),
        root_nodes.len(),
        "Not all roots are unique."
    );
    let version = check_all_the_same(metadata.iter().map(|meta| meta.get_version()))
        .expect("Not all meta files have the same version.");
    assert_eq!(version, xray::CURRENT_VERSION);
    let level = check_all_the_same(root_nodes.iter().map(|node| node.level()))
        .expect("Not all roots have the same level.");
    let deepest_level = check_all_the_same(metadata.iter().map(|meta| meta.get_deepest_level()))
        .expect("Not all meta files have the same deepest level.") as u8;
    let tile_size = check_all_the_same(metadata.iter().map(|meta| meta.get_tile_size()))
        .expect("Not all meta files have the same tile size.");
    // TODO: check whether all root nodes have the same bound_rect.
    let bounding_rect = metadata[0].get_bounding_rect().clone();

    let mut nodes = FnvHashSet::default();
    for meta in metadata {
        nodes.extend(meta.get_nodes().iter().map(|proto| NodeId::from(proto)));
    }

    Metadata {
        root_nodes,
        level,
        deepest_level,
        tile_size,
        bounding_rect,
        nodes,
    }
}

fn write_metadata(metadata: Metadata, output_directory: &Path) {
    let mut meta = proto::Meta::new();
    meta.set_bounding_rect(metadata.bounding_rect);
    meta.set_deepest_level(u32::from(metadata.deepest_level));
    meta.set_tile_size(metadata.tile_size);
    meta.set_version(xray::CURRENT_VERSION);

    for node_id in metadata.nodes {
        let mut proto = proto::NodeId::new();
        proto.set_index(node_id.index());
        proto.set_level(u32::from(node_id.level()));
        meta.mut_nodes().push(proto);
    }

    let mut buf_writer = BufWriter::new(File::create(output_directory.join("meta.pb")).unwrap());
    meta.write_to_writer(&mut buf_writer)
        .expect("Failed to write meta.pb.");
}

fn merge(mut metadata: Metadata, output_directory: &Path, tile_background_color: Color<u8>) {
    let mut current_level_nodes = metadata.root_nodes.clone();
    for current_level in (metadata.level..metadata.deepest_level).rev() {
        current_level_nodes = current_level_nodes
            .iter()
            .filter_map(|node| node.parent_id())
            .collect();
        generation::build_level(
            output_directory,
            metadata.tile_size,
            current_level,
            &current_level_nodes,
            tile_background_color,
        );
        metadata.nodes.extend(&current_level_nodes);
    }
    write_metadata(metadata, output_directory);
}

fn main() {
    let args = CommandlineArguments::from_args();
    let metadata = read_metadata_from_directories(&args.input_directories);
    let metadata = validate_metadata(&metadata);
    copy_all_images(&args.input_directories, &args.output_directory);
    merge(
        metadata,
        &args.output_directory,
        args.tile_background_color.to_color(),
    );
}
