use fnv::FnvHashSet;
use point_viewer::color::Color;
use quadtree::NodeId;
use std::path::{Path, PathBuf};
use structopt::StructOpt;
use xray::{generation, Meta};

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

fn copy_all_images(input_directories: &[PathBuf], output_directory: &Path) {
    for input_directory in input_directories {
        copy_images(input_directory, output_directory);
    }
}

fn read_metadata_from_directory(directory: &Path) -> Vec<Meta> {
    globwalk::GlobWalkerBuilder::new(directory, "meta*.pb")
        .build()
        .expect("Failed to build GlobWalker")
        .filter_map(Result::ok)
        .map(|dir_entry| PathBuf::from(dir_entry.path()))
        .map(|path| Meta::from_disk(&path).expect(&format!("Failed to load meta from {:?}.", path)))
        .collect()
}

fn read_metadata_from_directories(directories: &[PathBuf]) -> Vec<Meta> {
    directories
        .iter()
        .map(|directory| read_metadata_from_directory(&directory))
        .flatten()
        .collect()
}

fn get_root_node(meta: &Meta) -> Option<NodeId> {
    meta.nodes.iter().copied().min_by_key(|node| node.level())
}

fn get_root_nodes(meta: &[Meta]) -> Vec<Option<NodeId>> {
    meta.iter().map(get_root_node).collect()
}

struct MergedMetadata {
    root_nodes: FnvHashSet<NodeId>,
    level: u8,
    merged_meta: Meta,
}

// Checks wheter all the elements in the iterator have the same value and returns it.
// There must be at least one element in the iterator.
fn check_all_the_same<I, V>(mut iterator: I) -> Option<V>
where
    I: Iterator<Item = V>,
    V: std::cmp::PartialEq,
{
    let first = iterator.next().expect("Iterator cannot be empty");
    if iterator.all(|element| element == first) {
        Some(first)
    } else {
        None
    }
}

fn validate_metadata(metadata: &[Meta]) -> MergedMetadata {
    assert!(!metadata.is_empty(), "No meta.pb files found.");
    let (somes, nones): (Vec<Option<NodeId>>, Vec<Option<NodeId>>) = get_root_nodes(metadata)
        .iter()
        .partition(|res| res.is_some());
    assert!(!somes.is_empty(), "All quadtrees are empty.");
    if !nones.is_empty() {
        println!(
            "{} out of {} quadtrees are empty.",
            nones.len(),
            metadata.len()
        );
    }
    // The unwrap below is is safe.
    let root_nodes: FnvHashSet<NodeId> = somes.iter().map(|node_id| node_id.unwrap()).collect();
    assert_eq!(root_nodes.len(), somes.len(), "Not all roots are unique.");
    let level = check_all_the_same(root_nodes.iter().map(|node| node.level()))
        .expect("Not all roots have the same level.");
    let deepest_level = check_all_the_same(metadata.iter().map(|meta| meta.deepest_level))
        .expect("Not all meta files have the same deepest level.") as u8;
    let tile_size = check_all_the_same(metadata.iter().map(|meta| meta.tile_size))
        .expect("Not all meta files have the same tile size.");
    // TODO: check whether all root nodes have the same bound_rect.
    let bounding_rect = metadata[0].bounding_rect.clone();

    let mut nodes = FnvHashSet::default();
    for meta in metadata {
        nodes.extend(&meta.nodes);
    }

    MergedMetadata {
        root_nodes,
        level,
        merged_meta: Meta {
            deepest_level,
            tile_size,
            bounding_rect,
            nodes,
        },
    }
}

fn merge(mut metadata: MergedMetadata, output_directory: &Path, tile_background_color: Color<u8>) {
    let mut current_level_nodes = metadata.root_nodes;
    for current_level in (0..metadata.level).rev() {
        current_level_nodes = current_level_nodes
            .iter()
            .filter_map(|node| node.parent_id())
            .collect();
        generation::build_level(
            output_directory,
            metadata.merged_meta.tile_size,
            current_level,
            &current_level_nodes,
            tile_background_color,
        );
        metadata.merged_meta.nodes.extend(&current_level_nodes);
    }
    metadata
        .merged_meta
        .to_disk(output_directory)
        .expect("Failed to write meta.pb");
}

fn main() {
    let args = CommandlineArguments::from_args();
    let metadata = read_metadata_from_directories(&args.input_directories);
    let merged_metadata = validate_metadata(&metadata);
    copy_all_images(&args.input_directories, &args.output_directory);
    merge(
        merged_metadata,
        &args.output_directory,
        args.tile_background_color.to_color(),
    );
}
