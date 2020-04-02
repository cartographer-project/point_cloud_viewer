use fnv::FnvHashSet;
use point_viewer::color::Color;
use quadtree::{Node, NodeId};
use std::fs::create_dir_all;
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
    if input_directory.canonicalize().unwrap() == output_directory.canonicalize().unwrap() {
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
        .unwrap_or_else(|_| {
            panic!(
                "Failed to copy {:?} to {:?}.",
                dir_entry.path(),
                output_directory
            )
        });
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
        .map(|dir_entry| {
            Meta::from_disk(dir_entry.path())
                .unwrap_or_else(|_| panic!("Failed to load meta from {:?}.", dir_entry.path()))
        })
        .collect()
}

fn read_metadata_from_directories(directories: &[PathBuf]) -> Vec<Meta> {
    directories
        .iter()
        .map(|directory| read_metadata_from_directory(&directory))
        .flatten()
        .collect()
}

fn get_root_nodes(meta: &[Meta]) -> Vec<Node> {
    let root_nodes: Vec<Node> = meta.iter().filter_map(Meta::get_root_node).collect();
    if root_nodes.len() != meta.len() {
        eprintln!(
            "Skipped {} empty subquadtrees.",
            meta.len() - root_nodes.len()
        );
    }
    root_nodes
}

struct MergedMetadata {
    root_node_ids: FnvHashSet<NodeId>,
    level: u8,
    root_meta: Meta,
}

// Checks wheter all the elements in the iterator have the same value and returns it.
// There must be at least one element in the iterator.
fn all_equal<I, V>(iterator: I) -> Option<V>
where
    I: Iterator<Item = V>,
    V: std::cmp::PartialEq,
{
    all_equal_by_func(iterator, |left, right| left == right)
}

fn all_equal_by_func<I, V, F>(mut iterator: I, comp: F) -> Option<V>
where
    I: Iterator<Item = V>,
    F: Fn(&V, &V) -> bool,
{
    iterator.next().and_then(|first_element| {
        if iterator.all(|element| comp(&element, &first_element)) {
            Some(first_element)
        } else {
            None
        }
    })
}

fn validate_and_merge_metadata(metadata: &[Meta]) -> MergedMetadata {
    assert!(!metadata.is_empty(), "No subquadtrees meta files found.");
    let root_nodes_vec = get_root_nodes(metadata);
    let root_node_ids: FnvHashSet<NodeId> = root_nodes_vec.iter().map(|node| node.id).collect();
    assert_eq!(
        root_node_ids.len(),
        root_nodes_vec.len(),
        "Not all roots are unique."
    );
    let level = all_equal(root_node_ids.iter().map(|node_id| node_id.level()))
        .expect("Not all roots have the same level.");
    let deepest_level = all_equal(metadata.iter().map(|meta| meta.deepest_level))
        .expect("Not all meta files have the same deepest level.") as u8;
    let tile_size = all_equal(metadata.iter().map(|meta| meta.tile_size))
        .expect("Not all meta files have the same tile size.");
    let bounding_rect = {
        let mut root_node = root_nodes_vec.first().cloned();
        while let Some(node) = root_node {
            root_node = node.parent();
        }
        root_node.expect("No parent root node found.").bounding_rect
    };

    let mut nodes = FnvHashSet::default();
    for meta in metadata {
        nodes.extend(&meta.nodes);
    }

    MergedMetadata {
        root_node_ids,
        level,
        root_meta: Meta {
            deepest_level,
            tile_size,
            bounding_rect,
            nodes,
        },
    }
}

fn merge(mut metadata: MergedMetadata, output_directory: &Path, tile_background_color: Color<u8>) {
    let all_node_ids = generation::create_non_leaf_nodes(
        metadata.root_node_ids,
        metadata.level,
        0, // root_level
        output_directory,
        tile_background_color,
        metadata.root_meta.tile_size,
    );
    metadata.root_meta.nodes.extend(&all_node_ids);
    metadata
        .root_meta
        .to_disk(output_directory.join("meta.pb"))
        .expect("Failed to write meta.pb");
}

fn main() {
    let args = CommandlineArguments::from_args();
    for input_directory in &args.input_directories {
        assert!(
            input_directory.exists(),
            format!("Input directory {:?} doesn't exist.", input_directory)
        );
        assert!(
            input_directory.metadata().unwrap().is_dir(),
            format!("{:?} is not a directory.", input_directory)
        )
    }
    if !args.output_directory.exists() {
        create_dir_all(&args.output_directory).expect("Couldn't create the output directory.");
    }
    let metadata = read_metadata_from_directories(&args.input_directories);
    let merged_metadata = validate_and_merge_metadata(&metadata);
    copy_all_images(&args.input_directories, &args.output_directory);
    merge(
        merged_metadata,
        &args.output_directory,
        args.tile_background_color.to_color(),
    );
}
