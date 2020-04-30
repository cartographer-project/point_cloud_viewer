use clap::Clap;
use fnv::FnvHashSet;
use point_viewer::color::Color;
use quadtree::{Node, NodeId};
use std::fs::create_dir_all;
use std::io;
use std::path::{Path, PathBuf};
use xray::{generation, Meta, META_EXTENSION, META_FILENAME, META_PREFIX};

#[derive(Clap, Debug)]
#[clap(name = "merge_xray_quadtrees")]
/// Merge partial xray quadtrees. We assume that the root
/// of each quadtree belongs to the same level of the final
/// quadtree.
struct CommandlineArguments {
    /// Directory where to write the merged quadtree. Does *not*
    /// have to be disjoint from input_directories.
    #[clap(parse(from_os_str), long)]
    output_directory: PathBuf,
    /// Tile background color.
    #[clap(arg_enum, default_value = "white", long)]
    tile_background_color: generation::TileBackgroundColorArgument,
    /// Directories with, possibly multiple, partial xray quadtrees.
    #[clap(parse(from_os_str))]
    input_directories: Vec<PathBuf>,
}

fn copy_images(input_directory: &Path, output_directory: &Path) -> io::Result<()> {
    if input_directory.canonicalize()? == output_directory.canonicalize()? {
        return Ok(());
    }
    globwalk::GlobWalkerBuilder::new(
        input_directory,
        &format!("*.{}", xray::IMAGE_FILE_EXTENSION),
    )
    .build()
    .expect("Failed to build GlobWalker")
    .filter_map(Result::ok)
    .try_for_each(|dir_entry| {
        std::fs::copy(
            dir_entry.path(),
            output_directory.join(dir_entry.file_name()),
        )
        .map(|_| ())
    })
}

fn copy_all_images(input_directories: &[PathBuf], output_directory: &Path) -> io::Result<()> {
    input_directories
        .iter()
        .try_for_each(|input_directory| copy_images(input_directory, output_directory))
}

fn read_metadata_from_directory(directory: &Path) -> io::Result<Vec<Meta>> {
    globwalk::GlobWalkerBuilder::new(directory, format!("{}*.{}", *META_PREFIX, *META_EXTENSION))
        .build()
        .expect("Failed to build GlobWalker")
        .filter_map(Result::ok)
        .map(|dir_entry| Meta::from_disk(dir_entry.path()))
        .collect()
}

fn read_metadata_from_directories(directories: &[PathBuf]) -> io::Result<Vec<Meta>> {
    directories
        .iter()
        .map(|directory| read_metadata_from_directory(&directory))
        .collect::<io::Result<Vec<Vec<Meta>>>>()
        .map(|directories| directories.into_iter().flatten().collect())
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
fn all_equal<I, V>(iterator: I, error_message: &str) -> io::Result<V>
where
    I: Iterator<Item = V>,
    V: std::cmp::PartialEq,
{
    all_equal_by_func(iterator, |left, right| left == right)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, error_message))
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

fn validate_input_directory(input_directory: &Path) -> io::Result<()> {
    if !input_directory.exists() {
        Err(io::Error::new(
            io::ErrorKind::NotFound,
            format!("Input directory {:?} doesn't exist.", input_directory),
        ))
    } else if !input_directory.metadata()?.is_dir() {
        Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("{:?} is not a directory.", input_directory),
        ))
    } else {
        Ok(())
    }
}

fn validate_and_merge_metadata(metadata: &[Meta]) -> io::Result<MergedMetadata> {
    if metadata.is_empty() {
        return Err(io::Error::new(
            io::ErrorKind::NotFound,
            "No subquadtrees meta files found.",
        ));
    }
    let root_nodes_vec = get_root_nodes(metadata);
    if root_nodes_vec.is_empty() {
        return Err(io::Error::new(
            io::ErrorKind::Other,
            "All subquadtress are empty.",
        ));
    }
    let root_node_ids: FnvHashSet<NodeId> = root_nodes_vec.iter().map(|node| node.id).collect();
    if root_node_ids.len() != root_nodes_vec.len() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "Not all roots are unique.",
        ));
    }
    let level = all_equal(
        root_node_ids.iter().map(|node_id| node_id.level()),
        "Not all roots have the same level.",
    )?;
    let deepest_level = all_equal(
        metadata.iter().map(|meta| meta.deepest_level),
        "Not all meta files have the same deepest level.",
    )? as u8;
    let tile_size = all_equal(
        metadata.iter().map(|meta| meta.tile_size),
        "Not all meta files have the same tile size.",
    )?;
    let bounding_rect = {
        // This unwrap is safe by one of the assertions above.
        let mut root_node = root_nodes_vec.first().cloned().unwrap();
        while let Some(node) = root_node.parent() {
            root_node = node;
        }
        root_node.bounding_rect
    };

    let mut nodes = FnvHashSet::default();
    for meta in metadata {
        nodes.extend(&meta.nodes);
    }

    Ok(MergedMetadata {
        root_node_ids,
        level,
        root_meta: Meta {
            deepest_level,
            tile_size,
            bounding_rect,
            nodes,
        },
    })
}

fn merge(
    mut metadata: MergedMetadata,
    output_directory: &Path,
    tile_background_color: Color<u8>,
) -> io::Result<()> {
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
        .to_disk(output_directory.join(META_FILENAME))
}

fn main() -> io::Result<()> {
    let args = CommandlineArguments::parse();
    args.input_directories
        .iter()
        .try_for_each(|directory| validate_input_directory(&directory))?;
    if !args.output_directory.exists() {
        create_dir_all(&args.output_directory)?;
    }
    let metadata = read_metadata_from_directories(&args.input_directories)?;
    let merged_metadata = validate_and_merge_metadata(&metadata)?;
    copy_all_images(&args.input_directories, &args.output_directory)?;
    merge(
        merged_metadata,
        &args.output_directory,
        args.tile_background_color.to_color(),
    )
}
