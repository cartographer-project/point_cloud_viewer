use fnv::FnvHashSet;
use quadtree::{Direction, NodeId, SpatialNodeId};
use std::error::Error;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use structopt::StructOpt;
use xray::{
    generation::{assign_background_color, create_non_leaf_nodes, TileBackgroundColorArgument},
    inpaint::perform_inpainting,
    utils::{get_image_path, get_meta_pb_path},
    Meta,
};

#[derive(StructOpt, Debug)]
#[structopt(name = "inpaint_xray_quadtrees")]
/// Inpaint a (possibly partial) xray quadtree. We assume that the input
/// quadtree was generated with transparent background color, so we can
/// determine which pixels to actually fill.
struct CommandlineArguments {
    /// Directory with the (possibly partial) quadtree to be inpainted.
    /// Needs to include all leaf nodes of the neighboring quadtrees as well
    /// for smooth inpainting results.
    #[structopt(parse(from_os_str))]
    input_directory: PathBuf,
    /// Directory where to write the inpainted quadtree. Does *not*
    /// have to be disjoint from input_directory.
    #[structopt(parse(from_os_str), long)]
    output_directory: PathBuf,
    /// Tile background color.
    #[structopt(default_value = "white", long)]
    tile_background_color: TileBackgroundColorArgument,
    /// The inpainting distance in pixels to fill holes (in particular useful for high resolutions)
    #[structopt(long)]
    inpaint_distance_px: u8,
    /// The root node id to start inpainting with.
    #[structopt(long)]
    root_node_id: NodeId,
}

fn get_adjacent_leaf_node_ids(
    leaf_node_ids: &FnvHashSet<NodeId>,
    input_directory: &Path,
    root_node_id: NodeId,
) -> FnvHashSet<NodeId> {
    let spatial_root_node_id = SpatialNodeId::from(root_node_id);
    let mut neighboring_leaf_node_ids = FnvHashSet::default();
    let directions = [
        Direction::Left,
        Direction::Top,
        Direction::Right,
        Direction::Bottom,
    ];
    for i in 0..4 {
        if let Some(spatial_root_neighbor_id) = spatial_root_node_id.neighbor(directions[i]) {
            if let Ok(neighbor_meta) = Meta::from_disk(get_meta_pb_path(
                input_directory,
                NodeId::from(spatial_root_neighbor_id),
            )) {
                for neighbor_node_id in neighbor_meta
                    .nodes
                    .iter()
                    .filter(|id| id.level() == neighbor_meta.deepest_level)
                {
                    if let Some(spatial_id) =
                        SpatialNodeId::from(*neighbor_node_id).neighbor(directions[(i + 2) % 4])
                    {
                        if leaf_node_ids.contains(&NodeId::from(spatial_id)) {
                            neighboring_leaf_node_ids.insert(*neighbor_node_id);
                        }
                    }
                }
            }
        }
    }
    neighboring_leaf_node_ids
}

fn copy_nodes(
    node_ids: &FnvHashSet<NodeId>,
    input_directory: &Path,
    output_directory: &Path,
) -> io::Result<()> {
    for node_id in node_ids.iter() {
        fs::copy(
            get_image_path(input_directory, *node_id),
            get_image_path(output_directory, *node_id),
        )?;
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = CommandlineArguments::from_args();
    let input_directory = args.input_directory.canonicalize()?;
    if !args.output_directory.exists() {
        fs::create_dir_all(&args.output_directory)?;
    }
    let output_directory = args.output_directory.canonicalize()?;
    let tile_background_color = args.tile_background_color.to_color();
    let root_node_id = args.root_node_id;
    let meta = Meta::from_disk(get_meta_pb_path(&input_directory, root_node_id))?;

    let leaf_node_ids: FnvHashSet<NodeId> = meta
        .nodes
        .iter()
        .copied()
        .filter(|id| id.level() == meta.deepest_level)
        .collect();

    let adjacent_leaf_node_ids =
        get_adjacent_leaf_node_ids(&leaf_node_ids, &input_directory, root_node_id);
    if root_node_id.level() != 0 && adjacent_leaf_node_ids.is_empty() {
        println!(
            "No adjacent leaf nodes found in neighboring quadtrees. \
             Did you forget to copy them into {:?}?",
            &input_directory
        )
    }

    if input_directory != output_directory {
        fs::copy(
            get_meta_pb_path(&input_directory, root_node_id),
            get_meta_pb_path(&output_directory, root_node_id),
        )?;
        copy_nodes(&leaf_node_ids, &input_directory, &output_directory)?;
        copy_nodes(&adjacent_leaf_node_ids, &input_directory, &output_directory)?;
    }

    perform_inpainting(&output_directory, args.inpaint_distance_px, &leaf_node_ids)?;

    assign_background_color(&output_directory, tile_background_color, &leaf_node_ids)?;

    create_non_leaf_nodes(
        leaf_node_ids,
        meta.deepest_level,
        root_node_id.level(),
        &output_directory,
        tile_background_color,
        meta.tile_size,
    );

    if input_directory != output_directory {
        for adjacent_leaf_node_id in adjacent_leaf_node_ids {
            fs::remove_file(get_image_path(&output_directory, adjacent_leaf_node_id))?;
        }
    }

    Ok(())
}
