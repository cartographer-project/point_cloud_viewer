use crate::utils::get_image_path;
use fnv::FnvHashSet;
use image::{DynamicImage, GenericImage, GenericImageView, Luma, Rgba, RgbaImage, SubImage};
use imageproc::distance_transform::Norm;
use imageproc::map::{map_colors, map_colors2};
use imageproc::morphology::close;
use point_viewer::color::TRANSPARENT;
use point_viewer::utils::create_syncable_progress_bar;
use quadtree::{Direction, NodeId, SpatialNodeId};
use scoped_pool::Pool;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use texture_synthesis::{Dims, Error, Example, Session};

fn inpaint(image: RgbaImage, distance_px: u8) -> Result<RgbaImage, Error> {
    let (width, height) = image.dimensions();
    let mask = map_colors(&image, |p| Luma([p[3]]));
    let closed_mask = close(&mask, Norm::LInf, distance_px);
    let inpaint_mask = map_colors2(&closed_mask, &mask, |c, m| Luma([255 - (c[0] - m[0])]));
    let texsynth = Session::builder()
        // we perform multithreading outside, so one deterministic thread here is enough
        .max_thread_count(1)
        .inpaint_example(
            DynamicImage::ImageLuma8(inpaint_mask),
            Example::builder(DynamicImage::ImageRgba8(image))
                .set_sample_method(DynamicImage::ImageLuma8(mask)),
            Dims::new(width, height),
        )
        .build()?;
    let generated = texsynth.run(None);
    Ok(generated.into_image().into_rgba())
}

fn get_image_path_with_extension(
    spatial_node_id: SpatialNodeId,
    output_directory: &Path,
    extension: &str,
) -> PathBuf {
    get_image_path(output_directory, NodeId::from(spatial_node_id)).with_extension(extension)
}

fn get_inpaint_image_path(spatial_node_id: SpatialNodeId, output_directory: &Path) -> PathBuf {
    get_image_path_with_extension(spatial_node_id, output_directory, "inpaint.png")
}

fn image_from_path(image_path: &Path) -> Option<RgbaImage> {
    if image_path.exists() {
        Some(image::open(image_path).unwrap().to_rgba())
    } else {
        None
    }
}

fn spatial_id_from(
    spatial_node_id: SpatialNodeId,
    neighbor: impl Into<Option<Direction>>,
) -> Option<SpatialNodeId> {
    match neighbor.into() {
        Some(n) => spatial_node_id.neighbor(n),
        None => Some(spatial_node_id),
    }
}

fn image_from(
    spatial_node_id: SpatialNodeId,
    output_directory: &Path,
    neighbor: impl Into<Option<Direction>>,
) -> Option<RgbaImage> {
    spatial_id_from(spatial_node_id, neighbor)
        .and_then(|id| image_from_path(&get_image_path(output_directory, NodeId::from(id))))
}

fn inpaint_image_and_path_from(
    spatial_node_id: SpatialNodeId,
    output_directory: &Path,
    neighbor: impl Into<Option<Direction>>,
) -> Option<(RgbaImage, PathBuf)> {
    spatial_id_from(spatial_node_id, neighbor).and_then(|id| {
        let inpaint_image_path = get_inpaint_image_path(id, output_directory);
        image_from_path(&inpaint_image_path)
            .map(|inpaint_image| (inpaint_image, inpaint_image_path))
    })
}

fn stitched_image(spatial_node_id: SpatialNodeId, output_directory: &Path) -> Option<RgbaImage> {
    image_from(spatial_node_id, output_directory, None).map(|current| {
        let w = current.width() / 2;
        let h = current.height() / 2;

        let mut image = RgbaImage::from_pixel(4 * w, 4 * h, Rgba::from(TRANSPARENT.to_u8()));
        image.copy_from(&current, w, h);

        let mut copy_subimage =
            |dir: Direction, x: u32, y: u32, width: u32, height: u32, to_x: u32, to_y: u32| {
                if let Some(neighbor) = image_from(spatial_node_id, output_directory, dir) {
                    image.copy_from(&neighbor.view(x, y, width, height), to_x, to_y);
                }
            };

        copy_subimage(Direction::TopLeft, w, h, w, h, 0, 0);
        copy_subimage(Direction::Top, 0, h, 2 * w, h, w, 0);
        copy_subimage(Direction::TopRight, 0, h, w, h, 3 * w, 0);
        copy_subimage(Direction::Right, 0, 0, w, 2 * h, 3 * w, h);
        copy_subimage(Direction::BottomRight, 0, 0, w, h, 3 * w, 3 * h);
        copy_subimage(Direction::Bottom, 0, 0, 2 * w, h, w, 3 * h);
        copy_subimage(Direction::BottomLeft, w, 0, w, h, 0, 3 * h);
        copy_subimage(Direction::Left, w, 0, w, 2 * h, 0, h);

        image
    })
}

fn create_inpaint_image(
    spatial_node_id: SpatialNodeId,
    output_directory: &Path,
    inpaint_distance_px: u8,
) {
    if let Some(mut inpaint_image) = stitched_image(spatial_node_id, output_directory) {
        inpaint_image = inpaint(inpaint_image, inpaint_distance_px).expect("Inpaint failed.");
        let inpaint_image_path = get_inpaint_image_path(spatial_node_id, output_directory);
        inpaint_image.save(inpaint_image_path).unwrap();
    }
}

fn interpolate(
    i: u32,
    j: u32,
    this_weight: f32,
    this: &mut SubImage<&mut RgbaImage>,
    other: &mut SubImage<&mut RgbaImage>,
) {
    let this_pix = this.get_pixel_mut(i, j);
    let other_pix = other.get_pixel_mut(i, j);
    let interpolated = imageproc::pixelops::interpolate(*this_pix, *other_pix, this_weight);
    *this_pix = interpolated;
    *other_pix = interpolated;
}

fn interpolate_inpaint_image_with_right(spatial_node_id: SpatialNodeId, output_directory: &Path) {
    if let (Some((mut current, current_path)), Some((mut right, right_path))) = (
        inpaint_image_and_path_from(spatial_node_id, output_directory, None),
        inpaint_image_and_path_from(spatial_node_id, output_directory, Direction::Right),
    ) {
        let width = current.width() / 2;
        let height = current.height();
        let mut current_sub = current.sub_image(width, 0, width, height);
        let mut right_sub = right.sub_image(0, 0, width, height);
        for i in 0..width {
            let right_weight = i as f32 / (width - 1) as f32;
            for j in 0..height {
                interpolate(i, j, right_weight, &mut right_sub, &mut current_sub);
            }
        }
        current.save(current_path).unwrap();
        right.save(right_path).unwrap();
    }
}

fn interpolate_inpaint_image_with_bottom(spatial_node_id: SpatialNodeId, output_directory: &Path) {
    if let (Some((mut current, current_path)), Some((mut bottom, bottom_path))) = (
        inpaint_image_and_path_from(spatial_node_id, output_directory, None),
        inpaint_image_and_path_from(spatial_node_id, output_directory, Direction::Bottom),
    ) {
        let width = current.width();
        let height = current.height() / 2;
        let mut current_sub = current.sub_image(0, height, width, height);
        let mut bottom_sub = bottom.sub_image(0, 0, width, height);
        for j in 0..height {
            let bottom_weight = j as f32 / (height - 1) as f32;
            for i in 0..width {
                interpolate(i, j, bottom_weight, &mut bottom_sub, &mut current_sub);
            }
        }
        current.save(current_path).unwrap();
        bottom.save(bottom_path).unwrap();
    }
}

fn apply_inpainting(spatial_node_id: SpatialNodeId, output_directory: &Path) {
    if let Some((inpaint_image, _)) =
        inpaint_image_and_path_from(spatial_node_id, output_directory, None)
    {
        let (width, height) = inpaint_image.dimensions();
        let image_view = inpaint_image.view(width / 4, height / 4, width / 2, height / 2);
        let image_path = get_image_path(output_directory, NodeId::from(spatial_node_id));
        image_view.to_image().save(image_path).unwrap();
        fs::remove_file(get_inpaint_image_path(spatial_node_id, output_directory)).unwrap();
    }
}

fn inpainting_step<P, F>(
    message: &str,
    pool: &Pool,
    spatial_node_ids: &[SpatialNodeId],
    partitioning_function: P,
    spatial_node_function: F,
) where
    P: FnMut(&&SpatialNodeId) -> bool,
    F: Fn(SpatialNodeId) + Send + Copy,
{
    let progress_bar = create_syncable_progress_bar(spatial_node_ids.len(), message);
    let run_partition = |spatial_node_ids: Vec<SpatialNodeId>| {
        pool.scoped(|scope| {
            for spatial_node_id in spatial_node_ids {
                let progress_bar = Arc::clone(&progress_bar);
                scope.execute(move || {
                    spatial_node_function(spatial_node_id);
                    progress_bar.lock().unwrap().inc();
                });
            }
        });
    };

    let (first, second): (Vec<SpatialNodeId>, Vec<SpatialNodeId>) =
        spatial_node_ids.iter().partition(partitioning_function);
    run_partition(first);
    run_partition(second);

    progress_bar.lock().unwrap().finish_println("");
}

pub fn perform_inpainting(
    pool: &Pool,
    output_directory: &Path,
    inpaint_distance_px: u8,
    leaf_node_ids: &FnvHashSet<NodeId>,
) {
    if inpaint_distance_px == 0 {
        return;
    }

    let spatial_leaf_node_ids: Vec<SpatialNodeId> = leaf_node_ids
        .iter()
        .cloned()
        .map(SpatialNodeId::from)
        .collect();

    inpainting_step(
        "Creating inpaint images",
        pool,
        &spatial_leaf_node_ids,
        |_| false,
        |spatial_node_id| {
            create_inpaint_image(spatial_node_id, output_directory, inpaint_distance_px)
        },
    );

    inpainting_step(
        "Horizontally interpolating inpaint images",
        pool,
        &spatial_leaf_node_ids,
        // Interleave interpolation to avoid race conditions when writing images
        |spatial_node_id| spatial_node_id.x() % 2 == 0,
        |spatial_node_id| interpolate_inpaint_image_with_right(spatial_node_id, output_directory),
    );

    inpainting_step(
        "Vertically interpolating inpaint images",
        pool,
        &spatial_leaf_node_ids,
        // Interleave interpolation to avoid race conditions when writing images
        |spatial_node_id| spatial_node_id.y() % 2 == 0,
        |spatial_node_id| interpolate_inpaint_image_with_bottom(spatial_node_id, output_directory),
    );

    inpainting_step(
        "Applying inpainting",
        pool,
        &spatial_leaf_node_ids,
        |_| false,
        |spatial_node_id| apply_inpainting(spatial_node_id, output_directory),
    );
}
