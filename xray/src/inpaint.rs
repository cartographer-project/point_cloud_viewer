use crate::utils::get_image_path;
use cgmath::Vector2;
use fnv::FnvHashSet;
use image::{DynamicImage, GenericImage, GenericImageView, ImageBuffer, Luma, Rgba, RgbaImage};
use imageproc::distance_transform::Norm;
use imageproc::map::{map_colors, map_colors2};
use imageproc::morphology::close;
use imageproc::pixelops::interpolate;
use point_viewer::color::{Color, TRANSPARENT};
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

fn to_u8(rgba: Rgba<f32>) -> Rgba<u8> {
    Rgba([
        rgba[0].round() as u8,
        rgba[1].round() as u8,
        rgba[2].round() as u8,
        rgba[3].round() as u8,
    ])
}

fn to_f32(rgba: Rgba<u8>) -> Rgba<f32> {
    Rgba([
        rgba[0] as f32,
        rgba[1] as f32,
        rgba[2] as f32,
        rgba[3] as f32,
    ])
}

fn get_inpaint_image_path(spatial_node_id: SpatialNodeId, output_directory: &Path) -> PathBuf {
    get_image_path(output_directory, NodeId::from(spatial_node_id)).with_extension("inpaint.png")
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

fn inpaint_image_from(
    spatial_node_id: SpatialNodeId,
    output_directory: &Path,
    neighbor: impl Into<Option<Direction>>,
) -> Option<RgbaImage> {
    spatial_id_from(spatial_node_id, neighbor)
        .and_then(|id| image_from_path(&get_inpaint_image_path(id, output_directory)))
}

fn stitched_image(spatial_node_id: SpatialNodeId, output_directory: &Path) -> Option<RgbaImage> {
    image_from(spatial_node_id, output_directory, None).map(|current| {
        let w = current.width() / 2;
        let h = current.height() / 2;

        let mut image = RgbaImage::from_pixel(4 * w, 4 * h, Rgba::from(TRANSPARENT.to_u8()));
        image.copy_from(&current, w, h);

        use Direction::*;
        if let Some(top_left) = image_from(spatial_node_id, output_directory, TopLeft) {
            image.copy_from(&top_left.view(w, h, w, h), 0, 0);
        }
        if let Some(top) = image_from(spatial_node_id, output_directory, Top) {
            image.copy_from(&top.view(0, h, 2 * w, h), w, 0);
        }
        if let Some(top_right) = image_from(spatial_node_id, output_directory, TopRight) {
            image.copy_from(&top_right.view(0, h, w, h), 3 * w, 0);
        }
        if let Some(right) = image_from(spatial_node_id, output_directory, Right) {
            image.copy_from(&right.view(0, 0, w, 2 * h), 3 * w, h);
        }
        if let Some(bottom_right) = image_from(spatial_node_id, output_directory, BottomRight) {
            image.copy_from(&bottom_right.view(0, 0, w, h), 3 * w, 3 * h);
        }
        if let Some(bottom) = image_from(spatial_node_id, output_directory, Bottom) {
            image.copy_from(&bottom.view(0, 0, 2 * w, h), w, 3 * h);
        }
        if let Some(bottom_left) = image_from(spatial_node_id, output_directory, BottomLeft) {
            image.copy_from(&bottom_left.view(w, 0, w, h), 0, 3 * h);
        }
        if let Some(left) = image_from(spatial_node_id, output_directory, Left) {
            image.copy_from(&left.view(w, 0, w, 2 * h), 0, h);
        }
        image
    })
}

fn interpolate_inpaint_image(
    spatial_node_id: SpatialNodeId,
    output_directory: &Path,
) -> Option<RgbaImage> {
    inpaint_image_from(spatial_node_id, output_directory, None).map(|current| {
        let w = current.width() / 4;
        let h = current.height() / 4;

        let mut image = ImageBuffer::<Rgba<f32>, Vec<f32>>::from_pixel(
            2 * w,
            2 * h,
            Rgba::from(Color::<f32>::default()),
        );
        let current_inpaint_view = current.view(w, h, image.width(), image.height());
        for (i, j, pix) in image.enumerate_pixels_mut() {
            *pix = to_f32(current_inpaint_view.get_pixel(i, j));
        }

        use Direction::*;
        if let Some(top) = inpaint_image_from(spatial_node_id, output_directory, Top) {
            let mut sub_image = image.sub_image(0, 0, 2 * w, h);
            let width = sub_image.width();
            let height = sub_image.height();
            let inpaint_view = top.view(w, 3 * h, width, height);
            for j in 0..height {
                let factor = 0.5 * (height - 1 - j) as f32 / (height - 1) as f32;
                for i in 0..width {
                    let pix = sub_image.get_pixel_mut(i, j);
                    *pix = interpolate(to_f32(inpaint_view.get_pixel(i, j)), *pix, factor);
                }
            }
        }
        if let Some(bottom) = inpaint_image_from(spatial_node_id, output_directory, Bottom) {
            let mut sub_image = image.sub_image(0, h, 2 * w, h);
            let width = sub_image.width();
            let height = sub_image.height();
            let inpaint_view = bottom.view(w, 0, width, height);
            for j in 0..height {
                let factor = 0.5 * j as f32 / (height - 1) as f32;
                for i in 0..width {
                    let pix = sub_image.get_pixel_mut(i, j);
                    *pix = interpolate(to_f32(inpaint_view.get_pixel(i, j)), *pix, factor);
                }
            }
        }
        if let Some(left) = inpaint_image_from(spatial_node_id, output_directory, Left) {
            let mut sub_image = image.sub_image(0, 0, w, 2 * h);
            let width = sub_image.width();
            let height = sub_image.height();
            let inpaint_view = left.view(3 * w, h, width, height);
            for i in 0..width {
                let factor = 0.5 * (width - 1 - i) as f32 / (width - 1) as f32;
                for j in 0..height {
                    let pix = sub_image.get_pixel_mut(i, j);
                    *pix = interpolate(to_f32(inpaint_view.get_pixel(i, j)), *pix, factor);
                }
            }
        }
        if let Some(right) = inpaint_image_from(spatial_node_id, output_directory, Right) {
            let mut sub_image = image.sub_image(w, 0, w, 2 * h);
            let width = sub_image.width();
            let height = sub_image.height();
            let inpaint_view = right.view(0, h, width, height);
            for i in 0..width {
                let factor = 0.5 * i as f32 / (width - 1) as f32;
                for j in 0..height {
                    let pix = sub_image.get_pixel_mut(i, j);
                    *pix = interpolate(to_f32(inpaint_view.get_pixel(i, j)), *pix, factor);
                }
            }
        }
        map_colors(&image, to_u8)
    })
}

fn inpainting_step<F>(
    message: &str,
    pool: &Pool,
    spatial_node_ids: &[SpatialNodeId],
    spatial_node_function: F,
) where
    F: Fn(SpatialNodeId) + Send + Copy,
{
    let progress_bar = create_syncable_progress_bar(spatial_node_ids.len(), message);
    pool.scoped(|scope| {
        for spatial_node_id in spatial_node_ids {
            let progress_bar = Arc::clone(&progress_bar);
            scope.execute(move || {
                spatial_node_function(*spatial_node_id);
                progress_bar.lock().unwrap().inc();
            });
        }
    });
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
        |spatial_node_id| {
            if let Some(mut image) = stitched_image(spatial_node_id, output_directory) {
                image = inpaint(image, inpaint_distance_px).expect("Inpaint failed.");
                let image_path = get_inpaint_image_path(spatial_node_id, output_directory);
                image.save(image_path).unwrap();
            }
        },
    );

    inpainting_step(
        "Applying inpainting",
        pool,
        &spatial_leaf_node_ids,
        |spatial_node_id| {
            if let Some(image) = interpolate_inpaint_image(spatial_node_id, output_directory) {
                let node_id = NodeId::from(spatial_node_id);
                let image_path = get_image_path(output_directory, node_id);
                image.save(image_path).unwrap();
            }
        },
    );

    inpainting_step(
        "Removing inpaint images",
        pool,
        &spatial_leaf_node_ids,
        |spatial_node_id| {
            let image_path = get_inpaint_image_path(spatial_node_id, output_directory);
            if image_path.exists() {
                fs::remove_file(image_path).unwrap();
            }
        },
    );
}
