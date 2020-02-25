// Inpainting is applied to fill holes in the X-ray quadtree leaf tiles.
// As inpainting depends on sampling the neighborhood of pixels, if we
// directly took the original tiles as input, we would get visible color
// differences for the inpainted pixels along the tile borders. Because
// of this we enlarge all tiles, so they overlap each other by half of their
// dimension, apply inpainting on the enlarged tiles and bilinearly interpolate
// between tiles, before cutting out the original tile.

use crate::utils::{get_image_path, image_from_path};
use fnv::FnvHashSet;
use image::{
    DynamicImage, GenericImage, GenericImageView, ImageResult, Luma, Pixel, Rgba, RgbaImage,
    SubImage,
};
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
use texture_synthesis::{Dims, Example, Session};

fn inpaint(image: RgbaImage, distance_px: u8) -> RgbaImage {
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
        .build()
        .expect("Inpaint failed.");
    texsynth.run(None).into_image().into_rgba()
}

/// Interpolates pixels at position (i, j) of this and the other subimage,
/// where this_weight weighs the pixel of this image, while (1 - this_weight)
/// weighs the pixel of the other image.
fn interpolate_pixels(
    i: u32,
    j: u32,
    this: &mut SubImage<&mut RgbaImage>,
    other: &mut SubImage<&mut RgbaImage>,
    this_weight: f32,
) {
    let this_pix = this.get_pixel_mut(i, j);
    let other_pix = other.get_pixel_mut(i, j);
    // We don't use imageproc::pixelops::interpolate as it doesn't round the result
    this_pix.apply2(&other_pix, |this_c, other_c| {
        (this_c as f32 * this_weight + other_c as f32 * (1.0 - this_weight)).round() as u8
    });
    *other_pix = *this_pix;
}

fn interpolate_subimages<W>(
    this: &mut SubImage<&mut RgbaImage>,
    other: &mut SubImage<&mut RgbaImage>,
    this_weighting_function: W,
) where
    W: Fn(u32, u32) -> f32,
{
    let (width, height) = this.dimensions();
    for j in 0..height {
        for i in 0..width {
            interpolate_pixels(i, j, this, other, this_weighting_function(i, j));
        }
    }
}

struct SpatialNodeInpainter<'a> {
    spatial_node_id: SpatialNodeId,
    output_directory: &'a Path,
}

impl<'a> SpatialNodeInpainter<'a> {
    fn get_inpaint_image_path(&self) -> PathBuf {
        get_image_path(self.output_directory, NodeId::from(self.spatial_node_id))
            .with_extension("inpaint.png")
    }

    fn spatial_id_from(&self, neighbor: impl Into<Option<Direction>>) -> Option<SpatialNodeId> {
        match neighbor.into() {
            Some(n) => self.spatial_node_id.neighbor(n),
            None => Some(self.spatial_node_id),
        }
    }

    fn image_from(&self, neighbor: impl Into<Option<Direction>>) -> ImageResult<Option<RgbaImage>> {
        self.spatial_id_from(neighbor)
            .and_then(|id| {
                image_from_path(&get_image_path(self.output_directory, NodeId::from(id)))
            })
            .transpose()
    }

    fn inpaint_image_and_path_from(
        &self,
        neighbor: impl Into<Option<Direction>>,
    ) -> ImageResult<Option<(RgbaImage, PathBuf)>> {
        self.spatial_id_from(neighbor)
            .and_then(|spatial_node_id| {
                let inpainter = SpatialNodeInpainter {
                    spatial_node_id,
                    output_directory: self.output_directory,
                };
                let inpaint_image_path = inpainter.get_inpaint_image_path();
                image_from_path(&inpaint_image_path).map(|inpaint_image_res| {
                    inpaint_image_res.map(|inpaint_image| (inpaint_image, inpaint_image_path))
                })
            })
            .transpose()
    }

    fn stitched_image(&self) -> ImageResult<Option<RgbaImage>> {
        if let Some(current) = self.image_from(None)? {
            let w = current.width() / 2;
            let h = current.height() / 2;
            let mut image = RgbaImage::from_pixel(4 * w, 4 * h, Rgba::from(TRANSPARENT.to_u8()));
            image.copy_from(&current, w, h);
            let mut copy_subimage = |dir: Direction,
                                     x: u32,
                                     y: u32,
                                     width: u32,
                                     height: u32,
                                     to_x: u32,
                                     to_y: u32|
             -> ImageResult<()> {
                if let Some(neighbor) = self.image_from(dir)? {
                    image.copy_from(&neighbor.view(x, y, width, height), to_x, to_y);
                }
                Ok(())
            };
            copy_subimage(Direction::TopLeft, w, h, w, h, 0, 0)?;
            copy_subimage(Direction::Top, 0, h, 2 * w, h, w, 0)?;
            copy_subimage(Direction::TopRight, 0, h, w, h, 3 * w, 0)?;
            copy_subimage(Direction::Right, 0, 0, w, 2 * h, 3 * w, h)?;
            copy_subimage(Direction::BottomRight, 0, 0, w, h, 3 * w, 3 * h)?;
            copy_subimage(Direction::Bottom, 0, 0, 2 * w, h, w, 3 * h)?;
            copy_subimage(Direction::BottomLeft, w, 0, w, h, 0, 3 * h)?;
            copy_subimage(Direction::Left, w, 0, w, 2 * h, 0, h)?;
            Ok(Some(image))
        } else {
            Ok(None)
        }
    }

    fn create_inpaint_image(&self, inpaint_distance_px: u8) -> ImageResult<()> {
        if let Some(mut inpaint_image) = self.stitched_image()? {
            inpaint_image = inpaint(inpaint_image, inpaint_distance_px);
            let inpaint_image_path = self.get_inpaint_image_path();
            inpaint_image.save(inpaint_image_path)?;
        }
        Ok(())
    }

    fn interpolate_inpaint_image_with_right(&self) -> ImageResult<()> {
        if let (Some((mut current, current_path)), Some((mut right, right_path))) = (
            self.inpaint_image_and_path_from(None)?,
            self.inpaint_image_and_path_from(Direction::Right)?,
        ) {
            let (width, height) = (current.width() / 2, current.height());
            interpolate_subimages(
                &mut right.sub_image(0, 0, width, height),
                &mut current.sub_image(width, 0, width, height),
                |i, _| i as f32 / (width - 1) as f32,
            );
            current.save(current_path)?;
            right.save(right_path)?;
        }
        Ok(())
    }

    fn interpolate_inpaint_image_with_bottom(&self) -> ImageResult<()> {
        if let (Some((mut current, current_path)), Some((mut bottom, bottom_path))) = (
            self.inpaint_image_and_path_from(None)?,
            self.inpaint_image_and_path_from(Direction::Bottom)?,
        ) {
            let (width, height) = (current.width(), current.height() / 2);
            interpolate_subimages(
                &mut bottom.sub_image(0, 0, width, height),
                &mut current.sub_image(0, height, width, height),
                |_, j| j as f32 / (height - 1) as f32,
            );
            current.save(current_path)?;
            bottom.save(bottom_path)?;
        }
        Ok(())
    }

    fn apply_inpainting(&self) -> ImageResult<()> {
        if let Some((inpaint_image, inpaint_image_path)) = self.inpaint_image_and_path_from(None)? {
            let (width, height) = inpaint_image.dimensions();
            let image_view = inpaint_image.view(width / 4, height / 4, width / 2, height / 2);
            let image_path =
                get_image_path(self.output_directory, NodeId::from(self.spatial_node_id));
            image_view.to_image().save(image_path)?;
            fs::remove_file(inpaint_image_path)?;
        }
        Ok(())
    }
}

struct Inpainting<'a> {
    pool: &'a Pool,
    output_directory: &'a Path,
    spatial_node_ids: Vec<SpatialNodeId>,
}

impl<'a> Inpainting<'a> {
    fn step<P, F>(&self, message: &str, partitioning_function: P, inpainter_function: F)
    where
        P: FnMut(&&SpatialNodeId) -> bool,
        F: Fn(SpatialNodeInpainter) -> ImageResult<()> + Send + Copy,
    {
        let progress_bar = create_syncable_progress_bar(self.spatial_node_ids.len(), message);
        let run_partition = |spatial_node_ids: Vec<SpatialNodeId>| {
            self.pool.scoped(|scope| {
                for spatial_node_id in spatial_node_ids {
                    let inpainter = SpatialNodeInpainter {
                        spatial_node_id,
                        output_directory: self.output_directory,
                    };
                    let progress_bar = Arc::clone(&progress_bar);
                    scope.execute(move || {
                        // TODO(feuerste): Move to rayon and try_for_each to handle errors properly!
                        inpainter_function(inpainter).unwrap();
                        progress_bar.lock().unwrap().inc();
                    });
                }
            });
        };
        let (first, second): (Vec<SpatialNodeId>, Vec<SpatialNodeId>) = self
            .spatial_node_ids
            .iter()
            .partition(partitioning_function);
        run_partition(first);
        run_partition(second);
        progress_bar.lock().unwrap().finish_println("");
    }
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

    let spatial_node_ids: Vec<SpatialNodeId> = leaf_node_ids
        .iter()
        .cloned()
        .map(SpatialNodeId::from)
        .collect();

    let inpainting = Inpainting {
        pool,
        output_directory,
        spatial_node_ids,
    };

    inpainting.step(
        "Creating inpaint images",
        |_| false,
        |inpainter| inpainter.create_inpaint_image(inpaint_distance_px),
    );

    inpainting.step(
        "Horizontally interpolating inpaint images",
        // Interleave interpolation to avoid race conditions when writing images
        |spatial_node_id| spatial_node_id.x() % 2 == 0,
        |inpainter| inpainter.interpolate_inpaint_image_with_right(),
    );

    inpainting.step(
        "Vertically interpolating inpaint images",
        // Interleave interpolation to avoid race conditions when writing images
        |spatial_node_id| spatial_node_id.y() % 2 == 0,
        |inpainter| inpainter.interpolate_inpaint_image_with_bottom(),
    );

    inpainting.step(
        "Applying inpainting",
        |_| false,
        |inpainter| inpainter.apply_inpainting(),
    );
}
