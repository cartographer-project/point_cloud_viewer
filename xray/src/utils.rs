use image::{GenericImage, GenericImageView, ImageResult, Pixel, RgbaImage, SubImage};
use quadtree::NodeId;
use std::path::{Path, PathBuf};

pub fn get_image_path(directory: &Path, id: NodeId) -> PathBuf {
    directory
        .join(id.to_string())
        .with_extension(crate::IMAGE_FILE_EXTENSION)
}

pub fn image_from_path(image_path: &Path) -> Option<ImageResult<RgbaImage>> {
    if image_path.exists() {
        Some(image::open(image_path).map(|image| image.to_rgba()))
    } else {
        None
    }
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

pub fn interpolate_subimages<W>(
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
