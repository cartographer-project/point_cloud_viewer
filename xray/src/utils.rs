use crate::META_FILENAME;
use image::{GenericImage, GenericImageView, ImageResult, Pixel, RgbaImage, SubImage};
use quadtree::{NodeId, NODE_PREFIX};
use std::io;
use std::path::{Path, PathBuf};

lazy_static::lazy_static! {
    static ref META_PREFIX: &'static str = Path::new(META_FILENAME)
        .file_stem()
        .unwrap()
        .to_str()
        .unwrap();
    static ref META_EXTENSION: &'static str = Path::new(META_FILENAME)
        .extension()
        .unwrap()
        .to_str()
        .unwrap();
}

pub fn get_meta_pb_path(directory: &Path, id: NodeId) -> PathBuf {
    directory
        .join(id.to_string().replace(NODE_PREFIX, *META_PREFIX))
        .with_extension(*META_EXTENSION)
}

pub fn get_root_node_id_from_meta_pb_path(meta_path: &Path) -> io::Result<NodeId> {
    let invalid_input_error = || {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("Invalid path {:?}.", meta_path),
        )
    };
    let stem = meta_path
        .file_stem()
        .and_then(|stem| stem.to_str())
        .map(|stem| stem.replace(*META_PREFIX, NODE_PREFIX))
        .ok_or_else(invalid_input_error)?;

    stem.parse::<NodeId>().map_err(|_| invalid_input_error())
}

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_id_to_paths_back_and_forth() {
        let directory = PathBuf::from("/tmp/");
        let root_node_id = NodeId::root();
        let path = get_meta_pb_path(&directory, root_node_id);
        let expected_path = Path::new("/tmp").join(META_FILENAME);
        assert_eq!(path, expected_path);
        let derived_root_node_id =
            get_root_node_id_from_meta_pb_path(&path).expect("Failed to get root node id.");
        assert_eq!(root_node_id, derived_root_node_id);

        let directory = PathBuf::from("/tmp/");
        let root_node_id = NodeId::new(1, 2);
        let path = get_meta_pb_path(&directory, root_node_id);
        let expected_path = Path::new("/tmp").join(format!("{}2.{}", *META_PREFIX, *META_SUFFIX));
        assert_eq!(path, expected_path);
        let derived_root_node_id =
            get_root_node_id_from_meta_pb_path(&path).expect("Failed to get root node id.");
        assert_eq!(root_node_id, derived_root_node_id);
    }
}
