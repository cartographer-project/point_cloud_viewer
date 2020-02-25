use image::{ImageResult, RgbaImage};
use quadtree::NodeId;
use std::path::{Path, PathBuf};

pub fn get_image_path(directory: &Path, id: NodeId) -> PathBuf {
    directory.join(id.to_string()).with_extension("png")
}

pub fn image_from_path(image_path: &Path) -> Option<ImageResult<RgbaImage>> {
    if image_path.exists() {
        Some(image::open(image_path).map(|image| image.to_rgba()))
    } else {
        None
    }
}
