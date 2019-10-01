use crate::graphic::tiled_texture_loader::{TilePos, TiledTextureLoader};
use cgmath::Vector3;
use image::{LumaA, Rgba};
use point_viewer::math::Isometry3;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::{self, ErrorKind};

#[derive(Serialize, Deserialize)]
pub struct Metadata {
    pub tile_size: u32,
    pub world_from_terrain: Isometry3<f64>,
    pub origin: Vector3<f64>,
    pub resolution_m: f64,
    pub tile_positions: Vec<TilePos>,
}

pub type TextureLoaders = (TiledTextureLoader<LumaA<f32>>, TiledTextureLoader<Rgba<u8>>);

impl Metadata {
    pub fn from_dir<P: AsRef<std::path::Path>>(dir: P) -> io::Result<Self> {
        let meta_path = dir.as_ref().join("meta.json");
        let reader = File::open(meta_path)?;
        serde_json::from_reader(reader).map_err(|e| {
            let msg = format!("Could not parse meta.json: {}", e.to_string());
            io::Error::new(ErrorKind::InvalidData, msg)
        })
    }

    pub fn read_tiles<P: AsRef<std::path::Path>>(&self, dir: P) -> io::Result<TextureLoaders> {
        let height_tiles = self.tile_positions.iter().map(|&(x, y)| {
            let height_file_path = dir.as_ref().join(format!("x{:08}_y{:08}.height", x, y));
            ((x, y), height_file_path)
        });
        let color_tiles = self.tile_positions.iter().map(|&(x, y)| {
            let color_file_path = dir.as_ref().join(format!("x{:08}_y{:08}.color", x, y));
            ((x, y), color_file_path)
        });
        let height_tiles = TiledTextureLoader::new(self.tile_size, height_tiles)?;
        let color_tiles = TiledTextureLoader::new(self.tile_size, color_tiles)?;
        Ok((height_tiles, color_tiles))
    }

    pub fn write<P: AsRef<std::path::Path>>(&self, dir: P) -> io::Result<()> {
        let writer = File::create(dir.as_ref().join("meta.json"))?;
        serde_json::to_writer(writer, &self).map_err(|e| {
            let msg = format!("Could not write meta.json: {}", e.to_string());
            io::Error::new(ErrorKind::InvalidData, msg)
        })
    }
}
