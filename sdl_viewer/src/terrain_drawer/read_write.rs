use crate::graphic::tiled_texture_loader::{TilePos, TiledTextureLoader};
use cgmath::Vector3;
use image::{LumaA, Rgba};
use point_viewer::math::Isometry3;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::{Error, ErrorKind};
use std::path::PathBuf;

#[derive(Clone)]
pub struct Metadata {
    pub tile_size: u32,
    pub world_from_terrain: Isometry3<f64>,
    pub origin: Vector3<f64>,
    pub resolution_m: f64,
    pub tile_positions: Vec<TilePos>,
    pub dir: PathBuf,
}

pub type TextureLoaders = (TiledTextureLoader<LumaA<f32>>, TiledTextureLoader<Rgba<u8>>);

#[allow(dead_code)]
impl Metadata {
    // Custom serialization of some values
    pub fn from_dir<P: AsRef<std::path::Path>>(dir: P) -> Result<Self, Error> {
        let dir = dir.as_ref().to_owned();
        let meta_path = dir.join("meta.json");
        let reader = File::open(meta_path)?;
        let meta_serde = serde_json::from_reader(reader).map_err(|e| {
            let msg = format!("Could not parse meta.json: {}", e.to_string());
            Error::new(ErrorKind::InvalidData, msg)
        })?;
        Ok(Self::from_serde(meta_serde, dir))
    }

    pub fn read_tiles(&self) -> Result<TextureLoaders, Error> {
        let height_tiles = self.tile_positions.iter().map(|xy| {
            let (x, y) = xy;
            let height_file_path = self.dir.join(format!("x{:08}_y{:08}.height", x, y));
            (*xy, height_file_path)
        });
        let color_tiles = self.tile_positions.iter().map(|xy| {
            let (x, y) = xy;
            let color_file_path = self.dir.join(format!("x{:08}_y{:08}.color", x, y));
            (*xy, color_file_path)
        });
        let height_tiles = TiledTextureLoader::new(self.tile_size, height_tiles)?;
        let color_tiles = TiledTextureLoader::new(self.tile_size, color_tiles)?;
        Ok((height_tiles, color_tiles))
    }

    pub fn write(&self) -> Result<(), Error> {
        let meta_serde = self.clone().into_serde();
        let writer = File::create(self.dir.join("meta.json"))?;
        serde_json::to_writer(writer, &meta_serde).map_err(|e| {
            let msg = format!("Could not write meta.json: {}", e.to_string());
            Error::new(ErrorKind::InvalidData, msg)
        })
    }

    fn from_serde(serde: MetadataSerde, dir: PathBuf) -> Self {
        Metadata {
            tile_size: serde.tile_size,
            world_from_terrain: serde.world_from_terrain.into(),
            origin: serde.origin.into(),
            resolution_m: serde.resolution_m,
            tile_positions: serde.tile_positions,
            dir,
        }
    }

    fn into_serde(self) -> MetadataSerde {
        MetadataSerde {
            tile_size: self.tile_size,
            world_from_terrain: self.world_from_terrain.into(),
            origin: self.origin.into(),
            resolution_m: self.resolution_m,
            tile_positions: self.tile_positions,
        }
    }
}

// Some serde-derivable type we can convert into and from
#[derive(Serialize, Deserialize)]
struct MetadataSerde {
    tile_size: u32,
    world_from_terrain: Isometry3<f64>,
    origin: Vector3<f64>,
    resolution_m: f64,
    tile_positions: Vec<TilePos>,
}
