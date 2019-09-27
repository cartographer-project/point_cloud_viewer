use crate::graphic::tiled_texture_loader::{TilePos, TiledTextureLoader};
use byteorder::{LittleEndian, ReadBytesExt};
use cgmath::{Decomposed, Quaternion, Vector3};
use image::{LumaA, Rgba};
use point_viewer::math::Isometry3;
use std::fs::File;
use std::io::{ErrorKind, Read};
use std::path::PathBuf;

pub struct Metadata {
    pub tile_size: u32,
    pub world_from_terrain: Isometry3<f64>,
    pub origin: Vector3<f64>,
    pub resolution_m: f64,
    pub tile_positions: Vec<TilePos>,
    pub dir: PathBuf,
}

const META_SIGNATURE: &[u8; 9] = b"TERRAIN00";

impl Metadata {
    // Custom serialization of some values
    pub fn from_dir<P: AsRef<std::path::Path>>(dir: P) -> Result<Self, std::io::Error> {
        let dir = dir.as_ref().to_owned();
        let meta_path = dir.join("meta");
        let mut meta = File::open(meta_path)?;
        let mut signature = [0u8; 9];
        meta.read_exact(&mut signature)?;
        if &signature != META_SIGNATURE {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                "Wrong signature in meta file.",
            ));
        }
        let tile_size = meta.read_u32::<LittleEndian>()?;
        let resolution_m = meta.read_f64::<LittleEndian>()?;
        let origin_x = meta.read_f64::<LittleEndian>()?;
        let origin_y = meta.read_f64::<LittleEndian>()?;
        let origin_z = meta.read_f64::<LittleEndian>()?;
        let origin = Vector3::new(origin_x, origin_y, origin_z);
        let t_x = meta.read_f64::<LittleEndian>()?;
        let t_y = meta.read_f64::<LittleEndian>()?;
        let t_z = meta.read_f64::<LittleEndian>()?;
        let translation = Vector3::new(t_x, t_y, t_z);
        let q_x = meta.read_f64::<LittleEndian>()?;
        let q_y = meta.read_f64::<LittleEndian>()?;
        let q_z = meta.read_f64::<LittleEndian>()?;
        let q_w = meta.read_f64::<LittleEndian>()?;
        let rotation = Quaternion::new(q_w, q_x, q_y, q_z);
        let world_from_terrain = Decomposed {
            scale: 1.0,
            disp: translation,
            rot: rotation,
        }
        .into();
        let num_tiles = meta.read_u32::<LittleEndian>()?;
        let mut tile_positions = Vec::with_capacity(num_tiles as usize);
        for _ in 0..num_tiles {
            let x = meta.read_i32::<LittleEndian>()?;
            let y = meta.read_i32::<LittleEndian>()?;
            tile_positions.push((x, y));
        }
        Ok(Metadata {
            tile_size,
            world_from_terrain,
            origin,
            resolution_m,
            tile_positions,
            dir,
        })
    }

    pub fn read_tiles(
        &self,
    ) -> Result<(TiledTextureLoader<LumaA<f32>>, TiledTextureLoader<Rgba<u8>>), std::io::Error>
    {
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
}
