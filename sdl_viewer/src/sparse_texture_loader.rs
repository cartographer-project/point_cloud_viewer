use byteorder::{LittleEndian, ReadBytesExt};
use cgmath::{Decomposed, Quaternion, Vector2, Vector3};
use point_viewer::math::Isometry3;
use crate::graphic::tiled_texture_loader::TiledTextureLoader;
use image::{ImageBuffer, LumaA, Rgba};
use std::fs::File;


pub struct HeightAndColor {
    pub height: ImageBuffer<LumaA<f32>, Vec<f32>>,
    pub color: ImageBuffer<Rgba<u8>, Vec<u8>>,
}

pub struct SparseTextureLoader {
    origin_x: f64,
    origin_y: f64,
    origin_z: f64,
    resolution_m: f64,
    height_tiles: TiledTextureLoader<LumaA<f32>>,
    color_tiles: TiledTextureLoader<Rgba<u8>>,
}

impl SparseTextureLoader {
    pub fn new<P: AsRef<std::path::Path>>(
        path: P,
    ) -> Result<(Self, Isometry3<f64>), std::io::Error> {
        println!("Loading terrain");
        let mut meta =
            File::open(path.as_ref().join("meta")).expect("Could not open sparse texture dir");
        let tile_size = meta.read_u32::<LittleEndian>()?;
        let resolution_m = meta.read_f64::<LittleEndian>()?;
        let origin_x = meta.read_f64::<LittleEndian>()?;
        let origin_y = meta.read_f64::<LittleEndian>()?;
        let origin_z = meta.read_f64::<LittleEndian>()?;
        let t_x = meta.read_f64::<LittleEndian>()?;
        let t_y = meta.read_f64::<LittleEndian>()?;
        let t_z = meta.read_f64::<LittleEndian>()?;
        let q_x = meta.read_f64::<LittleEndian>()?;
        let q_y = meta.read_f64::<LittleEndian>()?;
        let q_z = meta.read_f64::<LittleEndian>()?;
        let q_w = meta.read_f64::<LittleEndian>()?;
        let translation = Vector3::new(t_x, t_y, t_z);
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
        let height_tiles = tile_positions.iter().map(|xy| {
            let (x, y) = xy;
            let height_file_path = path.as_ref().join(format!("x{:08}_y{:08}.height", x, y));
            (*xy, height_file_path)
        });
        let color_tiles = tile_positions.iter().map(|xy| {
            let (x, y) = xy;
            let color_file_path = path.as_ref().join(format!("x{:08}_y{:08}.color", x, y));
            (*xy, color_file_path)
        });
        let height_tiles = TiledTextureLoader::new(tile_size, height_tiles)?;
        let color_tiles = TiledTextureLoader::new(tile_size, color_tiles)?;

        let loader = SparseTextureLoader {
            origin_x,
            origin_y,
            origin_z,
            resolution_m,
            height_tiles,
            color_tiles,
        };
        Ok((loader, world_from_terrain))
    }

    pub fn resolution(&self) -> f64 {
        self.resolution_m
    }

    pub fn origin(&self) -> Vector3<f64> {
        Vector3::new(self.origin_x, self.origin_y, self.origin_z)
    }

    pub fn load(&self, min_x: i64, min_y: i64, width: usize, height: usize) -> HeightAndColor {
        // TODO(nnmm): Make more efficient by computing the intersection in a separate step and sharing the resutl
        HeightAndColor {
            height: self.height_tiles.load(min_x, min_y, width, height),
            color: self.color_tiles.load(min_x, min_y, width, height),
        }
    }

    pub fn to_grid_coords(&self, value: &Vector2<f64>) -> Vector2<i64> {
        // TODO: Does this work even for negative values?
        let x = ((value.x - self.origin_x) / self.resolution_m).floor();
        let y = ((value.y - self.origin_y) / self.resolution_m).floor();
        (Vector2::new(x, y)).cast().unwrap()
    }
}
