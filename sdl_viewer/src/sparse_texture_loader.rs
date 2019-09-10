use byteorder::{LittleEndian, ReadBytesExt};
use cgmath::{Decomposed, Quaternion, Vector2, Vector3};
use image::{GenericImage, GenericImageView, ImageBuffer, LumaA, Rgba};
use num_integer::Integer;
use point_viewer::math::Isometry3;
use std::collections::HashMap;
use std::fs::File; // for div_floor
use std::io::{BufReader, Read};

pub struct HeightAndColor {
    pub height: ImageBuffer<LumaA<f32>, Vec<f32>>,
    pub color: ImageBuffer<Rgba<u8>, Vec<u8>>,
}

pub struct SparseTextureLoader {
    origin_x: f64,
    origin_y: f64,
    origin_z: f64,
    resolution_m: f64,
    tile_size: u32,
    tiles: HashMap<(i32, i32), HeightAndColor>,
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

        let mut tiles = HashMap::new();
        let mut height_contents = vec![0.0; tile_size as usize * tile_size as usize * 2];
        let mut color_contents = vec![0; tile_size as usize * tile_size as usize * 4];
        for _ in 0..num_tiles {
            let x = meta.read_i32::<LittleEndian>()?;
            let y = meta.read_i32::<LittleEndian>()?;
            let height_file_path = path.as_ref().join(format!("x{:08}_y{:08}.height", x, y));
            let color_file_path = path.as_ref().join(format!("x{:08}_y{:08}.color", x, y));
            let mut height_rdr = BufReader::new(File::open(height_file_path)?);
            height_rdr.read_f32_into::<LittleEndian>(&mut height_contents)?;
            let height_buffer =
                ImageBuffer::from_raw(tile_size, tile_size, height_contents.clone())
                    .expect("Corrupt contents");
            let mut color_rdr = BufReader::new(File::open(color_file_path)?);
            color_rdr.read_exact(&mut color_contents)?;
            let color_buffer = ImageBuffer::from_raw(tile_size, tile_size, color_contents.clone())
                .expect("Corrupt contents");
            let previous_entry = tiles.insert(
                (x, y),
                HeightAndColor {
                    height: height_buffer,
                    color: color_buffer,
                },
            );
            assert!(previous_entry.is_none());
        }

        let loader = SparseTextureLoader {
            origin_x,
            origin_y,
            origin_z,
            resolution_m,
            tile_size,
            tiles,
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
        // return self.load_dummy2(min_x, min_y, width, height);
        let ts = &i64::from(self.tile_size);
        let (min_tile_x, min_mod_x) = min_x.div_mod_floor(ts);
        let (min_tile_y, min_mod_y) = min_y.div_mod_floor(ts);
        let max_x = min_x + width as i64;
        let max_y = min_y + height as i64;
        let (max_tile_x, max_mod_x) = max_x.div_mod_floor(ts);
        let (max_tile_y, max_mod_y) = max_y.div_mod_floor(ts);
        let mut height_buffer =
            ImageBuffer::from_pixel(width as u32, height as u32, LumaA([0.0; 2]));
        let mut color_buffer = ImageBuffer::from_pixel(width as u32, height as u32, Rgba([0; 4]));
        (min_tile_x..=max_tile_x)
            .flat_map(|tile_x| (min_tile_y..=max_tile_y).map(move |tile_y| (tile_x, tile_y)))
            .for_each(|(tile_x, tile_y)| {
                let x_off_src = if tile_x == min_tile_x {
                    min_mod_x as u32
                } else {
                    0
                };
                let y_off_src = if tile_y == min_tile_y {
                    min_mod_y as u32
                } else {
                    0
                };

                let x_off_dst = tile_x * i64::from(self.tile_size) + i64::from(x_off_src) - min_x;
                let y_off_dst = tile_y * i64::from(self.tile_size) + i64::from(y_off_src) - min_y;

                let len_x = if tile_x == max_tile_x {
                    max_mod_x as u32 - x_off_src
                } else {
                    self.tile_size as u32 - x_off_src
                };

                let len_y = if tile_y == max_tile_y {
                    max_mod_y as u32 - y_off_src
                } else {
                    self.tile_size as u32 - y_off_src
                };
                if let Some(src) = self.tiles.get(&(tile_x as i32, tile_y as i32)) {
                    let height_roi = src.height.view(x_off_src, y_off_src, len_x, len_y);
                    let color_roi = src.color.view(x_off_src, y_off_src, len_x, len_y);
                    height_buffer.copy_from(&height_roi, x_off_dst as u32, y_off_dst as u32);
                    color_buffer.copy_from(&color_roi, x_off_dst as u32, y_off_dst as u32);
                }
            });
        // super::graphic::debug(&buffer, format!("tex_{}_{}_{}_{}.png", min_x, min_y, width, height));
        HeightAndColor {
            height: height_buffer,
            color: color_buffer,
        }
    }

    // fn load_dummy(
    //     &self,
    //     min_x: i64,
    //     min_y: i64,
    //     width: usize,
    //     height: usize,
    // ) -> ImageBuffer<LumaA<f32>, Vec<f32>> {
    //     use std::collections::hash_map::DefaultHasher;
    //     use std::hash::Hasher;
    //     let mut pixels: Vec<f32> = Vec::with_capacity(height * width);
    //     for iy in min_y..min_y + height as i64 {
    //         for ix in min_x..min_x + width as i64 {
    //             let mut h = DefaultHasher::new();
    //             h.write_i64(ix);
    //             h.write_i64(iy);
    //             let hash = h.finish() % 256;
    //             let hash = hash as f32 / 255.0;
    //             pixels.push(hash);
    //             pixels.push(0.0); // TODO
    //         }
    //     }

    //     ImageBuffer::from_raw(width as u32, height as u32, pixels).unwrap()
    // }

    // fn load_dummy2(
    //     &self,
    //     min_x: i64,
    //     min_y: i64,
    //     width: usize,
    //     height: usize,
    // ) -> ImageBuffer<LumaA<f32>, Vec<f32>> {
    //     let mut pixels: Vec<f32> = Vec::with_capacity(height * width);
    //     for iy in min_y..min_y + height as i64 {
    //         for ix in min_x..min_x + width as i64 {
    //             pixels.push(if ix % 5 == 0 || iy % 5 == 0 { 1.0 } else { 0.0 });
    //             pixels.push(131071.0); // TODO
    //         }
    //     }

    //     ImageBuffer::from_raw(width as u32, height as u32, pixels).unwrap()
    // }

    // fn load_dummy3(
    //     &self,
    //     min_x: i64,
    //     min_y: i64,
    //     width: usize,
    //     height: usize,
    // ) -> ImageBuffer<LumaA<f32>, Vec<f32>> {
    //     println!(
    //         "load(min_x={}, min_y={}, width={}, height={})",
    //         min_x, min_y, width, height
    //     );
    //     let mut pixels: Vec<f32> = Vec::with_capacity(height * width);
    //     for iy in min_y..min_y + height as i64 {
    //         for ix in min_x..min_x + width as i64 {
    //             pixels.push((ix + 4) as f32 + (iy + 4) as f32 / 100.0);
    //             pixels.push(0.0); // TODO
    //         }
    //     }
    //     println!("{:?}", pixels);
    //     ImageBuffer::from_raw(width as u32, height as u32, pixels).unwrap()
    // }

    pub fn to_grid_coords(&self, value: &Vector2<f64>) -> Vector2<i64> {
        // TODO: Does this work even for negative values?
        let x = ((value.x - self.origin_x) / self.resolution_m).floor();
        let y = ((value.y - self.origin_y) / self.resolution_m).floor();
        (Vector2::new(x, y)).cast().unwrap()
    }
}
