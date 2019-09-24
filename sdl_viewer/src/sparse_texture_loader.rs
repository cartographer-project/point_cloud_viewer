use byteorder::{LittleEndian, ReadBytesExt};
use cgmath::{Decomposed, Quaternion, Vector2, Vector3};
use image::{GenericImage, GenericImageView, ImageBuffer, LumaA, Pixel, Rgba};
use num_integer::Integer;
use point_viewer::math::Isometry3;
use std::collections::HashMap;
use std::fs::File; // for div_floor
use std::io::{BufReader};
use std::path::Path;

/// Little trait that just dispatches to the correct read_xyz_into call for the image type
pub trait ReadLittleEndian {
    fn read_from<R: ReadBytesExt>(&mut self, rdr: &mut R) -> Result<(), std::io::Error>;
}

impl ReadLittleEndian for ImageBuffer<LumaA<f32>, Vec<f32>> {
    fn read_from<R: ReadBytesExt>(&mut self, rdr: &mut R) -> Result<(), std::io::Error> {
        rdr.read_f32_into::<LittleEndian>(self)
    }
}

impl ReadLittleEndian for ImageBuffer<Rgba<u8>, Vec<u8>> {
    fn read_from<R: ReadBytesExt>(&mut self, rdr: &mut R) -> Result<(), std::io::Error> {
        rdr.read_exact(self)
    }
}

type TilePos = (i32, i32);

pub struct TiledTexture<P: Pixel> {
    tile_size: u32,
    tiles: HashMap<TilePos, ImageBuffer<P, Vec<P::Subpixel>>>,
}

impl<P> TiledTexture<P>
where
    P: Pixel + 'static,
    ImageBuffer<P, Vec<P::Subpixel>>: ReadLittleEndian,
{
    pub fn new<I, Q>(tile_size: u32, tile_iter: I) -> Result<Self, std::io::Error>
    where
        I: Iterator<Item = ((i32, i32), Q)>,
        Q: AsRef<Path>,
    {
        let mut tiles = HashMap::new();
        for (xy, file_path) in tile_iter {
            let mut tile = ImageBuffer::new(tile_size, tile_size);
            let mut tile_reader = BufReader::new(File::open(file_path)?);
            tile.read_from(&mut tile_reader)?;
            let previous_entry = tiles.insert(xy, tile);
            assert!(previous_entry.is_none());
        }

        Ok(TiledTexture { tile_size, tiles })
    }

    /// Loads the specified region of the sparse texture into a ImageBuffer
    pub fn load(
        &self,
        min_x: i64,
        min_y: i64,
        width: usize,
        height: usize,
    ) -> ImageBuffer<P, Vec<P::Subpixel>> {
        let ts = &i64::from(self.tile_size);
        let max_x = min_x + width as i64;
        let max_y = min_y + height as i64;
        // Compute the tile position and within-tile position of the tile in the lower left corner
        let (min_tile_x, min_mod_x) = min_x.div_mod_floor(ts);
        let (min_tile_y, min_mod_y) = min_y.div_mod_floor(ts);
        // Same for the upper right corner
        let (max_tile_x, max_mod_x) = max_x.div_mod_floor(ts);
        let (max_tile_y, max_mod_y) = max_y.div_mod_floor(ts);
        let mut output_buffer = ImageBuffer::new(width as u32, height as u32);
        for tile_x in min_tile_x..=max_tile_x {
            for tile_y in min_tile_y..=max_tile_y {
                // At the left border, start with the correct offset
                let x_off_src = if tile_x == min_tile_x {
                    min_mod_x
                } else {
                    0
                };
                // Same for the lower border
                let y_off_src = if tile_y == min_tile_y {
                    min_mod_y
                } else {
                    0
                };
                // Where to place the tile in the destination image
                let x_off_dst = tile_x * i64::from(self.tile_size) + x_off_src - min_x;
                let y_off_dst = tile_y * i64::from(self.tile_size) + y_off_src - min_y;

                let len_x = if tile_x == max_tile_x {
                    max_mod_x - x_off_src
                } else {
                    self.tile_size as i64 - x_off_src
                };

                let len_y = if tile_y == max_tile_y {
                    max_mod_y - y_off_src
                } else {
                    self.tile_size as i64 - y_off_src
                };
                if let Some(src) = self.tiles.get(&(tile_x as i32, tile_y as i32)) {
                    let roi = src.view(x_off_src as u32, y_off_src as u32, len_x as u32, len_y as u32);
                    output_buffer.copy_from(&roi, x_off_dst as u32, y_off_dst as u32);
                }
            }
        }
        output_buffer
    }
}


pub struct HeightAndColor {
    pub height: ImageBuffer<LumaA<f32>, Vec<f32>>,
    pub color: ImageBuffer<Rgba<u8>, Vec<u8>>,
}

pub struct SparseTextureLoader {
    origin_x: f64,
    origin_y: f64,
    origin_z: f64,
    resolution_m: f64,
    height_tiles: TiledTexture<LumaA<f32>>,
    color_tiles: TiledTexture<Rgba<u8>>,
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
        let height_tiles = TiledTexture::new(tile_size, height_tiles)?;
        let color_tiles = TiledTexture::new(tile_size, color_tiles)?;

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
