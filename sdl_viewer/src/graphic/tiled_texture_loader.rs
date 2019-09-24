use byteorder::{LittleEndian, ReadBytesExt};
use image::{GenericImage, GenericImageView, ImageBuffer, LumaA, Pixel, Rgba};
use num_integer::Integer;
use std::collections::HashMap;
use std::fs::File;
use std::io::BufReader;
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

pub struct TiledTextureLoader<P: Pixel> {
    tile_size: u32,
    tiles: HashMap<TilePos, ImageBuffer<P, Vec<P::Subpixel>>>,
}

impl<P> TiledTextureLoader<P>
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

        Ok(TiledTextureLoader { tile_size, tiles })
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
                let x_off_src = if tile_x == min_tile_x { min_mod_x } else { 0 };
                // Same for the lower border
                let y_off_src = if tile_y == min_tile_y { min_mod_y } else { 0 };
                // Where to place the tile in the destination image
                let x_off_dst = tile_x * i64::from(self.tile_size) + x_off_src - min_x;
                let y_off_dst = tile_y * i64::from(self.tile_size) + y_off_src - min_y;

                let len_x = if tile_x == max_tile_x {
                    max_mod_x - x_off_src
                } else {
                    i64::from(self.tile_size) - x_off_src
                };

                let len_y = if tile_y == max_tile_y {
                    max_mod_y - y_off_src
                } else {
                    i64::from(self.tile_size) - y_off_src
                };
                if let Some(src) = self.tiles.get(&(tile_x as i32, tile_y as i32)) {
                    let roi = src.view(
                        x_off_src as u32,
                        y_off_src as u32,
                        len_x as u32,
                        len_y as u32,
                    );
                    output_buffer.copy_from(&roi, x_off_dst as u32, y_off_dst as u32);
                }
            }
        }
        output_buffer
    }
}
