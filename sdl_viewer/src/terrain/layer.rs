use crate::graphic::moving_window_texture::GlMovingWindowTexture;
use crate::graphic::tiled_texture_loader::{TilePos, TiledTextureLoader};
use crate::graphic::uniform::GlUniform;
use crate::graphic::GlProgram;
use byteorder::{LittleEndian, ReadBytesExt};
use cgmath::{Decomposed, Matrix4, Quaternion, Vector2, Vector3};
use image::{ImageBuffer, LumaA, Rgba};
use num_integer::Integer;
use point_viewer::math::Isometry3;
use std::convert::TryInto;
use std::fs::File;
use std::io::ErrorKind;
use std::io::Read;
use std::rc::Rc;

pub struct HeightAndColor {
    pub height: ImageBuffer<LumaA<f32>, Vec<f32>>,
    pub color: ImageBuffer<Rgba<u8>, Vec<u8>>,
}

pub struct TerrainLayer {
    u_origin: GlUniform<Vector3<f64>>,
    u_world_from_terrain: GlUniform<Matrix4<f64>>,
    u_resolution_m: GlUniform<f64>,
    u_terrain_pos_wrapped: GlUniform<Vector2<i32>>,
    terrain_pos: Vector2<i64>, // GLSL doesn't have i64
    height_tiles: TiledTextureLoader<LumaA<f32>>,
    color_tiles: TiledTextureLoader<Rgba<u8>>,
    heightmap: GlMovingWindowTexture<LumaA<f32>>,
    colormap: GlMovingWindowTexture<Rgba<u8>>,
    texture_size: u32,
    // TODO
    pub terrain_from_world: Isometry3<f64>,
}

struct Metadata {
    tile_size: u32,
    world_from_terrain: Isometry3<f64>,
    origin: Vector3<f64>,
    resolution_m: f64,
    tile_positions: Vec<TilePos>,
}

impl TerrainLayer {
    pub fn new<P: AsRef<std::path::Path>>(
        program: &GlProgram,
        path: P,
        texture_size: u32,
    ) -> Result<Self, std::io::Error> {
        println!("Loading terrain.");
        let meta_path = path.as_ref().join("meta");
        let metadata = Self::read_meta(meta_path)?;
        let height_tiles = metadata.tile_positions.iter().map(|xy| {
            let (x, y) = xy;
            let height_file_path = path.as_ref().join(format!("x{:08}_y{:08}.height", x, y));
            (*xy, height_file_path)
        });
        let color_tiles = metadata.tile_positions.iter().map(|xy| {
            let (x, y) = xy;
            let color_file_path = path.as_ref().join(format!("x{:08}_y{:08}.color", x, y));
            (*xy, color_file_path)
        });
        let height_tiles = TiledTextureLoader::new(metadata.tile_size, height_tiles)?;
        let color_tiles = TiledTextureLoader::new(metadata.tile_size, color_tiles)?;

        let u_origin = GlUniform::new(&program, "terrain_origin_m", metadata.origin);
        let u_world_from_terrain = GlUniform::new(
            &program,
            "terrain_to_world",
            Matrix4::from({
                let decomp: Decomposed<_, _> = metadata.world_from_terrain.clone().into();
                decomp
            }),
        );
        let u_resolution_m = GlUniform::new(&program, "terrain_res_m", metadata.resolution_m);
        let terrain_from_world = metadata.world_from_terrain.inverse();

        // Initial terrain pos
        let terrain_pos: Vector2<i64> = Vector2::new(
            ((-metadata.origin.x) / metadata.resolution_m).floor() as i64 - i64::from(texture_size) / 2,
            ((-metadata.origin.y) / metadata.resolution_m).floor() as i64 - i64::from(texture_size) / 2,
        );
        let u_terrain_pos_wrapped = GlUniform::new(
            &program,
            "terrain_pos",
            Vector2::new(
                terrain_pos.x.try_into().unwrap(),
                terrain_pos.y.try_into().unwrap(),
            ),
        );

        let height_initial = height_tiles.load(
            terrain_pos.x,
            terrain_pos.y,
            texture_size.try_into().unwrap(),
            texture_size.try_into().unwrap(),
        );
        let heightmap = GlMovingWindowTexture::new(
            &program,
            Rc::clone(&program.gl),
            "height",
            texture_size,
            0,
            height_initial,
        );

        let color_initial = color_tiles.load(
            terrain_pos.x,
            terrain_pos.y,
            texture_size.try_into().unwrap(),
            texture_size.try_into().unwrap(),
        );
        let colormap = GlMovingWindowTexture::new(
            &program,
            Rc::clone(&program.gl),
            "color",
            texture_size,
            1,
            color_initial,
        );

        println!("Loading terrain complete.");
        Ok(TerrainLayer {
            u_origin,
            u_world_from_terrain,
            u_resolution_m,
            u_terrain_pos_wrapped,
            terrain_pos,
            height_tiles,
            color_tiles,
            heightmap,
            colormap,
            texture_size,
            terrain_from_world,
        })
    }

    // Custom serialization of some values
    fn read_meta<P: AsRef<std::path::Path>>(path: P) -> Result<Metadata, std::io::Error> {
        let mut meta = File::open(path)?;
        const META_SIGNATURE: &[u8; 9] = b"TERRAIN00";
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
        })
    }

    pub fn submit(&self) {
        self.u_origin.submit();
        self.u_world_from_terrain.submit();
        self.u_resolution_m.submit();
        self.u_terrain_pos_wrapped.submit();
        self.heightmap.submit();
        self.colormap.submit();
    }

    pub fn load(&self, min_x: i64, min_y: i64, width: usize, height: usize) -> HeightAndColor {
        HeightAndColor {
            height: self.height_tiles.load(min_x, min_y, width, height),
            color: self.color_tiles.load(min_x, min_y, width, height),
        }
    }

    // We already have the data between prev_pos and prev_pos + texture_size
    // Only fetch the "L" shape that is needed, as separate horizontal and vertical strips.
    // Don't get confused, the horizontal strip is determined by the movement in y direction and
    // the vertical strip is determined by the movement in x direction.
    pub fn update_grid(&mut self, cur_pos: Vector2<i64>) {
        let prev_pos = self.terrain_pos;
        self.terrain_pos = cur_pos;
        let wrapped_x = cur_pos.x.mod_floor(&i64::from(self.texture_size)) as i32;
        let wrapped_y = cur_pos.y.mod_floor(&i64::from(self.texture_size)) as i32;
        self.u_terrain_pos_wrapped.value = Vector2::new(wrapped_x, wrapped_y);
        let moved = cur_pos - prev_pos;

        let hori_strip = if moved.y > 0 {
            self.load(
                cur_pos.x,
                prev_pos.y + i64::from(self.texture_size),
                self.texture_size.try_into().unwrap(),
                moved.y.try_into().unwrap(),
            )
        } else {
            self.load(
                cur_pos.x,
                cur_pos.y,
                self.texture_size.try_into().unwrap(),
                moved.y.abs().try_into().unwrap(),
            )
        };
        let vert_strip = if moved.x > 0 {
            self.load(
                prev_pos.x + i64::from(self.texture_size),
                cur_pos.y,
                moved.x.try_into().unwrap(),
                self.texture_size.try_into().unwrap(),
            )
        } else {
            self.load(
                cur_pos.x,
                cur_pos.y,
                moved.x.abs().try_into().unwrap(),
                self.texture_size.try_into().unwrap(),
            )
        };

        self.heightmap.incremental_update(
            moved.x as i32,
            moved.y as i32,
            vert_strip.height,
            hori_strip.height,
        );
        self.colormap.incremental_update(
            moved.x as i32,
            moved.y as i32,
            vert_strip.color,
            hori_strip.color,
        );
    }

    pub fn to_grid_coords(&self, value: &Vector2<f64>) -> Vector2<i64> {
        // TODO: Does this work even for negative values?
        let x = ((value.x - self.u_origin.value.x) / self.u_resolution_m.value).floor();
        let y = ((value.y - self.u_origin.value.y) / self.u_resolution_m.value).floor();
        (Vector2::new(x, y)).cast().unwrap()
    }
}
