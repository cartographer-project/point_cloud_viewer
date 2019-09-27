use crate::graphic::moving_window_texture::GlMovingWindowTexture;
use crate::graphic::tiled_texture_loader::{TilePos, TiledTextureLoader};
use crate::graphic::uniform::GlUniform;
use crate::graphic::GlProgram;
use crate::terrain::read_write::Metadata;
use cgmath::{Decomposed, Matrix4, Quaternion, Vector2, Vector3};
use image::{ImageBuffer, LumaA, Rgba};
use point_viewer::math::Isometry3;
use std::convert::TryInto;

use std::rc::Rc;

const F64_MAX_INT: i64 = 9007199254740992;

struct HeightAndColor {
    height: ImageBuffer<LumaA<f32>, Vec<f32>>,
    color: ImageBuffer<Rgba<u8>, Vec<u8>>,
}

pub struct TerrainLayer {
    u_origin: GlUniform<Vector3<f64>>,
    u_world_from_terrain: GlUniform<Matrix4<f64>>,
    u_resolution_m: GlUniform<f64>,
    terrain_pos: Vector2<i64>,
    u_terrain_pos: GlUniform<Vector2<f64>>,
    height_tiles: TiledTextureLoader<LumaA<f32>>,
    color_tiles: TiledTextureLoader<Rgba<u8>>,
    heightmap: GlMovingWindowTexture<LumaA<f32>>,
    colormap: GlMovingWindowTexture<Rgba<u8>>,
    texture_size: u32,
    // TODO
    pub terrain_from_world: Isometry3<f64>,
}

impl TerrainLayer {
    pub fn new<P: AsRef<std::path::Path>>(
        program: &GlProgram,
        path: P,
        texture_size: u32,
    ) -> Result<Self, std::io::Error> {
        println!("Loading terrain.");
        let metadata = Metadata::from_dir(path)?;
        let (height_tiles, color_tiles) = metadata.read_tiles()?;

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
            ((-metadata.origin.x) / metadata.resolution_m).floor() as i64
                - i64::from(texture_size) / 2,
            ((-metadata.origin.y) / metadata.resolution_m).floor() as i64
                - i64::from(texture_size) / 2,
        );
        let u_terrain_pos = GlUniform::new(
            &program,
            "terrain_pos",
            Vector2::new(terrain_pos.x as f64, terrain_pos.y as f64),
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
            terrain_pos,
            u_terrain_pos,
            height_tiles,
            color_tiles,
            heightmap,
            colormap,
            texture_size,
            terrain_from_world,
        })
    }

    pub fn submit(&self) {
        self.u_origin.submit();
        self.u_world_from_terrain.submit();
        self.u_resolution_m.submit();
        self.u_terrain_pos.submit();
        self.heightmap.submit();
        self.colormap.submit();
    }

    fn load(&self, min_x: i64, min_y: i64, width: usize, height: usize) -> HeightAndColor {
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
        assert!(
            cur_pos.x < F64_MAX_INT && cur_pos.x > -F64_MAX_INT,
            "Terrain location not representable."
        );
        assert!(
            cur_pos.y < F64_MAX_INT && cur_pos.y > -F64_MAX_INT,
            "Terrain location not representable."
        );
        self.u_terrain_pos.value = cur_pos.cast().unwrap();
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
