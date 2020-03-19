use crate::graphic::tiled_texture_loader::TiledTextureLoader;
use crate::graphic::{GlMovingWindowTexture, GlProgram, GlUniform};
use crate::terrain_drawer::read_write::Metadata;
use image::{ImageBuffer, LumaA, Rgba};
use nalgebra::{Isometry3, Matrix4, Point3, Vector2, Vector3};
use std::convert::TryInto;
use std::io;
use std::rc::Rc;

// The last integer that can be exactly represented in a f64
const F64_MAX_SAFE_INT: i64 = 9_007_199_254_740_992;
const F64_MIN_SAFE_INT: i64 = -F64_MAX_SAFE_INT;

struct HeightAndColor {
    height: ImageBuffer<LumaA<f32>, Vec<f32>>,
    color: ImageBuffer<Rgba<u8>, Vec<u8>>,
}

pub struct TerrainLayer {
    grid_coordinates: GridCoordinateFrame,
    // The terrain pos is the coordinate of the lower corner of the terrain.
    // It converted to f64 at the latest possible moment, so that all
    // calculations have 64-bit integer precision.
    terrain_pos: Vector2<i64>,
    u_terrain_pos: GlUniform<Vector2<f64>>,
    height_tiles: TiledTextureLoader<LumaA<f32>>,
    color_tiles: TiledTextureLoader<Rgba<u8>>,
    heightmap: GlMovingWindowTexture<LumaA<f32>>,
    colormap: GlMovingWindowTexture<Rgba<u8>>,
    // The texture size shouldn't be negative or larger than u32::MAX, but it's
    // more convenient for our calculations to store it as an i64.
    texture_size: i64,
}

#[allow(dead_code)]
impl TerrainLayer {
    pub fn new<P: AsRef<std::path::Path>>(
        program: &GlProgram,
        path: P,
        texture_size: u32,
    ) -> io::Result<Self> {
        assert!(texture_size % 2 == 0 && texture_size > 0);
        let metadata = Metadata::from_dir(&path)?;
        let (height_tiles, color_tiles) = metadata.read_tiles(&path)?;

        let grid_coordinates = GridCoordinateFrame::new(program, metadata, texture_size);

        // Initial terrain pos
        let terrain_pos: Vector2<i64> =
            grid_coordinates.terrain_pos_for_camera_pos(Point3::new(0.0, 0.0, 0.0));
        let float_terrain_pos = Self::convert_terrain_pos_to_float(terrain_pos);
        let u_terrain_pos = GlUniform::new(&program, "terrain_pos", float_terrain_pos);

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
            0, // texture_unit
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
            1, // texture_unit
            color_initial,
        );
        let texture_size = i64::from(texture_size);

        Ok(TerrainLayer {
            grid_coordinates,
            terrain_pos,
            u_terrain_pos,
            height_tiles,
            color_tiles,
            heightmap,
            colormap,
            texture_size,
        })
    }

    // We already have the data between self.terrain_pos and self.terrain_pos + texture_size
    // Only fetch the "L" shape that is needed, as separate horizontal and vertical strips.
    // Don't get confused, the horizontal strip is determined by the movement in y direction and
    // the vertical strip is determined by the movement in x direction.
    pub fn update(&mut self, cur_world_pos: Point3<f64>) {
        let cur_pos = self
            .grid_coordinates
            .terrain_pos_for_camera_pos(cur_world_pos);
        let moved = cur_pos - self.terrain_pos;

        let hori_strip = if moved.y > 0 {
            self.load(
                cur_pos.x,
                self.terrain_pos.y + self.texture_size,
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
                self.terrain_pos.x + self.texture_size,
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

        self.terrain_pos = cur_pos;
        self.u_terrain_pos.value = Self::convert_terrain_pos_to_float(cur_pos)
    }

    pub fn terrain_from_world(&self) -> &Isometry3<f64> {
        &self.grid_coordinates.terrain_from_world
    }

    pub fn submit(&self) {
        self.grid_coordinates.submit();
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

    // Helper function because OpenGL doesn't like i64
    fn convert_terrain_pos_to_float(v: Vector2<i64>) -> Vector2<f64> {
        assert!(
            v.x < F64_MAX_SAFE_INT && v.x > F64_MIN_SAFE_INT,
            "Terrain location not representable."
        );
        assert!(
            v.y < F64_MAX_SAFE_INT && v.y > F64_MIN_SAFE_INT,
            "Terrain location not representable."
        );
        Vector2::new(v.x as f64, v.y as f64)
    }
}

/// This struct's job is to convert from continuous world positions to discrete
/// terrain positions and to provide the necessary variables to do the inverse
/// calculation to the vertex shader.
struct GridCoordinateFrame {
    u_origin: GlUniform<Vector3<f64>>,
    u_world_from_terrain: GlUniform<Matrix4<f64>>,
    u_resolution_m: GlUniform<f64>,
    texture_half_extent: Vector2<i64>,
    terrain_from_world: Isometry3<f64>,
}

impl GridCoordinateFrame {
    fn new(program: &GlProgram, metadata: Metadata, texture_size: u32) -> Self {
        let u_origin = GlUniform::new(&program, "terrain_origin_m", metadata.origin);
        let terrain_from_world = metadata.world_from_terrain.inverse();
        let u_world_from_terrain = GlUniform::new(
            &program,
            "terrain_to_world",
            metadata.world_from_terrain.to_homogeneous(),
        );
        let u_resolution_m = GlUniform::new(&program, "terrain_res_m", metadata.resolution_m);
        let texture_half_extent =
            Vector2::new(i64::from(texture_size) / 2, i64::from(texture_size) / 2);
        GridCoordinateFrame {
            u_origin,
            u_world_from_terrain,
            u_resolution_m,
            texture_half_extent,
            terrain_from_world,
        }
    }

    /// Returns the terrain pos (i.e. the coordinate of the lower corner of the terrain) for
    /// a given camera position (in the world coordinate system).
    fn terrain_pos_for_camera_pos(&self, world_pos: Point3<f64>) -> Vector2<i64> {
        let local_pos = self.terrain_from_world * world_pos;
        let x = ((local_pos.x - self.u_origin.value.x) / self.u_resolution_m.value).floor();
        let y = ((local_pos.y - self.u_origin.value.y) / self.u_resolution_m.value).floor();
        assert!(
            x <= std::i64::MAX as f64 && x >= std::i64::MIN as f64,
            "Terrain location not representable."
        );
        assert!(
            y <= std::i64::MAX as f64 && y >= std::i64::MIN as f64,
            "Terrain location not representable."
        );
        Vector2::new(x as i64, y as i64) - self.texture_half_extent
    }

    fn submit(&self) {
        self.u_origin.submit();
        self.u_world_from_terrain.submit();
        self.u_resolution_m.submit();
    }
}
