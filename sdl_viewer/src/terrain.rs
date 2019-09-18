use crate::graphic::moving_window_texture::GlMovingWindowTexture;
use crate::graphic::uniform::GlUniform;
use crate::graphic::{GlBuffer, GlProgram, GlVertexArray};
use crate::opengl;
use crate::sparse_texture_loader::SparseTextureLoader;
use crate::{c_str, Extension};
use cgmath::{Decomposed, Matrix4, SquareMatrix, Vector2, Zero};
use image::{LumaA, Rgba};
use opengl::types::{GLsizeiptr, GLuint};
use point_viewer::math::Isometry3;
use point_viewer::octree::Octree;
use std::convert::{TryInto, TryFrom};
use std::ffi::c_void;
use std::mem;
use std::rc::Rc;

const TERRAIN_FRAGMENT_SHADER: &str = include_str!("../shaders/terrain.fs");
const TERRAIN_VERTEX_SHADER: &str = include_str!("../shaders/terrain.vs");
const TERRAIN_GEOMETRY_SHADER: &str = include_str!("../shaders/terrain.gs");

const GRID_SIZE: usize = 1023;
const INIT_TERRAIN_POS: i64 = -((GRID_SIZE + 1) as i64 / 2);

#[allow(dead_code)]
pub struct TerrainRenderer {
    program: GlProgram,
    u_transform: GlUniform<Matrix4<f64>>,
    u_terrain_pos: GlUniform<Vector2<i32>>,
    heightmap: GlMovingWindowTexture<LumaA<f32>>,
    colormap: GlMovingWindowTexture<Rgba<u8>>,
    camera_pos_xy_m: Vector2<f64>,
    vertex_array: GlVertexArray,
    buffer_position: GlBuffer,
    buffer_indices: GlBuffer,
    num_indices: usize,
    sparse_heightmap: SparseTextureLoader,
    terrain_to_world: Isometry3<f64>,
}

impl TerrainRenderer {
    pub fn new<P: AsRef<std::path::Path>>(gl: Rc<opengl::Gl>, heightmap_path: P) -> Self {
        let (sparse_heightmap, terrain_to_world) =
            SparseTextureLoader::new(heightmap_path).unwrap();

        let program = GlProgram::new_with_geometry_shader(
            Rc::clone(&gl),
            TERRAIN_VERTEX_SHADER,
            TERRAIN_FRAGMENT_SHADER,
            TERRAIN_GEOMETRY_SHADER,
        );
        let vertex_array = GlVertexArray::new(Rc::clone(&gl));

        // These need to be set only once
        GlUniform::new(&program, "grid_size", GRID_SIZE as f64).submit();

        GlUniform::new(&program, "terrain_res_m", sparse_heightmap.resolution()).submit();

        GlUniform::new(
            &program,
            "terrain_to_world",
            Matrix4::from({
                let decomp: Decomposed<_, _> = terrain_to_world.clone().into();
                decomp
            }),
        )
        .submit();

        GlUniform::new(&program, "terrain_origin_m", sparse_heightmap.origin()).submit();

        let u_transform = GlUniform::new(&program, "world_to_gl", Matrix4::identity());

        let initial_terrain_pos = sparse_heightmap.to_grid_coords(&Vector2::new(0.0, 0.0))
            + Vector2::new(INIT_TERRAIN_POS, INIT_TERRAIN_POS);

        let u_terrain_pos = GlUniform::new(
            &program,
            "terrain_pos",
            Vector2::new(
                initial_terrain_pos.x.try_into().unwrap(),
                initial_terrain_pos.y.try_into().unwrap(),
            ),
        );

        let (buffer_position, buffer_indices, num_indices) =
            Self::create_mesh(&program, &vertex_array, Rc::clone(&gl));

        let height_and_color = sparse_heightmap.load(
            initial_terrain_pos.x,
            initial_terrain_pos.y,
            GRID_SIZE + 1,
            GRID_SIZE + 1,
        );
        let heightmap = GlMovingWindowTexture::new(
            &program,
            Rc::clone(&gl),
            "height",
            u32::try_from(GRID_SIZE + 1).unwrap(),
            0,
            height_and_color.height,
        );

        let colormap = GlMovingWindowTexture::new(
            &program,
            Rc::clone(&gl),
            "color",
            u32::try_from(GRID_SIZE + 1).unwrap(),
            1,
            height_and_color.color,
        );

        let camera_pos_xy_m = Vector2::zero();

        Self {
            program,
            u_transform,
            u_terrain_pos,
            heightmap,
            colormap,
            camera_pos_xy_m,
            vertex_array,
            buffer_position,
            #[allow(dead_code)]
            buffer_indices,
            num_indices,
            sparse_heightmap,
            terrain_to_world,
        }
    }

    fn create_mesh(
        program: &GlProgram,
        vertex_array: &GlVertexArray,
        gl: Rc<opengl::Gl>,
    ) -> (GlBuffer, GlBuffer, usize) {
        let num_vertices = (GRID_SIZE + 1) * (GRID_SIZE + 1) * 3 as usize;
        let mut vertices: Vec<i32> = Vec::with_capacity(num_vertices);
        for iy in 0..=GRID_SIZE as i32 {
            for ix in 0..=GRID_SIZE as i32 {
                vertices.push(ix);
                vertices.push(iy);
                vertices.push(0);
            }
        }

        let flat_ix = |x: GLuint, y: GLuint| y * (GRID_SIZE + 1) as GLuint + x;
        let mut indices: Vec<GLuint> = Vec::with_capacity(GRID_SIZE * GRID_SIZE * 3 * 2);
        for iy in 0..GRID_SIZE as GLuint {
            for ix in 0..GRID_SIZE as GLuint {
                indices.push(flat_ix(ix, iy));
                indices.push(flat_ix(ix + 1, iy));
                indices.push(flat_ix(ix, iy + 1));
                indices.push(flat_ix(ix + 1, iy));
                indices.push(flat_ix(ix, iy + 1));
                indices.push(flat_ix(ix + 1, iy + 1));
            }
        }

        vertex_array.bind();

        let buffer_position = GlBuffer::new_array_buffer(Rc::clone(&gl));
        let buffer_indices = GlBuffer::new_element_array_buffer(Rc::clone(&gl));

        buffer_position.bind();
        unsafe {
            program.gl.BufferData(
                opengl::ARRAY_BUFFER,
                (vertices.len() * mem::size_of::<i32>()) as GLsizeiptr,
                vertices.as_ptr() as *const c_void,
                opengl::STATIC_DRAW,
            );
        }

        unsafe {
            let pos_attr = gl.GetAttribLocation(program.id, c_str!("aPos"));
            gl.EnableVertexAttribArray(pos_attr as GLuint);
            gl.VertexAttribIPointer(
                pos_attr as GLuint,
                3,
                opengl::INT,
                3 * mem::size_of::<i32>() as i32,
                std::ptr::null(),
            );
        }

        buffer_indices.bind();
        unsafe {
            program.gl.BufferData(
                opengl::ELEMENT_ARRAY_BUFFER,
                (indices.len() * mem::size_of::<GLuint>()) as GLsizeiptr,
                indices.as_ptr() as *const c_void,
                opengl::STATIC_DRAW,
            );
        }
        (buffer_position, buffer_indices, indices.len())
    }

    // ======================================= End setup =======================================

    pub fn camera_changed(&mut self, world_to_gl: &Matrix4<f64>, camera_to_world: &Matrix4<f64>) {
        // let pos = camera_to_world.w.clone().truncate();
        // let lower_corner: Vector3<f64> =
        //     pos - Vector3::new(HALF_EXTENT_M, HALF_EXTENT_M, 0.0);
        // let lower_corner: Vector2<i64> =
        //     discretize(&lower_corner.truncate(), TERRAIN_RES_M);
        // let pixels = dummy_heightmap(lower_corner.x, lower_corner.y, GRID_SIZE + 1, GRID_SIZE + 1);
        // self.heightmap.update(pixels.as_ptr() as *const c_void);
        // TODO: Do not compute inverse each time
        let terrain_from_world: Decomposed<_, _> = self.terrain_to_world.inverse().into();
        let camera_to_terrain: Matrix4<f64> = Matrix4::from(terrain_from_world) * camera_to_world;
        let cur_camera_pos_xy_m = Vector2::new(camera_to_terrain.w.x, camera_to_terrain.w.y);
        use cgmath::InnerSpace;
        if (cur_camera_pos_xy_m - self.camera_pos_xy_m).magnitude() > 1000.0 {
            println!("Movement too large");
            return;
        }
        self.update(self.camera_pos_xy_m, cur_camera_pos_xy_m);

        self.u_transform.value = *world_to_gl;
        self.camera_pos_xy_m = cur_camera_pos_xy_m;
    }

    fn update(&mut self, prev_camera_pos: Vector2<f64>, camera_pos: Vector2<f64>) {
        let prev_lower_corner: Vector2<i64> =
            self.sparse_heightmap.to_grid_coords(&prev_camera_pos)
                + Vector2::new(INIT_TERRAIN_POS, INIT_TERRAIN_POS);

        let cur_lower_corner: Vector2<i64> = self.sparse_heightmap.to_grid_coords(&camera_pos)
            + Vector2::new(INIT_TERRAIN_POS, INIT_TERRAIN_POS);
        self.u_terrain_pos.value = Vector2::new(
            cur_lower_corner
                .x
                .try_into()
                .expect("Terrain index too large"),
            cur_lower_corner
                .y
                .try_into()
                .expect("Terrain index too large"),
        );

        // We already have the data between prev_lower_corner and prev_lower_corner + size
        // Only fetch the "L" shape that is needed, as separate horizontal and vertical strips.
        // Don't get confused, the horizontal strip is determined by the movement in y direction and
        // the vertical strip is determined by the movement in x direction.
        let moved = cur_lower_corner - prev_lower_corner;
        const TEX_SIZE: usize = GRID_SIZE + 1;
        let hori_strip = if moved.y > 0 {
            self.sparse_heightmap.load(
                cur_lower_corner.x,
                prev_lower_corner.y + GRID_SIZE as i64 + 1,
                TEX_SIZE,
                moved.y.try_into().unwrap(),
            )
        } else {
            self.sparse_heightmap.load(
                cur_lower_corner.x,
                cur_lower_corner.y,
                TEX_SIZE,
                moved.y.abs().try_into().unwrap(),
            )
        };
        let vert_strip = if moved.x > 0 {
            self.sparse_heightmap.load(
                prev_lower_corner.x + GRID_SIZE as i64 + 1,
                cur_lower_corner.y,
                moved.x.try_into().unwrap(),
                TEX_SIZE,
            )
        } else {
            self.sparse_heightmap.load(
                cur_lower_corner.x,
                cur_lower_corner.y,
                moved.x.abs().try_into().unwrap(),
                TEX_SIZE,
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
        // if moved.x != 0 || moved.y != 0 {
        //     crate::graphic::debug(
        //         &self.heightmap.debug_tex,
        //         format!(
        //             "dbg_{}_{}.png",
        //             self.heightmap.texture_offset.x, self.heightmap.texture_offset.y
        //         ),
        //     );
        // }
        // debug(&self.heightmap);
    }

    pub fn draw(&mut self) {
        unsafe {
            self.vertex_array.bind();
            self.program.gl.UseProgram(self.program.id);
            self.program
                .gl
                .PolygonMode(opengl::FRONT_AND_BACK, opengl::LINE);
            // self.program.gl.Disable(opengl::CULL_FACE);

            self.u_transform.submit();
            self.u_terrain_pos.submit();

            self.heightmap.submit();
            // self.colormap.submit();

            self.program.gl.Enable(opengl::BLEND);
            self.program
                .gl
                .BlendFunc(opengl::SRC_ALPHA, opengl::ONE_MINUS_SRC_ALPHA);
            self.program.gl.DrawElements(
                opengl::TRIANGLES,
                self.num_indices as i32,
                opengl::UNSIGNED_INT,
                std::ptr::null(),
            );
            self.program.gl.Disable(opengl::BLEND);
        }
    }
}

pub struct TerrainExtension {
    terrain_renderer: TerrainRenderer,
}

impl Extension for TerrainExtension {
    fn pre_init<'a, 'b>(app: clap::App<'a, 'b>) -> clap::App<'a, 'b> {
        app.arg(
            clap::Arg::with_name("terrain")
                .long("terrain")
                .takes_value(true)
                .help("Terrain directory."),
        )
    }

    fn new(matches: &clap::ArgMatches, opengl: Rc<opengl::Gl>) -> Self {
        TerrainExtension {
            terrain_renderer: TerrainRenderer::new(opengl, matches.value_of("terrain").unwrap()),
        }
    }

    fn local_from_global(
        &self,
        _matches: &clap::ArgMatches,
        octree: &Octree,
    ) -> Option<Isometry3<f64>> {
        // use collision::Aabb;
        // let center = octree.bounding_box().center();
        // Some(Isometry3::from(local_frame_from_ecef_transform(center)))
        Some(self.terrain_renderer.terrain_to_world.inverse())
    }

    fn camera_changed(&mut self, transform: &Matrix4<f64>, camera_to_world: &Matrix4<f64>) {
        self.terrain_renderer
            .camera_changed(&transform, &camera_to_world);
    }

    fn draw(&mut self) {
        self.terrain_renderer.draw();
    }
}

// Returns transform needed to go from ECEF to local frame with the specified origin where
// the axes are ENU (east, north, up <in the direction normal to the oblate spheroid
// used as Earth's ellipsoid, which does not generally pass through the center of the Earth>)
// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU
// transcribed from `localFrameFromEcefWgs84()` in avsoftware/src/common/math/geo_math.cc
use cgmath::{Deg, Matrix3, Point3, Quaternion, Rotation, Vector3};
use nav_types::{ECEF, WGS84};

pub fn local_frame_from_ecef_transform(
    local_frame_origin: Point3<f64>,
) -> Decomposed<Vector3<f64>, Quaternion<f64>> {
    const PI_HALF: Deg<f64> = Deg(90.0);

    let origin_vector = Vector3::new(
        local_frame_origin.x,
        local_frame_origin.y,
        local_frame_origin.z,
    );
    let lat_lng_alt = WGS84::from(ECEF::new(origin_vector.x, origin_vector.y, origin_vector.z));
    let rotation_matrix = Matrix3::from_angle_z(-PI_HALF)
        * Matrix3::from_angle_y(Deg(lat_lng_alt.latitude_degrees()) - PI_HALF)
        * Matrix3::from_angle_z(Deg(-lat_lng_alt.longitude_degrees()));
    let rotation = Quaternion::from(rotation_matrix);

    Decomposed {
        scale: 1.0,
        rot: rotation,
        disp: rotation.rotate_vector(-origin_vector),
    }
}
