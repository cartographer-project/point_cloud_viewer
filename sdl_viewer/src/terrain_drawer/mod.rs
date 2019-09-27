use crate::graphic::uniform::GlUniform;
use crate::graphic::{GlBuffer, GlProgram, GlVertexArray};
use crate::opengl;
use crate::terrain_drawer::layer::TerrainLayer;
use crate::{c_str, Extension};
use cgmath::{Decomposed, Matrix4, SquareMatrix, Vector2, Zero};

use opengl::types::{GLsizeiptr, GLuint};
use point_viewer::math::Isometry3;
use point_viewer::octree::Octree;

use std::ffi::c_void;
use std::mem;
use std::rc::Rc;

mod layer;
mod read_write;

const TERRAIN_FRAGMENT_SHADER: &str = include_str!("../../shaders/terrain.fs");
const TERRAIN_VERTEX_SHADER: &str = include_str!("../../shaders/terrain.vs");
const TERRAIN_GEOMETRY_SHADER: &str = include_str!("../../shaders/terrain.gs");

const GRID_SIZE: u32 = 1023;
const INIT_TERRAIN_POS: i64 = -((GRID_SIZE + 1) as i64 / 2);

#[allow(dead_code)]
pub struct TerrainRenderer {
    program: GlProgram,
    u_transform: GlUniform<Matrix4<f64>>,
    camera_pos_xy_m: Vector2<f64>,
    vertex_array: GlVertexArray,
    buffer_position: GlBuffer,
    buffer_indices: GlBuffer,
    num_indices: usize,
    terrain_layer: TerrainLayer,
}

impl TerrainRenderer {
    pub fn new<P: AsRef<std::path::Path>>(gl: Rc<opengl::Gl>, heightmap_path: P) -> Self {
        let program = GlProgram::new_with_geometry_shader(
            Rc::clone(&gl),
            TERRAIN_VERTEX_SHADER,
            TERRAIN_FRAGMENT_SHADER,
            TERRAIN_GEOMETRY_SHADER,
        );

        let terrain_layer = TerrainLayer::new(&program, heightmap_path, GRID_SIZE + 1).unwrap();

        let vertex_array = GlVertexArray::new(Rc::clone(&gl));

        // These need to be set only once
        GlUniform::new(&program, "grid_size", GRID_SIZE as f64).submit();

        let u_transform = GlUniform::new(&program, "world_to_gl", Matrix4::identity());

        let (buffer_position, buffer_indices, num_indices) =
            Self::create_mesh(&program, &vertex_array, Rc::clone(&gl));

        let camera_pos_xy_m = Vector2::zero();

        Self {
            program,
            u_transform,
            camera_pos_xy_m,
            vertex_array,
            buffer_position,
            #[allow(dead_code)]
            buffer_indices,
            num_indices,
            terrain_layer,
        }
    }

    fn create_mesh(
        program: &GlProgram,
        vertex_array: &GlVertexArray,
        gl: Rc<opengl::Gl>,
    ) -> (GlBuffer, GlBuffer, usize) {
        let num_vertices = (GRID_SIZE + 1) as usize * (GRID_SIZE + 1) as usize * 3;
        let mut vertices: Vec<i32> = Vec::with_capacity(num_vertices);
        for iy in 0..=GRID_SIZE as i32 {
            for ix in 0..=GRID_SIZE as i32 {
                vertices.push(ix);
                vertices.push(iy);
                vertices.push(0);
            }
        }

        let flat_ix = |x: GLuint, y: GLuint| y * (GRID_SIZE + 1) as GLuint + x;
        let mut indices: Vec<GLuint> =
            Vec::with_capacity(GRID_SIZE as usize * GRID_SIZE as usize * 3 * 2);
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
        // TODO: Do not convert each time
        let terrain_from_world = Matrix4::from({
            let decomp: Decomposed<_, _> = self.terrain_layer.terrain_from_world.clone().into();
            decomp
        });
        let camera_to_terrain: Matrix4<f64> = terrain_from_world * camera_to_world;
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

    fn update(&mut self, _prev_camera_pos: Vector2<f64>, camera_pos: Vector2<f64>) {
        let cur_lower_corner: Vector2<i64> = self.terrain_layer.to_grid_coords(&camera_pos)
            + Vector2::new(INIT_TERRAIN_POS, INIT_TERRAIN_POS);

        // We already have the data between prev_lower_corner and prev_lower_corner + size
        // Only fetch the "L" shape that is needed, as separate horizontal and vertical strips.
        // Don't get confused, the horizontal strip is determined by the movement in y direction and
        // the vertical strip is determined by the movement in x direction.
        self.terrain_layer.update_grid(cur_lower_corner);
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
            self.terrain_layer.submit();

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
        _octree: &Octree,
    ) -> Option<Isometry3<f64>> {
        Some(
            self.terrain_renderer
                .terrain_layer
                .terrain_from_world
                .clone(),
        )
    }

    fn camera_changed(&mut self, transform: &Matrix4<f64>, camera_to_world: &Matrix4<f64>) {
        self.terrain_renderer
            .camera_changed(&transform, &camera_to_world);
    }

    fn draw(&mut self) {
        self.terrain_renderer.draw();
    }
}
