use crate::c_str;
use crate::graphic::{GlBuffer, GlProgram, GlProgramBuilder, GlUniform, GlVertexArray};
use crate::opengl;
use cgmath::{EuclideanSpace, Matrix4, Point3, SquareMatrix, Vector2, Zero};
use point_viewer::math::Isometry3;

use opengl::types::{GLsizeiptr, GLuint};

use std::ffi::c_void;
use std::mem;
use std::rc::Rc;

mod layer;
mod read_write;

pub use layer::TerrainLayer;
pub use read_write::Metadata;

const TERRAIN_FRAGMENT_SHADER: &str = include_str!("../../shaders/terrain.fs");
const TERRAIN_VERTEX_SHADER: &str = include_str!("../../shaders/terrain.vs");
const TERRAIN_GEOMETRY_SHADER: &str = include_str!("../../shaders/terrain.gs");

const GRID_SIZE: u32 = 1023;

#[allow(dead_code)]
pub struct TerrainRenderer {
    program: GlProgram,
    u_transform: GlUniform<Matrix4<f64>>,
    camera_pos_xy_m: Vector2<f64>,
    vertex_array: GlVertexArray,
    buffer_position: GlBuffer,
    buffer_indices: GlBuffer,
    num_indices: usize,
    terrain_layers: Vec<TerrainLayer>,
}

impl TerrainRenderer {
    pub fn new<I>(gl: Rc<opengl::Gl>, terrain_paths: I) -> Self
    where
        I: Iterator,
        I::Item: AsRef<std::path::Path>,
    {
        let program =
            GlProgramBuilder::new_with_vertex_shader(Rc::clone(&gl), TERRAIN_VERTEX_SHADER)
                .geometry_shader(TERRAIN_GEOMETRY_SHADER)
                .fragment_shader(TERRAIN_FRAGMENT_SHADER)
                .build();

        let u_transform = GlUniform::new(&program, "world_to_gl", Matrix4::identity());

        let camera_pos_xy_m = Vector2::zero();

        let vertex_array = GlVertexArray::new(Rc::clone(&gl));

        let (buffer_position, buffer_indices, num_indices) =
            Self::create_mesh(&program, &vertex_array, Rc::clone(&gl));

        let terrain_layers = terrain_paths
            .map(|p| TerrainLayer::new(&program, p, GRID_SIZE + 1).unwrap())
            .collect();

        Self {
            program,
            u_transform,
            camera_pos_xy_m,
            vertex_array,
            buffer_position,
            #[allow(dead_code)]
            buffer_indices,
            num_indices,
            terrain_layers,
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
        let camera_pos = Point3::from_vec(camera_to_world.w.truncate());
        self.terrain_layers
            .iter_mut()
            .for_each(|layer| layer.update(camera_pos));

        self.u_transform.value = *world_to_gl;
    }

    pub fn draw(&mut self) {
        if self.terrain_layers.is_empty() {
            return;
        }
        unsafe {
            self.vertex_array.bind();
            self.program.gl.UseProgram(self.program.id);
            self.program
                .gl
                .PolygonMode(opengl::FRONT_AND_BACK, opengl::LINE);
            // self.program.gl.Disable(opengl::CULL_FACE);

            self.u_transform.submit();
            self.program.gl.Enable(opengl::BLEND);
            for layer in self.terrain_layers.iter() {
                layer.submit();
                self.program.gl.DrawElements(
                    opengl::TRIANGLES,
                    self.num_indices as i32,
                    opengl::UNSIGNED_INT,
                    std::ptr::null(),
                );
            }

            self.program
                .gl
                .BlendFunc(opengl::SRC_ALPHA, opengl::ONE_MINUS_SRC_ALPHA);
            self.program.gl.Disable(opengl::BLEND);
        }
    }

    pub fn local_from_global(&self) -> Option<Isometry3<f64>> {
        self.terrain_layers.first().map(|layer| layer.terrain_from_world())
    }
}
