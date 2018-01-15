// Copyright 2016 The Cartographer Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use cgmath::{Matrix, Matrix4};
use color::Color;
use graphic::{GlBuffer, GlProgram, GlVertexArray};
use opengl;
use opengl::types::{GLboolean, GLint, GLsizeiptr, GLuint};
use point_viewer::math::{Cube, CuboidLike};
use std::mem;
use std::ptr;

const FRAGMENT_SHADER_OUTLINED_BOX: &'static str = include_str!("../shaders/box_drawer_outline.fs");
const VERTEX_SHADER_OUTLINED_BOX: &'static str = include_str!("../shaders/box_drawer_outline.vs");

pub struct BoxDrawer<'a> {
    outline_program: GlProgram<'a>,

    // Uniforms locations.
    u_transform: GLint,
    u_color: GLint,

    // Vertex array and buffers
    vertex_array: GlVertexArray<'a>,
    _buffer_position: GlBuffer<'a>,
    _buffer_indices: GlBuffer<'a>,
}

impl<'a> BoxDrawer<'a> {
    pub fn new(gl: &'a opengl::Gl) -> Self {
        let outline_program =
            GlProgram::new(gl, VERTEX_SHADER_OUTLINED_BOX, FRAGMENT_SHADER_OUTLINED_BOX);
        let u_transform;
        let u_color;

        unsafe {
            gl.UseProgram(outline_program.id);
            u_transform = gl.GetUniformLocation(outline_program.id, c_str!("transform"));
            u_color = gl.GetUniformLocation(outline_program.id, c_str!("color"));
        }

        let vertex_array = GlVertexArray::new(gl);
        vertex_array.bind();

        // vertex buffer: define 8 vertices of the box
        let _buffer_position = GlBuffer::new_array_buffer(gl);
        _buffer_position.bind();
        let vertices: [[f32; 3]; 8] = [
            [-1.0, -1.0, 1.0],  // vertices of front quad
            [1.0, -1.0, 1.0],   //
            [1.0, 1.0, 1.0],    //
            [-1.0, 1.0, 1.0],   //
            [-1.0, -1.0, -1.0], // vertices of back quad
            [1.0, -1.0, -1.0],  //
            [1.0, 1.0, -1.0],   //
            [-1.0, 1.0, -1.0],  //
        ];
        unsafe {
            gl.BufferData(
                opengl::ARRAY_BUFFER,
                (vertices.len() * 3 * mem::size_of::<f32>()) as GLsizeiptr,
                mem::transmute(&vertices[0]),
                opengl::STATIC_DRAW,
            );
        }

        // define index buffer for 24 edges of the box
        let _buffer_indices = GlBuffer::new_element_array_buffer(gl);
        _buffer_indices.bind();
        let line_indices: [[i32; 2]; 12] = [
            [0, 1],
            [1, 2],
            [2, 3],
            [3, 0], // front quad
            [4, 5],
            [5, 6],
            [6, 7],
            [7, 4], // back back
            [1, 5],
            [6, 2], // right quad
            [4, 0],
            [3, 7], // left quad
        ];
        unsafe {
            gl.BufferData(
                opengl::ELEMENT_ARRAY_BUFFER,
                (line_indices.len() * 2 * mem::size_of::<i32>()) as GLsizeiptr,
                mem::transmute(&line_indices[0]),
                opengl::STATIC_DRAW,
            );
        }

        unsafe {
            let pos_attr = gl.GetAttribLocation(outline_program.id, c_str!("position"));
            gl.EnableVertexAttribArray(pos_attr as GLuint);
            gl.VertexAttribPointer(
                pos_attr as GLuint,
                3,
                opengl::FLOAT,
                opengl::FALSE,
                3 * mem::size_of::<f32>() as i32,
                ptr::null(),
            );
        }
        BoxDrawer {
            outline_program,
            u_transform,
            u_color,
            vertex_array,
            _buffer_position,
            _buffer_indices,
        }
    }

    // Draws the outline of the box where each vertex is transformed with 'transform'.
    fn draw_outlines_from_transformation(&self, transform: &Matrix4<f32>, color: &Color) {
        self.vertex_array.bind();

        unsafe {
            self.outline_program.gl.UseProgram(self.outline_program.id);
            self.outline_program.gl.UniformMatrix4fv(
                self.u_transform,
                1,
                false as GLboolean,
                transform.as_ptr(),
            );
            self.outline_program.gl.Uniform4f(
                self.u_color,
                color.red,
                color.green,
                color.blue,
                color.alpha,
            );
            self.outline_program.gl.DrawElements(
                opengl::LINES,
                24,
                opengl::UNSIGNED_INT,
                ptr::null(),
            );
        }
    }

    // Draws the outline of 'cube' using 'color'.
    // Internally, the box is defined in local coordinates.
    // We the properties of 'cube' to transform the box into world space.
    // Then we use 'world_to_gl' to transform it into clip space.
    pub fn draw_outlines(&self, cube: &Cube, world_to_gl: &Matrix4<f32>, color: &Color) {
        let scale_matrix = Matrix4::from_scale(cube.edge_length() / 2.0);
        let translation_matrix = Matrix4::from_translation(cube.center());
        let transformation_matrix = world_to_gl * translation_matrix * scale_matrix;

        self.draw_outlines_from_transformation(&transformation_matrix, &color);
    }
}
