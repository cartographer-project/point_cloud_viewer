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

use gl;
use glhelper::{compile_shader, link_program};
use graphic::{GlBuffer, GlProgram, GlVertexArray};
use gl::types::{GLboolean, GLint, GLsizeiptr, GLuint};
use std::str;
use std::mem;
use std::ptr;
use cgmath::{Array, Matrix, Matrix4};

const FRAGMENT_SHADER_OUTLINED_BOX: &'static str = include_str!("../shaders/outlinedBox.fs");
const VERTEX_SHADER_OUTLINED_BOX: &'static str = include_str!("../shaders/outlinedBox.vs");

pub struct OutlinedBoxDrawer
{
    program: GlProgram,

    // Uniforms locations.
    u_transform: GLint,
    u_color: GLint,

    // vertex array and buffers
    vertex_array: GlVertexArray,
    _buffer_position: GlBuffer,
    _buffer_indices: GlBuffer,
}

impl OutlinedBoxDrawer {
    pub fn new() -> Self {
        let program = GlProgram::new(VERTEX_SHADER_OUTLINED_BOX, FRAGMENT_SHADER_OUTLINED_BOX);  
        let u_transform;
        let u_color;
    
        unsafe {
            gl::UseProgram(program.id);
            u_transform = gl::GetUniformLocation(program.id, c_str!("transform"));
            u_color = gl::GetUniformLocation(program.id, c_str!("color"));
        }

        let vertex_array = GlVertexArray::new();
        vertex_array.bind();

        // vertex buffer: define 8 vertices of the box
        let _buffer_position = GlBuffer::new();
        _buffer_position.bind(gl::ARRAY_BUFFER);
        let vertices: [f32; 3*8] = [
            -1.0, -1.0, 1.0,
            1.0, -1.0, 1.0,
            1.0,  1.0, 1.0,
            -1.0,  1.0, 1.0,
            -1.0, -1.0, -1.0,
            1.0, -1.0, -1.0,
            1.0,  1.0, -1.0,
            -1.0,  1.0, -1.0,
        ];
        unsafe {
            gl::BufferData(
                gl::ARRAY_BUFFER,
                (vertices.len() * mem::size_of::<f32>()) as GLsizeiptr,
                mem::transmute(&vertices[0]),
                gl::STATIC_DRAW,
            );
        }

        // define index buffer for 24 edges of the box
        let _buffer_indices = GlBuffer::new();
        _buffer_indices.bind(gl::ELEMENT_ARRAY_BUFFER);
        let indices: [i32; 24] = [
            0,1, 1,2, 2,3, 3,0,		// front
		    4,5, 5,6, 6,7, 7,4,		// back
		    1,5, 6,2,				// right
		    4,0, 3,7,				// left
        ];
        unsafe {
            gl::BufferData(
                gl::ELEMENT_ARRAY_BUFFER,
                (indices.len() * mem::size_of::<i32>()) as GLsizeiptr,
                mem::transmute(&indices[0]),
                gl::STATIC_DRAW,
            );
        }

        unsafe{
            let pos_attr = gl::GetAttribLocation(program.id, c_str!("aPos"));
            gl::EnableVertexAttribArray(pos_attr as GLuint);
            gl::VertexAttribPointer(
                pos_attr as GLuint,
                3,
                gl::FLOAT,
                gl::FALSE,
                3 * mem::size_of::<f32>() as i32,
                ptr::null(),
            );
        }
        OutlinedBoxDrawer {
            program,
            u_transform,
            u_color,
            vertex_array,
            _buffer_position,
            _buffer_indices
        }        
    }

    pub fn update_transform(&self, matrix: &Matrix4<f32>) {
        unsafe {
            gl::UseProgram(self.program.id);
            gl::UniformMatrix4fv(self.u_transform, 1, false as GLboolean, matrix.as_ptr());
        }
    }

    pub fn update_color(&self, color: &Vec<f32>) {
        unsafe {
            gl::UseProgram(self.program.id);
            gl::Uniform4fv(self.u_color, 1, color.as_ptr());
        }
    }

    pub fn draw(&self) {
        self.vertex_array.bind();

        unsafe {
            gl::UseProgram(self.program.id);
            gl::Enable(gl::DEPTH_TEST);
            gl::DrawElements(gl::LINES, 24, gl::UNSIGNED_INT, ptr::null());
        }
    }
}
