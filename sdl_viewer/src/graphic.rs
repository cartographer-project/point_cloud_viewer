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

//! Higher level abstractions around core OpenGL concepts.

use crate::glhelper::{compile_shader, link_program};
use crate::opengl::types::GLuint;
use crate::opengl::{self, Gl};
use std::rc::Rc;
use std::str;

pub struct GlProgram {
    pub gl: Rc<Gl>,
    pub id: GLuint,
}

impl GlProgram {
    pub fn new(gl: Rc<opengl::Gl>, vertex_shader: &str, fragment_shader: &str) -> Self {
        let vertex_shader_id = compile_shader(&*gl, vertex_shader, opengl::VERTEX_SHADER);
        let fragment_shader_id = compile_shader(&*gl, fragment_shader, opengl::FRAGMENT_SHADER);
        let id = link_program(&*gl, vertex_shader_id, fragment_shader_id);

        // TODO(hrapp): Pull out some saner abstractions around program compilation.
        unsafe {
            gl.DeleteShader(vertex_shader_id);
            gl.DeleteShader(fragment_shader_id);
        }
        GlProgram { gl, id }
    }
}

impl Drop for GlProgram {
    fn drop(&mut self) {
        unsafe {
            self.gl.DeleteProgram(self.id);
        }
    }
}

pub struct GlBuffer {
    gl: Rc<Gl>,
    id: GLuint,
    buffer_type: GLuint,
}

impl GlBuffer {
    pub fn new_array_buffer(gl: Rc<opengl::Gl>) -> Self {
        let mut id = 0;
        unsafe {
            gl.GenBuffers(1, &mut id);
        }
        GlBuffer {
            gl,
            id,
            buffer_type: opengl::ARRAY_BUFFER,
        }
    }

    pub fn new_element_array_buffer(gl: Rc<opengl::Gl>) -> Self {
        let mut id = 0;
        unsafe {
            gl.GenBuffers(1, &mut id);
        }
        GlBuffer {
            gl,
            id,
            buffer_type: opengl::ELEMENT_ARRAY_BUFFER,
        }
    }

    pub fn bind(&self) {
        unsafe {
            self.gl.BindBuffer(self.buffer_type, self.id);
        }
    }
}

impl Drop for GlBuffer {
    fn drop(&mut self) {
        unsafe {
            self.gl.DeleteBuffers(1, &self.id);
        }
    }
}

pub struct GlVertexArray {
    gl: Rc<Gl>,
    id: GLuint,
}

impl GlVertexArray {
    pub fn new(gl: Rc<Gl>) -> Self {
        let mut id = 0;
        unsafe {
            gl.GenVertexArrays(1, &mut id);
        }
        GlVertexArray { gl, id }
    }

    pub fn bind(&self) {
        unsafe {
            self.gl.BindVertexArray(self.id);
        }
    }
}

impl Drop for GlVertexArray {
    fn drop(&mut self) {
        unsafe {
            self.gl.DeleteVertexArrays(1, &self.id);
        }
    }
}
