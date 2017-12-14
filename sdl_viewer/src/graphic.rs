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

use gl;
use gl::types::GLuint;
use glhelper::{compile_shader, link_program};
use std::str;

pub struct GlProgram {
    pub id: GLuint,
}

impl GlProgram {
    pub fn new(vertex_shader: &str, fragment_shader: &str) -> Self {
        let vertex_shader_id = compile_shader(vertex_shader, gl::VERTEX_SHADER);
        let fragment_shader_id = compile_shader(fragment_shader, gl::FRAGMENT_SHADER);
        let id = link_program(vertex_shader_id, fragment_shader_id);

        // TODO(hrapp): Pull out some saner abstractions around program compilation.
        unsafe {
            gl::DeleteShader(vertex_shader_id);
            gl::DeleteShader(fragment_shader_id);
        }

        GlProgram { id }
    }
}

impl Drop for GlProgram {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteProgram(self.id);
        }
    }
}

pub struct GlBuffer {
    id: GLuint,
}

impl GlBuffer {
    pub fn new() -> Self {
        let mut id = 0;
        unsafe {
            gl::GenBuffers(1, &mut id);
        }
        GlBuffer { id }
    }

    pub fn bind(&self, buffer_type: GLuint) {
        unsafe {
            gl::BindBuffer(buffer_type, self.id);
        }
    }
}

impl Drop for GlBuffer {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteBuffers(1, &self.id);
        }
    }
}

pub struct GlVertexArray {
    id: GLuint,
}

impl GlVertexArray {
    pub fn new() -> Self {
        let mut id = 0;
        unsafe {
            gl::GenVertexArrays(1, &mut id);
        }
        GlVertexArray { id }
    }

    pub fn bind(&self) {
        unsafe {
            gl::BindVertexArray(self.id);
        }
    }
}

impl Drop for GlVertexArray {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteVertexArrays(1, &self.id);
        }
    }
}
