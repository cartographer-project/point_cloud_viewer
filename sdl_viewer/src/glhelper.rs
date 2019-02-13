// Copyright 2017 Joseph A Mark <sjeohp@gmail.com>

// Permission is hereby granted, free of charge, to any person obtaining a copy of this software
// and associated documentation files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all copies or
// substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
// BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// This code here was extracted from the glhelper crate. We could not depend on it directly, since
// we generate our own gl module which cannot be easily injected.

use crate::opengl;
use crate::opengl::types::{GLchar, GLenum, GLint, GLuint};
use std::ffi::CString;
use std::ptr;
use std::str;

pub fn compile_shader(gl: &opengl::Gl, code: &str, kind: GLenum) -> GLuint {
    let shader;
    unsafe {
        shader = gl.CreateShader(kind);
        let c_str = CString::new(code.as_bytes()).unwrap();
        gl.ShaderSource(shader, 1, &c_str.as_ptr(), ptr::null());
        gl.CompileShader(shader);
        let mut status = i32::from(opengl::FALSE);
        gl.GetShaderiv(shader, opengl::COMPILE_STATUS, &mut status);
        if status != (i32::from(opengl::TRUE)) {
            let mut len = 0;
            gl.GetShaderiv(shader, opengl::INFO_LOG_LENGTH, &mut len);
            let mut buf = Vec::with_capacity(len as usize);
            buf.set_len((len as usize) - 1); // subtract 1 to skip the trailing null character
            gl.GetShaderInfoLog(
                shader,
                len,
                ptr::null_mut(),
                buf.as_mut_ptr() as *mut GLchar,
            );
            panic!(
                "{}",
                str::from_utf8(&buf).expect("ShaderInfoLog invalid UTF8")
            );
        }
    }
    shader
}

pub fn link_program(
    gl: &opengl::Gl,
    vertex_shader_id: GLuint,
    fragment_shader_id: GLuint,
) -> GLuint {
    unsafe {
        let program = gl.CreateProgram();
        gl.AttachShader(program, vertex_shader_id);
        gl.AttachShader(program, fragment_shader_id);
        gl.LinkProgram(program);
        gl.DetachShader(program, vertex_shader_id);
        gl.DetachShader(program, fragment_shader_id);

        let mut status = i32::from(opengl::FALSE);
        gl.GetProgramiv(program, opengl::LINK_STATUS, &mut status);
        if status != (i32::from(opengl::TRUE)) {
            let mut len: GLint = 0;
            gl.GetProgramiv(program, opengl::INFO_LOG_LENGTH, &mut len);
            let mut buf = Vec::with_capacity(len as usize);
            buf.set_len((len as usize) - 1); // subtract 1 to skip the trailing null character
            gl.GetProgramInfoLog(
                program,
                len,
                ptr::null_mut(),
                buf.as_mut_ptr() as *mut GLchar,
            );
            panic!(
                "{}",
                str::from_utf8(&buf).expect("ProgramInfoLog invalid UTF8")
            );
        }
        program
    }
}
