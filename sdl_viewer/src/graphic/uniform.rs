use crate::graphic::GlProgram;
use crate::opengl;
use crate::opengl::types::{GLboolean, GLint};
use cgmath::{Matrix, Matrix4, Vector2, Vector3};
use std::ffi::CString;
use std::rc::Rc;

pub trait Uniform {
    unsafe fn submit(&self, gl: &opengl::Gl, location: GLint);
}

impl Uniform for f32 {
    unsafe fn submit(&self, gl: &opengl::Gl, location: GLint) {
        gl.Uniform1f(location, *self);
    }
}

impl Uniform for f64 {
    unsafe fn submit(&self, gl: &opengl::Gl, location: GLint) {
        gl.Uniform1d(location, *self);
    }
}

impl Uniform for Matrix4<f64> {
    unsafe fn submit(&self, gl: &opengl::Gl, location: GLint) {
        gl.UniformMatrix4dv(location, 1, false as GLboolean, self.as_ptr());
    }
}

impl Uniform for Vector2<f32> {
    unsafe fn submit(&self, gl: &opengl::Gl, location: GLint) {
        gl.Uniform2f(location, self.x, self.y);
    }
}

impl Uniform for Vector2<f64> {
    unsafe fn submit(&self, gl: &opengl::Gl, location: GLint) {
        gl.Uniform2d(location, self.x, self.y);
    }
}

impl Uniform for Vector2<i32> {
    unsafe fn submit(&self, gl: &opengl::Gl, location: GLint) {
        gl.Uniform2i(location, self.x, self.y);
    }
}

impl Uniform for Vector3<f32> {
    unsafe fn submit(&self, gl: &opengl::Gl, location: GLint) {
        gl.Uniform3f(location, self.x, self.y, self.z);
    }
}

impl Uniform for Vector3<f64> {
    unsafe fn submit(&self, gl: &opengl::Gl, location: GLint) {
        gl.Uniform3d(location, self.x, self.y, self.z);
    }
}

pub struct GlUniform<T> {
    location: GLint,
    gl: Rc<opengl::Gl>,
    pub value: T,
}

impl<T: Uniform> GlUniform<T> {
    pub fn new(program: &GlProgram, name: &str, value: T) -> Self {
        let location;
        let name_c = CString::new(name).unwrap();
        unsafe {
            program.gl.UseProgram(program.id);
            location = program
                .gl
                .GetUniformLocation(program.id, name_c.as_ptr() as *const i8);
        }
        GlUniform {
            location,
            gl: Rc::clone(&program.gl),
            value,
        }
    }

    pub fn submit(&self) {
        unsafe {
            self.value.submit(&self.gl, self.location);
        }
    }
}
