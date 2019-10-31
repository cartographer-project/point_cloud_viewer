use crate::opengl;
use crate::opengl::types::{GLchar, GLenum, GLint, GLuint};
use std::ffi::CString;
use std::ptr;
use std::rc::Rc;
use std::str;

pub struct GlProgramBuilder<'a> {
    gl: Rc<opengl::Gl>,
    vertex_shader: &'a str,
    geometry_shader: Option<&'a str>,
    fragment_shader: Option<&'a str>,
}

impl<'a> GlProgramBuilder<'a> {
    pub fn new_with_vertex_shader(gl: Rc<opengl::Gl>, vertex_shader: &'a str) -> Self {
        GlProgramBuilder {
            gl,
            vertex_shader,
            geometry_shader: None,
            fragment_shader: None,
        }
    }
    pub fn geometry_shader(mut self, geometry_shader: &'a str) -> Self {
        self.geometry_shader = Some(geometry_shader);
        self
    }
    pub fn fragment_shader(mut self, fragment_shader: &'a str) -> Self {
        self.fragment_shader = Some(fragment_shader);
        self
    }
    pub fn build(self) -> GlProgram {
        let vertex_shader_id = compile_shader(&self.gl, self.vertex_shader, opengl::VERTEX_SHADER);
        let geometry_shader_id = self
            .geometry_shader
            .map(|gs| compile_shader(&self.gl, gs, opengl::GEOMETRY_SHADER));
        let fragment_shader_id = self
            .fragment_shader
            .map(|fs| compile_shader(&self.gl, fs, opengl::FRAGMENT_SHADER));
        let id = link_program(
            &self.gl,
            vertex_shader_id,
            geometry_shader_id,
            fragment_shader_id,
        );
        unsafe {
            self.gl.DeleteShader(vertex_shader_id);
            geometry_shader_id.map(|sid| self.gl.DeleteShader(sid));
            fragment_shader_id.map(|sid| self.gl.DeleteShader(sid));
        }
        GlProgram { gl: self.gl, id }
    }
}

pub struct GlProgram {
    pub gl: Rc<opengl::Gl>,
    pub id: GLuint,
}

impl Drop for GlProgram {
    fn drop(&mut self) {
        unsafe {
            self.gl.DeleteProgram(self.id);
        }
    }
}

fn compile_shader(gl: &opengl::Gl, code: &str, kind: GLenum) -> GLuint {
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

fn link_program(
    gl: &opengl::Gl,
    vertex_shader_id: GLuint,
    geometry_shader_id: Option<GLuint>,
    fragment_shader_id: Option<GLuint>,
) -> GLuint {
    unsafe {
        let program = gl.CreateProgram();
        gl.AttachShader(program, vertex_shader_id);
        if let Some(id) = geometry_shader_id {
            gl.AttachShader(program, id)
        };
        if let Some(id) = fragment_shader_id {
            gl.AttachShader(program, id);
        }
        gl.LinkProgram(program);
        gl.DetachShader(program, vertex_shader_id);
        if let Some(id) = geometry_shader_id {
            gl.DetachShader(program, id)
        };
        if let Some(id) = fragment_shader_id {
            gl.DetachShader(program, id)
        };

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
