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
        let mut shader_ids = Vec::new();
        shader_ids.push(compile_shader(
            &self.gl,
            self.vertex_shader,
            opengl::VERTEX_SHADER,
        ));
        if let Some(gs) = self.geometry_shader {
            shader_ids.push(compile_shader(&self.gl, gs, opengl::GEOMETRY_SHADER));
        }
        if let Some(fs) = self.fragment_shader {
            shader_ids.push(compile_shader(&self.gl, fs, opengl::FRAGMENT_SHADER));
        }
        let id = link_program(&self.gl, &shader_ids);
        unsafe {
            shader_ids
                .iter()
                .for_each(|shader_id| self.gl.DeleteShader(*shader_id));
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

fn link_program(gl: &opengl::Gl, shader_ids: &[GLuint]) -> GLuint {
    unsafe {
        let program = gl.CreateProgram();
        shader_ids
            .iter()
            .for_each(|shader_id| gl.AttachShader(program, *shader_id));
        gl.LinkProgram(program);
        shader_ids
            .iter()
            .for_each(|shader_id| gl.DetachShader(program, *shader_id));

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
