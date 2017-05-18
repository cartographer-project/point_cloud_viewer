extern crate sdl2;

extern crate point_viewer;
extern crate time;
extern crate cgmath;

use cgmath::{InnerSpace, Rad, Deg, Vector3, Vector2, Zero, Matrix4, Matrix, Array, One, Rotation,
             Rotation3, Decomposed, Transform, Quaternion, Angle};
use point_viewer::math::CuboidLike;
use point_viewer::octree;
use sdl2::event::{Event, WindowEvent};
use sdl2::keyboard::Scancode;
use sdl2::video::GLProfile;
use std::ffi::CString;
use std::mem;
use std::path::PathBuf;
use std::process;
use std::ptr;
use std::str;

#[allow(non_upper_case_globals)]
mod gl {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}

use gl::types::{GLint, GLuint, GLchar, GLenum, GLboolean, GLsizeiptr};

const FRAGMENT_SHADER: &'static str = include_str!("../shaders/points.fs");
const VERTEX_SHADER: &'static str = include_str!("../shaders/points.vs");

// Found in glhelper (MIT License) crate and modified to our needs.
pub fn compile_shader(code: &str, kind: GLenum) -> GLuint {
    let shader;
    unsafe {
        shader = gl::CreateShader(kind);
        let c_str = CString::new(code.as_bytes()).unwrap();
        gl::ShaderSource(shader, 1, &c_str.as_ptr(), ptr::null());
        gl::CompileShader(shader);
        let mut status = gl::FALSE as GLint;
        gl::GetShaderiv(shader, gl::COMPILE_STATUS, &mut status);
        if status != (gl::TRUE as GLint) {
            let mut len = 0;
            gl::GetShaderiv(shader, gl::INFO_LOG_LENGTH, &mut len);
            let mut buf = Vec::with_capacity(len as usize);
            buf.set_len((len as usize) - 1); // subtract 1 to skip the trailing null character
            gl::GetShaderInfoLog(shader,
                                 len,
                                 ptr::null_mut(),
                                 buf.as_mut_ptr() as *mut GLchar);
            panic!("{}",
                   str::from_utf8(&buf)
                       .ok()
                       .expect("ShaderInfoLog invalid UTF8"));
        }
    }
    shader
}

pub fn link_program(vertex_shader_id: GLuint, fragment_shader_id: GLuint) -> GLuint {
    unsafe {
        let program = gl::CreateProgram();
        gl::AttachShader(program, vertex_shader_id);
        gl::AttachShader(program, fragment_shader_id);
        gl::LinkProgram(program);
        gl::DetachShader(program, vertex_shader_id);
        gl::DetachShader(program, fragment_shader_id);

        let mut status = gl::FALSE as GLint;
        gl::GetProgramiv(program, gl::LINK_STATUS, &mut status);
        if status != (gl::TRUE as GLint) {
            let mut len: GLint = 0;
            gl::GetProgramiv(program, gl::INFO_LOG_LENGTH, &mut len);
            let mut buf = Vec::with_capacity(len as usize);
            buf.set_len((len as usize) - 1); // subtract 1 to skip the trailing null character
            gl::GetProgramInfoLog(program,
                                  len,
                                  ptr::null_mut(),
                                  buf.as_mut_ptr() as *mut GLchar);
            panic!("{}",
                   str::from_utf8(&buf)
                       .ok()
                       .expect("ProgramInfoLog invalid UTF8"));
        }
        program
    }
}

struct GlProgram {
    id: GLuint,
}

impl GlProgram {
    fn new(vertex_shader: &str, fragment_shader: &str) -> Self {
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
    fn drop(&mut self) {}
}

// Constructs a projection matrix. Math lifted from ThreeJS.
fn make_projection_matrix<A: Into<Rad<f32>>>(near: f32,
                                             far: f32,
                                             fov: A,
                                             zoom: f32,
                                             aspect_ratio: f32)
                                             -> Matrix4<f32> {
    let top = 0.5 * near * fov.into().tan() / zoom;
    let height = 2. * top;
    let width = aspect_ratio * height;
    let left = -0.5 * width;

    let right = left + width;
    let bottom = top - height;

    // Matrix is column major.
    let x = 2. * near / (right - left);
    let y = 2. * near / (top - bottom);

    let a = (right + left) / (right - left);
    let b = (top + bottom) / (top - bottom);
    let c = -(far + near) / (far - near);
    let d = -2. * far * near / (far - near);

    Matrix4::new(
        x, 0., 0., 0., // Column 0
        0., y, 0., 0., // Column 1
        a, b, c, -1., // Column 2
        0., 0., d, 0. // Column 3
    )
}

#[derive(Debug,PartialEq)]
enum MouseState {
    Rotating,
    None,
}

#[derive(Debug)]
struct Camera {
    movement_speed: f32,
    mouse_state: MouseState,
    rotate_start: Vector2<f32>,
    theta: Rad<f32>,
    phi: Rad<f32>,

    transform: Decomposed<Vector3<f32>, Quaternion<f32>>,
    moving_backward: bool,
    moving_forward: bool,
    moving_left: bool,
    moving_right: bool,
    moving_down: bool,
    moving_up: bool,

    projection_matrix: Matrix4<f32>,
    width: i32,
    height: i32,
}

impl Camera {
    fn new(width: i32, height: i32) -> Self {
        let mut camera = Camera {
            movement_speed: 1.5,
            moving_backward: false,
            moving_forward: false,
            moving_left: false,
            moving_right: false,
            moving_down: false,
            moving_up: false,
            mouse_state: MouseState::None,
            rotate_start: Vector2::zero(),
            theta: Rad(0.),
            phi: Rad(0.),
            transform: Decomposed {
                scale: 1.,
                rot: Quaternion::one(),
                disp: Vector3::new(0., 0., 150.),
            },

            // These will be set by set_size().
            projection_matrix: One::one(),
            width: 0,
            height: 0,
        };
        camera.set_size(width, height);
        camera
    }

    fn set_size(&mut self, width: i32, height: i32) {
        self.width = width;
        self.height = height;
        self.projection_matrix =
            make_projection_matrix(0.1, 10000., Deg(45.), 1., width as f32 / height as f32);
        unsafe {
            gl::Viewport(0, 0, width, height);
        }
    }

    fn get_world_to_gl(&self) -> Matrix4<f32> {
        let world_to_camera: Matrix4<f32> = self.transform.inverse_transform().unwrap().into();
        self.projection_matrix * world_to_camera
    }

    fn update(&mut self) {
        let mut pan = Vector3::zero();
        if self.moving_right {
            pan.x += 1.;
        }
        if self.moving_left {
            pan.x -= 1.;
        }
        if self.moving_backward {
            pan.z += 1.;
        }
        if self.moving_forward {
            pan.z -= 1.;
        }
        if self.moving_up {
            pan.y += 1.;
        }
        if self.moving_down {
            pan.y -= 1.;
        }

        if pan.magnitude2() > 0. {
            let translation = self.transform
                .rot
                .rotate_vector(pan.normalize() * self.movement_speed);
            self.transform.disp += translation;
        }

        let rotation_z = Quaternion::from_angle_z(self.theta);
        let rotation_x = Quaternion::from_angle_x(self.phi);
        self.transform.rot = rotation_z * rotation_x;
    }

    fn start_rotating(&mut self, x: i32, y: i32) {
        self.mouse_state = MouseState::Rotating;
        self.rotate_start.x = x as f32;
        self.rotate_start.y = y as f32;
    }

    fn stop_rotating(&mut self) {
        self.mouse_state = MouseState::None;
    }

    fn mouse_motion(&mut self, x: i32, y: i32) {
        if self.mouse_state == MouseState::None {
            return;
        }
        let end = Vector2::new(x as f32, y as f32);
        let delta = end - self.rotate_start;
        self.rotate_start = end;
        self.theta -= Rad(2. * std::f32::consts::PI * delta.x / self.width as f32);
        self.phi -= Rad(2. * std::f32::consts::PI * delta.y / self.height as f32);
    }

    fn mouse_wheel(&mut self, delta: i32) {
        let sign = delta.signum() as f32;
        self.movement_speed += sign * 0.1 * self.movement_speed;
        self.movement_speed = self.movement_speed.max(0.1);
    }
}

struct GlBuffer {
    id: GLuint,
}

impl GlBuffer {
    fn new() -> Self {
        let mut id = 0;
        unsafe {
            gl::GenBuffers(1, &mut id);
        }
        GlBuffer { id }
    }

    fn bind(&self) {
        unsafe {
            gl::BindBuffer(gl::ARRAY_BUFFER, self.id);
        }
    }
}

impl Drop for GlBuffer {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteBuffers(1, &mut self.id);
        }
    }
}

struct GlVertexArray {
    id: GLuint,
}

impl GlVertexArray {
    fn new() -> Self {
        let mut id = 0;
        unsafe {
            gl::GenVertexArrays(1, &mut id);
        }
        GlVertexArray { id }
    }

    fn bind(&self) {
        unsafe {
            gl::BindVertexArray(self.id);
        }
    }
}

impl Drop for GlVertexArray {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteVertexArrays(1, &mut self.id);
        }
    }
}

struct NodeDrawer {
    program: GlProgram,

    // Uniforms locations.
    u_world_to_gl: GLint,
    u_edge_length: GLint,
    u_min: GLint,
}

impl NodeDrawer {
    fn new() -> Self {
        let program = GlProgram::new(VERTEX_SHADER, FRAGMENT_SHADER);
        let u_world_to_gl;
        let u_edge_length;
        let u_min;
        unsafe {
            gl::UseProgram(program.id);
            gl::Enable(gl::PROGRAM_POINT_SIZE);
            gl::Enable(gl::DEPTH_TEST);

            u_world_to_gl = gl::GetUniformLocation(program.id,
                                                   CString::new("world_to_gl").unwrap().as_ptr());
            u_edge_length = gl::GetUniformLocation(program.id,
                                                   CString::new("edge_length").unwrap().as_ptr());
            u_min = gl::GetUniformLocation(program.id, CString::new("min").unwrap().as_ptr());
        }
        NodeDrawer {
            program,
            u_world_to_gl,
            u_edge_length,
            u_min,
        }
    }

    fn update_world_to_gl(&self, matrix: &Matrix4<f32>) {
        unsafe {
            gl::UniformMatrix4fv(self.u_world_to_gl,
                                 1,
                                 false as GLboolean,
                                 mem::transmute(matrix.as_ptr()));
        }
    }

    fn draw(&self, node_view: &NodeView) -> i64 {
        node_view.vertex_array.bind();
        unsafe {
            gl::Uniform1f(self.u_edge_length,
                          node_view.meta.bounding_cube.edge_length());
            gl::Uniform3fv(self.u_min,
                           1,
                           mem::transmute(node_view.meta.bounding_cube.min().as_ptr()));
            gl::DrawArrays(gl::POINTS, 0, node_view.meta.num_points as i32);
        }
        node_view.meta.num_points
    }
}

struct NodeView {
    meta: octree::NodeMeta,

    // The buffers are bound by 'vertex_array', so we never refer to them. But they must outlive
    // this 'NodeView'.
    vertex_array: GlVertexArray,
    _buffer_position: GlBuffer,
    _buffer_color: GlBuffer,
}

impl NodeView {
    fn new(program: &GlProgram, node_data: octree::NodeData) -> Self {
        let vertex_array = GlVertexArray::new();
        vertex_array.bind();

        let buffer_position = GlBuffer::new();
        let buffer_color = GlBuffer::new();

        unsafe {
            buffer_position.bind();
            let (normalize, data_type) = match node_data.meta.position_encoding {
                octree::PositionEncoding::Uint8 => (true, gl::UNSIGNED_BYTE),
                octree::PositionEncoding::Uint16 => (true, gl::UNSIGNED_SHORT),
                octree::PositionEncoding::Float32 => (false, gl::FLOAT),
            };
            gl::BufferData(gl::ARRAY_BUFFER,
                           node_data.position.len() as GLsizeiptr,
                           mem::transmute(&node_data.position[0]),
                           gl::STATIC_DRAW);

            // Specify the layout of the vertex data
            let pos_attr = gl::GetAttribLocation(program.id,
                                                 CString::new("position").unwrap().as_ptr());
            gl::EnableVertexAttribArray(pos_attr as GLuint);
            gl::VertexAttribPointer(pos_attr as GLuint,
                                    3,
                                    data_type,
                                    normalize as GLboolean,
                                    0,
                                    ptr::null());

            buffer_color.bind();
            gl::BufferData(gl::ARRAY_BUFFER,
                           node_data.color.len() as GLsizeiptr,
                           mem::transmute(&node_data.color[0]),
                           gl::STATIC_DRAW);
            let color_attr = gl::GetAttribLocation(program.id,
                                                   CString::new("color").unwrap().as_ptr());
            gl::EnableVertexAttribArray(color_attr as GLuint);
            gl::VertexAttribPointer(color_attr as GLuint,
                                    3,
                                    gl::UNSIGNED_BYTE,
                                    gl::FALSE as GLboolean,
                                    0,
                                    ptr::null());
        }
        NodeView {
            vertex_array,
            _buffer_position: buffer_position,
            _buffer_color: buffer_color,
            meta: node_data.meta,
        }
    }
}

fn main() {
    let directory = PathBuf::from("octree/");
    let otree = octree::Octree::new(&directory).unwrap();

    let ctx = sdl2::init().unwrap();
    let video_subsystem = ctx.video().unwrap();

    let gl_attr = video_subsystem.gl_attr();

    // TODO(hrapp): This should use OpenGL ES 2.0 to be compatible with WebGL, so this can be made
    // to work with emscripten.
    gl_attr.set_context_profile(GLProfile::Core);
    gl_attr.set_context_version(3, 2);

    const WINDOW_WIDTH: i32 = 800;
    const WINDOW_HEIGHT: i32 = 600;
    let window = match video_subsystem
              .window("sdl2_viewer", WINDOW_WIDTH as u32, WINDOW_HEIGHT as u32)
              .position_centered()
              .resizable()
              .opengl()
              .build() {
        Ok(window) => window,
        Err(err) => panic!("failed to create window: {}", err),
    };

    let gl_attr = video_subsystem.gl_attr();

    assert_eq!(gl_attr.context_profile(), GLProfile::Core);

    gl::load_with(|s| {
                      let ptr = video_subsystem.gl_get_proc_address(s);
                      unsafe { std::mem::transmute(ptr) }
                  });

    println!("SDL_GL_RED_SIZE: {}", gl_attr.red_size());
    println!("SDL_GL_GREEN_SIZE: {}", gl_attr.green_size());
    println!("SDL_GL_BLUE_SIZE: {}", gl_attr.blue_size());
    println!("SDL_GL_ALPHA_SIZE: {}", gl_attr.alpha_size());
    println!("SDL_GL_BUFFER_SIZE: {}", gl_attr.buffer_size());
    println!("SDL_GL_DOUBLEBUFFER: {}", gl_attr.double_buffer());
    println!("SDL_GL_DEPTH_SIZE: {}", gl_attr.depth_size());
    println!("SDL_GL_STENCIL_SIZE: {}", gl_attr.stencil_size());
    println!("SDL_GL_STEREO: {}", gl_attr.stereo());
    println!("SDL_GL_MULTISAMPLEBUFFERS: {}",
             gl_attr.multisample_buffers());
    println!("SDL_GL_MULTISAMPLESAMPLES: {}",
             gl_attr.multisample_samples());
    println!("SDL_GL_ACCELERATED_VISUAL: {}",
             gl_attr.accelerated_visual());
    println!("SDL_GL_CONTEXT_MAJOR_VERSION: {}",
             gl_attr.context_major_version());
    println!("SDL_GL_CONTEXT_MINOR_VERSION: {}",
             gl_attr.context_minor_version());
    println!("SDL_GL_SHARE_WITH_CURRENT_CONTEXT: {}",
             gl_attr.share_with_current_context());
    println!("SDL_GL_FRAMEBUFFER_SRGB_CAPABLE: {}",
             gl_attr.framebuffer_srgb_compatible());

    // We need to keep the context alive while we are using SDL.
    // TODO(hrapp): This seems to be required for Mac OS X and does not hurt on Linux, but to my
    // understanding, a context should already be created at this point.
    let _context = window.gl_create_context().unwrap();
    video_subsystem.gl_set_swap_interval(1);

    let node_drawer = NodeDrawer::new();

    let mut camera = Camera::new(WINDOW_WIDTH, WINDOW_HEIGHT);

    let m = camera.get_world_to_gl();
    let mut node_views = Vec::new();
    for node in otree.get_visible_nodes(&m, camera.width, camera.height, octree::UseLod::No) {
        // We always request nodes at full resolution (i.e. not subsampled by the backend), because
        // we can just as effectively subsample the number of points we draw in the client.
        const ALL_POINTS_LOD: i32 = 1;
        let node_data = otree.get_node_data(&node.id, ALL_POINTS_LOD).unwrap();
        node_views.push(NodeView::new(&node_drawer.program, node_data));
    }

    let mut events = ctx.event_pump().unwrap();
    let mut num_frames = 0;
    let mut last_log = time::PreciseTime::now();
    let mut main_loop = || {
        for event in events.poll_iter() {
            match event {
                Event::Quit { .. } => process::exit(1),
                Event::KeyDown { scancode: Some(code), .. } => {
                    match code {
                        Scancode::Escape => process::exit(1),
                        Scancode::W => camera.moving_forward = true,
                        Scancode::S => camera.moving_backward = true,
                        Scancode::A => camera.moving_left = true,
                        Scancode::D => camera.moving_right = true,
                        Scancode::Z => camera.moving_down = true,
                        Scancode::Q => camera.moving_up = true,
                        _ => (),
                    }
                }
                Event::KeyUp { scancode: Some(code), .. } => {
                    match code {
                        Scancode::W => camera.moving_forward = false,
                        Scancode::S => camera.moving_backward = false,
                        Scancode::A => camera.moving_left = false,
                        Scancode::D => camera.moving_right = false,
                        Scancode::Z => camera.moving_down = false,
                        Scancode::Q => camera.moving_up = false,
                        _ => (),
                    }
                }
                Event::MouseButtonDown { which: 0, x, y, .. } => {
                    camera.start_rotating(x, y);
                }
                Event::MouseButtonUp { which: 0, .. } => {
                    camera.stop_rotating();
                }
                Event::MouseMotion { x, y, .. } => {
                    camera.mouse_motion(x, y);
                }
                Event::MouseWheel { y, .. } => {
                    camera.mouse_wheel(y);
                }
                Event::Window { win_event: WindowEvent::SizeChanged(w, h), .. } => {
                    camera.set_size(w, h);
                }
                _ => (),
            }
        }

        camera.update();
        node_drawer.update_world_to_gl(&camera.get_world_to_gl());

        let mut num_points_drawn = 0;
        unsafe {
            gl::ClearColor(0., 1., 0., 1.);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

            for view in &node_views {
                num_points_drawn += node_drawer.draw(&view);
            }
        }

        window.gl_swap_window();
        num_frames += 1;
        let now = time::PreciseTime::now();
        if last_log.to(now) > time::Duration::seconds(1) {
            let duration = last_log.to(now).num_microseconds().unwrap();
            let fps = (num_frames * 1_000_000u32) as f32 / duration as f32;
            num_frames = 0;
            last_log = now;
            println!("FPS: {:#?}, num_points: {}", fps, num_points_drawn);
        }
    };

    loop {
        main_loop();
    }
}
