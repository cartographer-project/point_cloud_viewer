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

extern crate cgmath;
extern crate point_viewer;
extern crate rand;
extern crate sdl2;
extern crate time;
#[macro_use]
extern crate sdl_viewer;
extern crate clap;

use cgmath::{Array, Matrix, Matrix4};
use point_viewer::math::CuboidLike;
use point_viewer::octree;
use rand::{Rng, thread_rng};
use sdl2::event::{Event, WindowEvent};
use sdl2::keyboard::Scancode;
use sdl2::video::GLProfile;
use sdl_viewer::{Camera, gl};
use sdl_viewer::box_drawer::BoxDrawer;
use sdl_viewer::color::YELLOW;
use sdl_viewer::gl::types::{GLboolean, GLint, GLsizeiptr, GLuint};
use sdl_viewer::graphic::{GlBuffer, GlProgram, GlVertexArray};
use std::collections::{HashMap, HashSet};
use std::collections::hash_map::Entry;
use std::mem;
use std::path::PathBuf;
use std::ptr;
use std::str;
use std::sync::mpsc::{Receiver, Sender, self};
use std::sync::Arc;

const FRAGMENT_SHADER: &'static str = include_str!("../shaders/points.fs");
const VERTEX_SHADER: &'static str = include_str!("../shaders/points.vs");

fn reshuffle(new_order: &[usize], old_data: Vec<u8>, bytes_per_vertex: usize) -> Vec<u8> {
    assert_eq!(new_order.len() * bytes_per_vertex, old_data.len());
    let mut new_data = Vec::with_capacity(old_data.len());
    for point_index in new_order {
        let i = point_index * bytes_per_vertex;
        new_data.extend(&old_data[i..i + bytes_per_vertex]);
    }
    assert_eq!(old_data.len(), new_data.len());
    new_data
}

struct NodeDrawer {
    program: GlProgram,

    // Uniforms locations.
    u_world_to_gl: GLint,
    u_edge_length: GLint,
    u_size: GLint,
    u_gamma: GLint,
    u_min: GLint,
}

impl NodeDrawer {
    fn new() -> Self {
        let program = GlProgram::new(VERTEX_SHADER, FRAGMENT_SHADER);
        let u_world_to_gl;
        let u_edge_length;
        let u_size;
        let u_gamma;
        let u_min;
        unsafe {
            gl::UseProgram(program.id);

            u_world_to_gl = gl::GetUniformLocation(program.id, c_str!("world_to_gl"));
            u_edge_length = gl::GetUniformLocation(program.id, c_str!("edge_length"));
            u_size = gl::GetUniformLocation(program.id, c_str!("size"));
            u_gamma = gl::GetUniformLocation(program.id, c_str!("gamma"));
            u_min = gl::GetUniformLocation(program.id, c_str!("min"));
        }
        NodeDrawer {
            program,
            u_world_to_gl,
            u_edge_length,
            u_size,
            u_gamma,
            u_min,
        }
    }

    fn update_world_to_gl(&self, matrix: &Matrix4<f32>) {
        unsafe {
            gl::UseProgram(self.program.id);            
            gl::UniformMatrix4fv(self.u_world_to_gl, 1, false as GLboolean, matrix.as_ptr());
        }
    }

    fn draw(&self, node_view: &NodeView, level_of_detail: i32, point_size: f32, gamma: f32) -> i64 {
        node_view.vertex_array.bind();
        let num_points = node_view
            .meta
            .num_points_for_level_of_detail(level_of_detail);
        unsafe {
            gl::UseProgram(self.program.id);
            gl::Enable(gl::PROGRAM_POINT_SIZE);
            gl::Enable(gl::DEPTH_TEST);

            gl::Uniform1f(
                self.u_edge_length,
                node_view.meta.bounding_cube.edge_length(),
            );
            gl::Uniform1f( self.u_size, point_size);
            gl::Uniform1f( self.u_gamma, gamma);
            gl::Uniform3fv(self.u_min, 1, node_view.meta.bounding_cube.min().as_ptr());

            gl::DrawArrays(gl::POINTS, 0, num_points as i32);

            gl::Disable(gl::PROGRAM_POINT_SIZE);
        }
        num_points
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
        unsafe {
            gl::UseProgram(program.id);
        }

        let vertex_array = GlVertexArray::new();
        vertex_array.bind();

        // We draw the points in random order. This allows us to only draw the first N if we want
        // to draw less.
        let mut indices: Vec<usize> = (0..node_data.meta.num_points as usize).collect();
        let mut rng = thread_rng();
        rng.shuffle(&mut indices);

        let position = reshuffle(
            &indices,
            node_data.position,
            match node_data.meta.position_encoding {
                octree::PositionEncoding::Uint8 => 3,
                octree::PositionEncoding::Uint16 => 6,
                octree::PositionEncoding::Float32 => 12,
            },
        );
        let color = reshuffle(&indices, node_data.color, 3);

        let buffer_position = GlBuffer::new_array_buffer();
        let buffer_color = GlBuffer::new_array_buffer();

        unsafe {
            buffer_position.bind();
            let (normalize, data_type) = match node_data.meta.position_encoding {
                octree::PositionEncoding::Uint8 => (true, gl::UNSIGNED_BYTE),
                octree::PositionEncoding::Uint16 => (true, gl::UNSIGNED_SHORT),
                octree::PositionEncoding::Float32 => (false, gl::FLOAT),
            };
            gl::BufferData(
                gl::ARRAY_BUFFER,
                position.len() as GLsizeiptr,
                mem::transmute(&position[0]),
                gl::STATIC_DRAW,
            );

            // Specify the layout of the vertex data.
            let pos_attr = gl::GetAttribLocation(program.id, c_str!("position"));
            gl::EnableVertexAttribArray(pos_attr as GLuint);
            gl::VertexAttribPointer(
                pos_attr as GLuint,
                3,
                data_type,
                normalize as GLboolean,
                0,
                ptr::null(),
            );

            buffer_color.bind();
            gl::BufferData(
                gl::ARRAY_BUFFER,
                color.len() as GLsizeiptr,
                mem::transmute(&color[0]),
                gl::STATIC_DRAW,
            );
            let color_attr = gl::GetAttribLocation(program.id, c_str!("color"));
            gl::EnableVertexAttribArray(color_attr as GLuint);
            gl::VertexAttribPointer(
                color_attr as GLuint,
                3,
                gl::UNSIGNED_BYTE,
                gl::FALSE as GLboolean,
                0,
                ptr::null(),
            );
        }
        NodeView {
            vertex_array,
            _buffer_position: buffer_position,
            _buffer_color: buffer_color,
            meta: node_data.meta,
        }
    }
}

// Keeps track of the nodes that were requested in-order and loads then one by one on request.
struct NodeViewContainer {
    node_views: HashMap<octree::NodeId, NodeView>,
    // The node_ids that the I/O thread is currently loading.
    requested: HashSet<octree::NodeId>,
    // Communication with the I/O thread.
    node_id_sender: Sender<octree::NodeId>,
    node_data_receiver: Receiver<(octree::NodeId, octree::NodeData)>,
}

impl NodeViewContainer {
    fn new(octree: Arc<octree::Octree>) -> Self {
        // We perform I/O in a separate thread in order to not block the main thread while loading.
        // Data sharing is done through channels.
        let (node_id_sender, node_id_receiver) = mpsc::channel();
        let (node_data_sender, node_data_receiver) = mpsc::channel();
        std::thread::spawn(move||{
            // Loads the next node data in the receiver queue.
            for node_id in node_id_receiver.into_iter() {
                // We always request nodes at full resolution (i.e. not subsampled by the backend), because
                // we can just as effectively subsample the number of points we draw in the client.
                const ALL_POINTS_LOD: i32 = 1;                
                let node_data = octree.get_node_data(&node_id, ALL_POINTS_LOD).unwrap();
                // TODO(hrapp): reshuffle
                node_data_sender.send((node_id, node_data)).unwrap();
            } 
        });
        NodeViewContainer {
            node_views: HashMap::new(),
            requested: HashSet::new(),
            node_id_sender: node_id_sender,
            node_data_receiver: node_data_receiver,
        }
    }

    // Returns the 'NodeView' for 'node_id' if it is already loaded, otherwise returns None, but
    // requested the node for loading in the I/O thread
    fn get_or_request(&mut self, node_id: &octree::NodeId, program: &GlProgram) -> Option<&NodeView> {
        while let Ok((node_id, node_data)) = self.node_data_receiver.try_recv() {
            // Put loaded node into hash map.
            self.requested.remove(&node_id);
            self.node_views
                .insert(node_id, NodeView::new(program, node_data));
            // TODO(sirver): Use a LRU Cache to throw nodes out that we haven't used in a while.
        }

        match self.node_views.entry(*node_id) {
            Entry::Vacant(_) => {
                // Limit the number of requested nodes because after a camera move
                // requested nodes might not be in the frustum anymore.
                if !self.requested.contains(&node_id) && self.requested.len() < 10 {  
                    self.requested.insert(*node_id);
                    self.node_id_sender.send(*node_id).unwrap();
                }
                None
            }
            Entry::Occupied(e) => Some(e.into_mut()),
        }
    }

    fn request_all(&mut self, node_ids: &[octree::NodeId]) {
        for &node_id in node_ids {
            match self.node_views.entry(node_id) {
                Entry::Vacant(_) => {
                    if !self.requested.contains(&node_id) {
                        self.requested.insert(node_id);
                        self.node_id_sender.send(node_id).unwrap();
                    }
                }
                Entry::Occupied(_) => {},
            }
        }
    }
}

fn main() {
    let matches = clap::App::new("sdl_viewer")
        .args(
            &[
                clap::Arg::with_name("octree_directory")
                    .help("Input directory of the octree directory to serve.")
                    .index(1)
                    .required(true),
            ]
        )
        .get_matches();

    let octree_directory = PathBuf::from(matches.value_of("octree_directory").unwrap());
    let octree = Arc::new(octree::Octree::new(&octree_directory).unwrap());

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

    // We need to create a context now, only after can we actually legally load the gl functions
    // and query 'gl_attr'.
    let _context = window.gl_create_context().unwrap();
    video_subsystem.gl_set_swap_interval(1);

    assert_eq!(gl_attr.context_profile(), GLProfile::Core);

    gl::load_with(
        |s| {
            let ptr = video_subsystem.gl_get_proc_address(s);
            unsafe { std::mem::transmute(ptr) }
        }
    );

    let node_drawer = NodeDrawer::new();
    let mut node_views = NodeViewContainer::new(octree.clone());
    let mut visible_nodes = Vec::new();

    let box_drawer = BoxDrawer::new();
    let octree_box_color = YELLOW;
    let mut show_octree_nodes = false;

    let mut camera = Camera::new(WINDOW_WIDTH, WINDOW_HEIGHT);

    let mut events = ctx.event_pump().unwrap();
    let mut num_frames = 0;
    let mut last_log = time::PreciseTime::now();
    let mut force_load_all = false;
    let mut show_octree_nodes = false;
    let mut use_level_of_detail;
    let mut point_size = 2.;
    let mut gamma = 1.;

    'outer_loop: loop {
        for event in events.poll_iter() {
            match event {
                Event::Quit { .. } => break 'outer_loop,
                Event::KeyDown { scancode: Some(code), .. } => {
                    match code {
                        Scancode::Escape => break 'outer_loop,
                        Scancode::W => camera.moving_forward = true,
                        Scancode::S => camera.moving_backward = true,
                        Scancode::A => camera.moving_left = true,
                        Scancode::D => camera.moving_right = true,
                        Scancode::Z => camera.moving_down = true,
                        Scancode::Q => camera.moving_up = true,
                        Scancode::F => force_load_all = true,
                        Scancode::O => show_octree_nodes = !show_octree_nodes,
                        Scancode::Num7 => gamma -= 0.1,
                        Scancode::Num8 => gamma += 0.1,
                        Scancode::Num9 => point_size -= 0.1,
                        Scancode::Num0 => point_size += 0.1,
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
                Event::MouseMotion {
                    xrel,
                    yrel,
                    mousestate,
                    ..
                } if mousestate.left() => camera.mouse_drag(xrel, yrel),
                Event::MouseWheel { y, .. } => {
                    camera.mouse_wheel(y);
                }
                Event::Window { win_event: WindowEvent::SizeChanged(w, h), .. } => {
                    camera.set_size(w, h);
                }
                _ => (),
            }
        }

        if camera.update() {
            use_level_of_detail = true;
            node_drawer.update_world_to_gl(&camera.get_world_to_gl());
            visible_nodes = octree.get_visible_nodes(
                &camera.get_world_to_gl(),
                camera.width,
                camera.height,
                octree::UseLod::Yes,
            );
        } else {
            use_level_of_detail = false;
        }

        if force_load_all {
            println!("Force loading all currently visible nodes.");
            let visible_node_ids: Vec<_> = visible_nodes.iter().map(|n|{n.id}).collect();
            node_views.request_all(&visible_node_ids);
            force_load_all = false;
        }

        let mut num_points_drawn = 0;
        let mut num_nodes_drawn = 0;
        unsafe {
            gl::ClearColor(0., 0., 0., 1.);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

            for visible_node in &visible_nodes {
                // TODO(sirver): Track a point budget here when moving, so that FPS never drops too
                // low.
                if let Some(view) = node_views.get_or_request(&visible_node.id, &node_drawer.program) {
                    num_points_drawn += node_drawer.draw(
                        view,
                        if use_level_of_detail {
                            visible_node.level_of_detail
                        } else {
                            1
                        },
                        point_size, gamma
                    );
                    num_nodes_drawn += 1;

                    // debug drawer
                    if show_octree_nodes {
                        box_drawer.draw_outlines(&view.meta.bounding_cube, &camera.get_world_to_gl(), &octree_box_color);
                    }
                }
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
            println!(
                "FPS: {:#?}, Drew {} points from {} loaded nodes. {} nodes should be shown.",
                fps,
                num_points_drawn,
                num_nodes_drawn,
                visible_nodes.len()
            );
        }
    }
}
