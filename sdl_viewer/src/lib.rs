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

use serde_derive::{Deserialize, Serialize};

/// Unsafe macro to create a static null-terminated c-string for interop with OpenGL.
#[macro_export]
macro_rules! c_str {
    ($s:expr) => {
        concat!($s, "\0").as_ptr() as *const i8
    };
}

mod camera;
#[allow(
    non_upper_case_globals,
    clippy::too_many_arguments,
    clippy::unreadable_literal,
    clippy::unused_unit
)]
// TODO(feuerste): Move this up when Rust 1.40 is stable.
#[cfg_attr(clippy_has_missing_safety_doc, allow(clippy::missing_safety_doc))]
pub mod opengl {
    include!(concat!(env!("OUT_DIR"), "/bindings.rs"));
}
pub mod box_drawer;
pub mod graphic;
pub mod node_drawer;
pub mod terrain_drawer;

use crate::box_drawer::BoxDrawer;
use crate::camera::Camera;
use crate::node_drawer::{NodeDrawer, NodeViewContainer};
use crate::terrain_drawer::TerrainRenderer;
use cgmath::{Matrix4, SquareMatrix};
use point_viewer::color::YELLOW;
use point_viewer::data_provider::DataProviderFactory;
use point_viewer::math::Isometry3;
use point_viewer::octree::{self, Octree};
use sdl2::event::{Event, WindowEvent};
use sdl2::keyboard::{Mod, Scancode};
use sdl2::video::{GLProfile, SwapInterval};
use std::cmp;
use std::io;
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::{mpsc, Arc};
use std::thread;

struct PointCloudRenderer {
    gl: Rc<opengl::Gl>,
    node_drawer: NodeDrawer,
    last_moving: time::Instant,
    // TODO(sirver): Logging does not fit into this classes responsibilities.
    last_log: time::Instant,
    visible_nodes: Vec<octree::NodeId>,
    get_visible_nodes_params_tx: mpsc::Sender<Matrix4<f64>>,
    get_visible_nodes_result_rx: mpsc::Receiver<Vec<octree::NodeId>>,
    num_frames: u32,
    point_size: f32,
    gamma: f32,
    needs_drawing: bool,
    max_nodes_in_memory: usize,
    world_to_gl: Matrix4<f64>,
    max_nodes_moving: usize,
    show_octree_nodes: bool,
    node_views: NodeViewContainer,
    box_drawer: BoxDrawer,
}

#[derive(Debug)]
enum DrawResult {
    HasDrawn,
    NoChange,
}
impl PointCloudRenderer {
    pub fn new(
        max_nodes_in_memory: usize,
        gl: Rc<opengl::Gl>,
        octree: Arc<octree::Octree>,
    ) -> Self {
        let now = time::Instant::now();

        // This thread waits for requests to calculate the currently visible nodes, runs a
        // calculation and sends the visible nodes back to the drawing thread. If multiple requests
        // queue up while it is processing one, it will drop all but the latest one before
        // restarting the next calculation.
        let (get_visible_nodes_params_tx, rx) = mpsc::channel::<Matrix4<f64>>();
        let (tx, get_visible_nodes_result_rx) = mpsc::channel();
        let octree_clone = octree.clone();
        thread::spawn(move || {
            while let Ok(mut matrix) = rx.recv() {
                // Drain the channel, we only ever want to update the latest.
                while let Ok(newer_matrix) = rx.try_recv() {
                    matrix = newer_matrix;
                }
                let visible_nodes = octree_clone.get_visible_nodes(&matrix);
                tx.send(visible_nodes).unwrap();
            }
        });

        Self {
            last_moving: now,
            last_log: now,
            visible_nodes: Vec::new(),
            node_drawer: NodeDrawer::new(&Rc::clone(&gl)),
            num_frames: 0,
            point_size: 1.,
            gamma: 1.,
            get_visible_nodes_params_tx,
            get_visible_nodes_result_rx,
            max_nodes_moving: max_nodes_in_memory,
            needs_drawing: true,
            show_octree_nodes: false,
            max_nodes_in_memory,
            node_views: NodeViewContainer::new(octree, max_nodes_in_memory),
            box_drawer: BoxDrawer::new(&Rc::clone(&gl)),
            world_to_gl: Matrix4::identity(),
            gl,
        }
    }

    pub fn camera_changed(&mut self, world_to_gl: &Matrix4<f64>) {
        self.last_moving = time::Instant::now();
        self.needs_drawing = true;
        self.node_drawer.update_world_to_gl(world_to_gl);
        self.get_visible_nodes_params_tx
            .send(world_to_gl.clone())
            .unwrap();
        self.last_moving = time::Instant::now();
        self.world_to_gl = *world_to_gl;
    }

    pub fn toggle_show_octree_nodes(&mut self) {
        self.show_octree_nodes = !self.show_octree_nodes;
    }

    pub fn adjust_gamma(&mut self, delta: f32) {
        self.gamma += delta;
        self.needs_drawing = true;
    }

    pub fn adjust_point_size(&mut self, delta: f32) {
        // Point size == 1. is the smallest that is rendered.
        self.point_size = (self.point_size + delta).max(1.);
        self.needs_drawing = true;
    }

    pub fn draw(&mut self) -> DrawResult {
        let mut draw_result = DrawResult::NoChange;
        let mut num_points_drawn = 0;
        let mut num_nodes_drawn = 0;

        let now = time::Instant::now();
        let moving = now - self.last_moving < time::Duration::milliseconds(150);
        self.needs_drawing |= self.node_views.consume_arrived_nodes(&self.node_drawer);
        while let Ok(visible_nodes) = self.get_visible_nodes_result_rx.try_recv() {
            self.visible_nodes.clear();
            self.visible_nodes.extend(visible_nodes);
            self.needs_drawing = true;
        }

        if self.needs_drawing {
            unsafe {
                self.gl.ClearColor(0., 0., 0., 1.);
                self.gl
                    .Clear(opengl::COLOR_BUFFER_BIT | opengl::DEPTH_BUFFER_BIT);
            }
        }

        // We use a heuristic to keep the frame rate as stable as possible by increasing/decreasing the number of nodes to draw.
        let max_nodes_to_display = if moving {
            self.max_nodes_moving
        } else {
            self.max_nodes_in_memory
        };
        let filtered_visible_nodes = self.visible_nodes.iter().take(max_nodes_to_display);

        for node_id in filtered_visible_nodes {
            let view = self.node_views.get_or_request(&node_id);
            if !self.needs_drawing || view.is_none() {
                continue;
            }
            let view = view.unwrap();
            num_points_drawn += self.node_drawer.draw(
                view,
                1, /* level of detail */
                self.point_size,
                self.gamma,
            );
            num_nodes_drawn += 1;

            if self.show_octree_nodes {
                self.box_drawer.draw_outlines(
                    &view.meta.bounding_cube.to_aabb3(),
                    &self.world_to_gl,
                    &YELLOW,
                );
            }
        }
        if self.needs_drawing {
            draw_result = DrawResult::HasDrawn;
        }
        self.needs_drawing = moving;

        self.num_frames += 1;
        let now = time::Instant::now();
        if now - self.last_log > time::Duration::seconds(1) {
            let duration_s = (now - self.last_log).as_seconds_f64();
            let fps = f64::from(self.num_frames) / duration_s;
            if moving {
                if fps < 20. {
                    self.max_nodes_moving = (self.max_nodes_moving as f32 * 0.9) as usize;
                }
                if fps > 25. && self.max_nodes_moving < self.max_nodes_in_memory {
                    self.max_nodes_moving = (self.max_nodes_moving as f32 * 1.1) as usize;
                }
            }
            self.num_frames = 0;
            self.last_log = now;
            println!(
                "FPS: {:.2}, Drew {} points from {} loaded nodes. {} nodes \
                 should be shown, Cache {} MB",
                fps,
                num_points_drawn,
                num_nodes_drawn,
                self.visible_nodes.len(),
                self.node_views.get_used_memory_bytes() as f32 / 1024. / 1024.,
            );
        }
        draw_result
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct CameraStates {
    states: Vec<camera::State>,
}

fn save_camera(index: usize, pose_path: &Option<PathBuf>, camera: &Camera) {
    if pose_path.is_none() {
        println!("Not serving from a local directory. Cannot save camera.");
        return;
    }
    assert!(index < 10);
    let mut states = ::std::fs::read_to_string(pose_path.as_ref().unwrap())
        .and_then(|data| {
            serde_json::from_str(&data)
                .map_err(|_| io::Error::new(io::ErrorKind::Other, "Could not read camera file."))
        })
        .unwrap_or_else(|_| CameraStates {
            states: vec![camera.state(); 10],
        });
    states.states[index] = camera.state();

    match std::fs::write(
        pose_path.as_ref().unwrap(),
        serde_json::to_string_pretty(&states).unwrap().as_bytes(),
    ) {
        Ok(_) => (),
        Err(e) => println!(
            "Could not write {}: {}",
            pose_path.as_ref().unwrap().display(),
            e
        ),
    }
    println!("Saved current camera position as {}.", index);
}

fn load_camera(index: usize, pose_path: &Option<PathBuf>, camera: &mut Camera) {
    if pose_path.is_none() {
        println!("Not serving from a local directory. Cannot load camera.");
        return;
    }
    assert!(index < 10);
    let states = ::std::fs::read_to_string(pose_path.as_ref().unwrap())
        .and_then(|data| {
            serde_json::from_str(&data)
                .map_err(|_| io::Error::new(io::ErrorKind::Other, "Could not read camera file."))
        })
        .unwrap_or_else(|_| CameraStates {
            states: vec![camera.state(); 10],
        });
    camera.set_state(states.states[index]);
}

pub trait Extension {
    fn pre_init<'a, 'b>(app: clap::App<'a, 'b>) -> clap::App<'a, 'b>;
    fn new(matches: &clap::ArgMatches, opengl: Rc<opengl::Gl>) -> Self;
    fn local_from_global(matches: &clap::ArgMatches, octree: &Octree) -> Option<Isometry3<f64>>;
    fn camera_changed(&mut self, transform: &Matrix4<f64>);
    fn draw(&mut self);
}

trait Joystick {
    fn act(&self, camera: &mut Camera);
    fn joystick(&self) -> &sdl2::joystick::Joystick;
}

struct XBoxJoystick {
    joystick: sdl2::joystick::Joystick,
}

impl Joystick for XBoxJoystick {
    fn act(&self, camera: &mut Camera) {
        let right = f64::from(self.joystick.axis(0).unwrap()) / 1000.;
        let forward = f64::from(self.joystick.axis(1).unwrap()) / 1000.;
        let turning_right = -f64::from(self.joystick.axis(3).unwrap()) / 32000.;
        let turning_up = -f64::from(self.joystick.axis(4).unwrap()) / 32000.;
        camera.pan(right, 0., forward);
        camera.rotate(turning_up, turning_right);
    }

    fn joystick(&self) -> &sdl2::joystick::Joystick {
        &self.joystick
    }
}

struct SpaceMouseJoystick {
    joystick: sdl2::joystick::Joystick,
}

impl Joystick for SpaceMouseJoystick {
    fn act(&self, camera: &mut Camera) {
        let x = f64::from(self.joystick.axis(0).unwrap()) / 500.;
        let y = f64::from(-self.joystick.axis(1).unwrap()) / 500.;
        let z = f64::from(-self.joystick.axis(2).unwrap()) / 500.;
        let up = f64::from(self.joystick.axis(3).unwrap()) / 500.;
        // Combine tilting and turning on the knob.
        let around = f64::from(self.joystick.axis(4).unwrap()) / 500.
            - f64::from(self.joystick.axis(5).unwrap()) / 500.;
        camera.pan(x, y, z);
        camera.rotate(up, around);
    }

    fn joystick(&self) -> &sdl2::joystick::Joystick {
        &self.joystick
    }
}

pub fn run<T: Extension>(data_provider_factory: DataProviderFactory) {
    let mut app = clap::App::new("sdl_viewer").args(&[
        clap::Arg::with_name("octree")
            .help("Input path of the octree.")
            .index(1)
            .required(true),
        clap::Arg::with_name("terrain")
            .long("terrain")
            .takes_value(true)
            .multiple(true)
            .help("Terrain directories (multiple possible)."),
        clap::Arg::with_name("cache_size_mb")
            .help(
                "Maximum cache size in MB for octree nodes in GPU memory. \
                 The default value is 2000 MB and the valid range is 1000 MB to 16000 MB.",
            )
            .required(false),
    ]);
    app = T::pre_init(app);

    let matches = app.get_matches();

    let octree_argument = matches.value_of("octree").unwrap();

    // Maximum number of MB for the octree node cache. The default is 2 GB
    let cache_size_mb: usize = matches
        .value_of("cache_size_mb")
        .unwrap_or("2000")
        .parse()
        .expect("Could not parse 'cache_size_mb' option.");

    // Maximum number of MB for the octree node cache in range 1..16 GB. The default is 2 GB
    let limit_cache_size_mb = cmp::max(1000, cmp::min(16_000, cache_size_mb));

    // Assuming about 200 KB per octree node on average
    let max_nodes_in_memory = limit_cache_size_mb * 5;

    // If no octree was generated create a FromDisk loader
    let octree: Arc<Octree> = Arc::from(
        data_provider_factory
            .generate_data_provider(octree_argument)
            .and_then(|provider| Octree::from_data_provider(provider))
            .unwrap_or_else(|_| panic!("Couldn't create octree from path '{}'.", octree_argument)),
    );

    let mut pose_path = None;
    let pose_path_buf = PathBuf::from(&octree_argument).join("poses.json");
    if pose_path_buf.exists() {
        pose_path = Some(pose_path_buf);
    }

    let ctx = sdl2::init().unwrap();
    let video_subsystem = ctx.video().unwrap();

    // We need to open the joysticks we are interested in and keep the object alive to receive
    // input from it. We just open the first we find.
    let joystick_subsystem = ctx.joystick().unwrap();
    let mut joysticks = Vec::new();
    for idx in 0..joystick_subsystem
        .num_joysticks()
        .expect("Should be able to enumerate joysticks.")
    {
        if let Ok(joystick) = joystick_subsystem.open(idx) {
            let (kind, j) = if joystick.name().contains("Xbox") {
                (
                    "XBox controller",
                    Box::new(XBoxJoystick { joystick }) as Box<dyn Joystick>,
                )
            } else {
                (
                    "Space mouse",
                    Box::new(SpaceMouseJoystick { joystick }) as Box<dyn Joystick>,
                )
            };

            println!(
                "Found a joystick named '{}' ({} axes, {} buttons, {} balls, {} hats). Will treat it as a {}.",
                j.joystick().name(),
                j.joystick().num_axes(),
                j.joystick().num_buttons(),
                j.joystick().num_balls(),
                j.joystick().num_hats(),
                kind
            );
            joysticks.push(j);
        }
    }

    let gl_attr = video_subsystem.gl_attr();

    // TODO(hrapp): This should use OpenGL ES 2.0 to be compatible with WebGL, so this can be made
    // to work with emscripten.
    gl_attr.set_context_profile(GLProfile::Core);
    gl_attr.set_context_version(4, 1);

    const WINDOW_WIDTH: i32 = 800;
    const WINDOW_HEIGHT: i32 = 600;
    let window = match video_subsystem
        .window("sdl2_viewer", WINDOW_WIDTH as u32, WINDOW_HEIGHT as u32)
        .position_centered()
        .resizable()
        .opengl()
        .build()
    {
        Ok(window) => window,
        Err(err) => panic!("failed to create window: {}", err),
    };

    // We need to create a context now, only after can we actually legally load the gl functions
    // and query 'gl_attr'.
    let _context = window.gl_create_context().unwrap();
    let _swap_interval = video_subsystem.gl_set_swap_interval(SwapInterval::VSync);

    assert_eq!(gl_attr.context_profile(), GLProfile::Core);

    let gl = Rc::new(opengl::Gl::load_with(|s| {
        let ptr = video_subsystem.gl_get_proc_address(s);
        ptr as *const std::ffi::c_void
    }));

    let mut extension = T::new(&matches, Rc::clone(&gl));
    let ext_local_from_global = T::local_from_global(&matches, &octree);
    let mut renderer = PointCloudRenderer::new(max_nodes_in_memory, Rc::clone(&gl), octree);
    let terrain_paths = matches.values_of("terrain").unwrap_or_default();
    let mut terrain_renderer = TerrainRenderer::new(Rc::clone(&gl), terrain_paths);
    let local_from_global = ext_local_from_global.or_else(|| terrain_renderer.local_from_global());
    let mut camera = Camera::new(&gl, WINDOW_WIDTH, WINDOW_HEIGHT, local_from_global);

    let mut events = ctx.event_pump().unwrap();
    let mut last_frame_time = time::Instant::now();
    'outer_loop: loop {
        for event in events.poll_iter() {
            match event {
                Event::Quit { .. } => break 'outer_loop,
                Event::KeyDown {
                    scancode: Some(code),
                    keymod,
                    ..
                } => {
                    if keymod.is_empty() || keymod == Mod::NUMMOD {
                        match code {
                            Scancode::Escape => break 'outer_loop,
                            Scancode::W => camera.moving_forward = true,
                            Scancode::S => camera.moving_backward = true,
                            Scancode::A => camera.moving_left = true,
                            Scancode::D => camera.moving_right = true,
                            Scancode::Z => camera.moving_down = true,
                            Scancode::Q => camera.moving_up = true,
                            Scancode::T => camera.toggle_ct_mode(&gl),
                            Scancode::U => camera.move_ct(-0.5, &gl),
                            Scancode::I => camera.move_ct(0.5, &gl),
                            Scancode::J => camera.move_far_plane_ct(-0.5, &gl),
                            Scancode::K => camera.move_far_plane_ct(0.5, &gl),
                            Scancode::Left => camera.turning_left = true,
                            Scancode::Right => camera.turning_right = true,
                            Scancode::Down => camera.turning_down = true,
                            Scancode::Up => camera.turning_up = true,
                            Scancode::O => renderer.toggle_show_octree_nodes(),
                            Scancode::Num7 => renderer.adjust_gamma(-0.1),
                            Scancode::Num8 => renderer.adjust_gamma(0.1),
                            Scancode::Num9 => renderer.adjust_point_size(-0.1),
                            Scancode::Num0 => renderer.adjust_point_size(0.1),
                            _ => (),
                        }
                    } else if keymod.intersects(Mod::LCTRLMOD | Mod::RCTRLMOD)
                        && keymod.intersects(Mod::LSHIFTMOD | Mod::RSHIFTMOD)
                    {
                        // CTRL + SHIFT is pressed.
                        match code {
                            Scancode::Num1 => save_camera(0, &pose_path, &camera),
                            Scancode::Num2 => save_camera(1, &pose_path, &camera),
                            Scancode::Num3 => save_camera(2, &pose_path, &camera),
                            Scancode::Num4 => save_camera(3, &pose_path, &camera),
                            Scancode::Num5 => save_camera(4, &pose_path, &camera),
                            Scancode::Num6 => save_camera(5, &pose_path, &camera),
                            Scancode::Num7 => save_camera(6, &pose_path, &camera),
                            Scancode::Num8 => save_camera(7, &pose_path, &camera),
                            Scancode::Num9 => save_camera(8, &pose_path, &camera),
                            Scancode::Num0 => save_camera(9, &pose_path, &camera),
                            _ => (),
                        }
                    } else if keymod.intersects(Mod::LCTRLMOD | Mod::RCTRLMOD) {
                        // CTRL is pressed.
                        match code {
                            Scancode::Num1 => load_camera(0, &pose_path, &mut camera),
                            Scancode::Num2 => load_camera(1, &pose_path, &mut camera),
                            Scancode::Num3 => load_camera(2, &pose_path, &mut camera),
                            Scancode::Num4 => load_camera(3, &pose_path, &mut camera),
                            Scancode::Num5 => load_camera(4, &pose_path, &mut camera),
                            Scancode::Num6 => load_camera(5, &pose_path, &mut camera),
                            Scancode::Num7 => load_camera(6, &pose_path, &mut camera),
                            Scancode::Num8 => load_camera(7, &pose_path, &mut camera),
                            Scancode::Num9 => load_camera(8, &pose_path, &mut camera),
                            Scancode::Num0 => load_camera(9, &pose_path, &mut camera),
                            _ => (),
                        }
                    }
                }
                Event::KeyUp {
                    scancode: Some(code),
                    ..
                } => match code {
                    Scancode::W => camera.moving_forward = false,
                    Scancode::S => camera.moving_backward = false,
                    Scancode::A => camera.moving_left = false,
                    Scancode::D => camera.moving_right = false,
                    Scancode::Z => camera.moving_down = false,
                    Scancode::Q => camera.moving_up = false,
                    Scancode::Left => camera.turning_left = false,
                    Scancode::Right => camera.turning_right = false,
                    Scancode::Down => camera.turning_down = false,
                    Scancode::Up => camera.turning_up = false,
                    _ => (),
                },
                Event::MouseMotion {
                    xrel,
                    yrel,
                    mousestate,
                    ..
                } => {
                    if mousestate.left() {
                        camera.mouse_drag_rotate(xrel, yrel)
                    } else if mousestate.right() {
                        camera.mouse_drag_pan(xrel, yrel)
                    }
                }
                Event::MouseWheel { y, .. } => {
                    camera.mouse_wheel(y);
                }
                Event::Window {
                    win_event: WindowEvent::SizeChanged(w, h),
                    ..
                } => {
                    camera.set_size(&gl, w, h);
                }
                _ => (),
            }
        }

        for j in &joysticks {
            j.act(&mut camera);
        }
        let current_time = time::Instant::now();
        let elapsed = current_time - last_frame_time;
        last_frame_time = current_time;
        if camera.update(elapsed) {
            renderer.camera_changed(&camera.get_world_to_gl());
            terrain_renderer
                .camera_changed(&camera.get_world_to_gl(), &camera.get_camera_to_world());
            extension.camera_changed(&camera.get_world_to_gl());
        }

        match renderer.draw() {
            DrawResult::HasDrawn => {
                terrain_renderer.draw();
                extension.draw();
                window.gl_swap_window()
            }
            DrawResult::NoChange => (),
        }
    }
}
