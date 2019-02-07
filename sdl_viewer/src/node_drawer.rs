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

use cgmath::{Array, Matrix, Matrix4};
use fnv::FnvHashSet;
use graphic::{GlBuffer, GlProgram, GlVertexArray};
use lru_cache::LruCache;
use opengl;
use opengl::types::{GLboolean, GLint, GLsizeiptr, GLuint};
use point_viewer::octree;
use rand::{thread_rng, Rng};
use std;
use std::os::raw::c_void;
use std::ptr;
use std::rc::Rc;
use std::str;
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;

const FRAGMENT_SHADER: &str = include_str!("../shaders/points.fs");
const VERTEX_SHADER: &str = include_str!("../shaders/points.vs");

fn reshuffle(new_order: &[usize], old_data: &[u8], bytes_per_vertex: usize) -> Vec<u8> {
    assert_eq!(new_order.len() * bytes_per_vertex, old_data.len());
    let mut new_data = Vec::with_capacity(old_data.len());
    for point_index in new_order {
        let i = point_index * bytes_per_vertex;
        new_data.extend(&old_data[i..i + bytes_per_vertex]);
    }
    assert_eq!(old_data.len(), new_data.len());
    new_data
}

pub struct NodeDrawer {
    pub program: GlProgram,

    // Uniforms locations.
    u_world_to_gl: GLint,
    u_edge_length: GLint,
    u_size: GLint,
    u_gamma: GLint,
    u_min: GLint,
}

impl NodeDrawer {
    pub fn new(gl: Rc<opengl::Gl>) -> Self {
        let program = GlProgram::new(Rc::clone(&gl), VERTEX_SHADER, FRAGMENT_SHADER);
        let u_world_to_gl;
        let u_edge_length;
        let u_size;
        let u_gamma;
        let u_min;
        unsafe {
            gl.UseProgram(program.id);

            u_world_to_gl = gl.GetUniformLocation(program.id, c_str!("world_to_gl"));
            u_edge_length = gl.GetUniformLocation(program.id, c_str!("edge_length"));
            u_size = gl.GetUniformLocation(program.id, c_str!("size"));
            u_gamma = gl.GetUniformLocation(program.id, c_str!("gamma"));
            u_min = gl.GetUniformLocation(program.id, c_str!("min"));
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

    pub fn update_world_to_gl(&self, matrix: &Matrix4<f32>) {
        unsafe {
            self.program.gl.UseProgram(self.program.id);
            self.program.gl.UniformMatrix4fv(
                self.u_world_to_gl,
                1,
                false as GLboolean,
                matrix.as_ptr(),
            );
        }
    }

    pub fn draw(
        &self,
        node_view: &NodeView,
        level_of_detail: i32,
        point_size: f32,
        gamma: f32,
    ) -> i64 {
        node_view.vertex_array.bind();
        let num_points = node_view
            .meta
            .num_points_for_level_of_detail(level_of_detail);
        unsafe {
            self.program.gl.UseProgram(self.program.id);
            self.program.gl.Enable(opengl::PROGRAM_POINT_SIZE);
            self.program.gl.Enable(opengl::DEPTH_TEST);

            self.program.gl.Uniform1f(
                self.u_edge_length,
                node_view.meta.bounding_cube.edge_length(),
            );
            self.program.gl.Uniform1f(self.u_size, point_size);
            self.program.gl.Uniform1f(self.u_gamma, gamma);
            self.program
                .gl
                .Uniform3fv(self.u_min, 1, node_view.meta.bounding_cube.min().as_ptr());

            self.program
                .gl
                .DrawArrays(opengl::POINTS, 0, num_points as i32);

            self.program.gl.Disable(opengl::PROGRAM_POINT_SIZE);
        }
        num_points
    }
}

pub struct NodeView {
    pub meta: octree::NodeMeta,

    // The buffers are bound by 'vertex_array', so we never refer to them. But they must outlive
    // this 'NodeView'.
    vertex_array: GlVertexArray,
    _buffer_position: GlBuffer,
    _buffer_color: GlBuffer,
    used_memory_bytes: usize,
}

impl NodeView {
    fn new(program: &GlProgram, node_data: octree::NodeData) -> Self {
        unsafe {
            program.gl.UseProgram(program.id);
        }

        let vertex_array = GlVertexArray::new(Rc::clone(&program.gl));
        vertex_array.bind();

        // We draw the points in random order. This allows us to only draw the first N if we want
        // to draw less.
        let mut indices: Vec<usize> = (0..node_data.meta.num_points as usize).collect();
        let mut rng = thread_rng();
        rng.shuffle(&mut indices);

        let position = reshuffle(
            &indices,
            &node_data.position,
            match node_data.meta.position_encoding {
                octree::PositionEncoding::Uint8 => 3,
                octree::PositionEncoding::Uint16 => 6,
                octree::PositionEncoding::Float32 => 12,
            },
        );
        let color = reshuffle(&indices, &node_data.color, 3);

        let buffer_position = GlBuffer::new_array_buffer(Rc::clone(&program.gl));
        let buffer_color = GlBuffer::new_array_buffer(Rc::clone(&program.gl));

        unsafe {
            buffer_position.bind();
            let (normalize, data_type) = match node_data.meta.position_encoding {
                octree::PositionEncoding::Uint8 => (true, opengl::UNSIGNED_BYTE),
                octree::PositionEncoding::Uint16 => (true, opengl::UNSIGNED_SHORT),
                octree::PositionEncoding::Float32 => (false, opengl::FLOAT),
            };
            program.gl.BufferData(
                opengl::ARRAY_BUFFER,
                position.len() as GLsizeiptr,
                &position[0] as *const u8 as *const c_void,
                opengl::STATIC_DRAW,
            );

            // Specify the layout of the vertex data.
            let pos_attr = program.gl.GetAttribLocation(program.id, c_str!("position"));
            program.gl.EnableVertexAttribArray(pos_attr as GLuint);
            program.gl.VertexAttribPointer(
                pos_attr as GLuint,
                3,
                data_type,
                normalize as GLboolean,
                0,
                ptr::null(),
            );

            buffer_color.bind();
            program.gl.BufferData(
                opengl::ARRAY_BUFFER,
                color.len() as GLsizeiptr,
                &color[0] as *const u8 as *const c_void,
                opengl::STATIC_DRAW,
            );
            let color_attr = program.gl.GetAttribLocation(program.id, c_str!("color"));
            program.gl.EnableVertexAttribArray(color_attr as GLuint);
            program.gl.VertexAttribPointer(
                color_attr as GLuint,
                3,
                opengl::UNSIGNED_BYTE,
                opengl::FALSE as GLboolean,
                0,
                ptr::null(),
            );
        }
        NodeView {
            vertex_array,
            _buffer_position: buffer_position,
            _buffer_color: buffer_color,
            meta: node_data.meta,
            used_memory_bytes: position.len() + color.len(),
        }
    }
}

// Keeps track of the nodes that were requested in-order and loads then one by one on request.
pub struct NodeViewContainer {
    node_views: LruCache<octree::NodeId, NodeView>,
    // The node_ids that the I/O thread is currently loading.
    requested: FnvHashSet<octree::NodeId>,
    // Communication with the I/O thread.
    node_id_sender: Sender<octree::NodeId>,
    node_data_receiver: Receiver<(octree::NodeId, octree::NodeData)>,
}

impl NodeViewContainer {
    pub fn new(octree: Arc<Box<octree::Octree>>, max_nodes_in_memory: usize) -> Self {
        // We perform I/O in a separate thread in order to not block the main thread while loading.
        // Data sharing is done through channels.
        let (node_id_sender, node_id_receiver) = mpsc::channel();
        let (node_data_sender, node_data_receiver) = mpsc::channel();
        std::thread::spawn(move || {
            // Loads the next node data in the receiver queue.
            for node_id in node_id_receiver {
                let node_data = octree.get_node_data(&node_id).unwrap();
                // TODO(hrapp): reshuffle
                node_data_sender.send((node_id, node_data)).unwrap();
            }
        });
        NodeViewContainer {
            node_views: LruCache::new(max_nodes_in_memory),
            requested: FnvHashSet::default(),
            node_id_sender: node_id_sender,
            node_data_receiver: node_data_receiver,
        }
    }

    pub fn consume_arrived_nodes(&mut self, program: &GlProgram) -> bool {
        let mut consumed_any = false;
        while let Ok((node_id, node_data)) = self.node_data_receiver.try_recv() {
            // Put loaded node into hash map.
            self.requested.remove(&node_id);
            self.node_views
                .insert(node_id, NodeView::new(program, node_data));
            consumed_any = true;
        }
        consumed_any
    }

    // Returns the 'NodeView' for 'node_id' if it is already loaded, otherwise returns None, but
    // requested the node for loading in the I/O thread
    pub fn get_or_request(&mut self, node_id: &octree::NodeId) -> Option<&NodeView> {
        if self.node_views.contains_key(node_id) {
            return self.node_views.get_mut(node_id).map(|f| f as &NodeView);
        }

        // Limit the number of requested nodes because after a camera move
        // requested nodes might not be in the frustum anymore.
        if !self.requested.contains(node_id) && self.requested.len() < 10 {
            self.requested.insert(*node_id);
            self.node_id_sender.send(*node_id).unwrap();
        }
        None
    }

    pub fn request_all(&mut self, node_ids: &[octree::NodeId]) {
        for &node_id in node_ids {
            if !self.node_views.contains_key(&node_id) && !self.requested.contains(&node_id) {
                self.requested.insert(node_id);
                self.node_id_sender.send(node_id).unwrap();
            }
        }
    }

    pub fn get_used_memory_bytes(&self) -> usize {
        self.node_views
            .iter()
            .map(|(_, node_view)| node_view.used_memory_bytes)
            .sum()
    }
}
