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

use cgmath::{Decomposed, Deg, InnerSpace, Matrix4, One, PerspectiveFov, Quaternion, Rad, Rotation,
             Rotation3, Transform, Vector3, Zero};
use opengl;
use std::f32;

#[derive(Debug)]
pub struct Camera {
    pub moving_backward: bool,
    pub moving_forward: bool,
    pub moving_left: bool,
    pub moving_right: bool,
    pub moving_down: bool,
    pub moving_up: bool,
    pub turning_left: bool,
    pub turning_right: bool,
    pub turning_down: bool,
    pub turning_up: bool,
    pub width: i32,
    pub height: i32,

    movement_speed: f32,
    theta: Rad<f32>,
    phi: Rad<f32>,
    pan: Vector3<f32>,
    delta_theta: Rad<f32>,
    delta_phi: Rad<f32>,

    moved: bool,
    transform: Decomposed<Vector3<f32>, Quaternion<f32>>,

    projection_matrix: Matrix4<f32>,
}

#[derive(Debug,Serialize,Deserialize,Clone,Copy)]
pub struct State {
    transform: Decomposed<Vector3<f32>, Quaternion<f32>>,
    phi: Rad<f32>,
    theta: Rad<f32>,
}

impl Camera {
    pub fn new(gl: &opengl::Gl, width: i32, height: i32) -> Self {
        let mut camera = Camera {
            movement_speed: 1.5,
            moving_backward: false,
            moving_forward: false,
            moving_left: false,
            moving_right: false,
            moving_down: false,
            moving_up: false,
            turning_left: false,
            turning_right: false,
            turning_down: false,
            turning_up: false,
            moved: true,
            theta: Rad::zero(),
            phi: Rad::zero(),
            pan: Vector3::zero(),
            delta_theta: Rad::zero(),
            delta_phi: Rad::zero(),
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
        camera.set_size(gl, width, height);
        camera
    }

    pub fn state(&self) -> State  {
        State {
            transform: self.transform,
            phi: self.phi,
            theta: self.theta
        }
    }

    pub fn set_state(&mut self, state: State) {
        self.transform = state.transform;
        self.phi = state.phi;
        self.theta = state.theta;
        self.moved = true;
    }

    pub fn set_size(&mut self, gl: &opengl::Gl, width: i32, height: i32) {
        self.width = width;
        self.height = height;
        self.projection_matrix = Matrix4::from(PerspectiveFov {
            fovy: Rad::from(Deg(45.)),
            aspect: width as f32 / height as f32,
            near: 0.1,
            far: 10000.0,
        });
        unsafe {
            gl.Viewport(0, 0, width, height);
        }
        self.moved = true;
    }

    pub fn get_world_to_gl(&self) -> Matrix4<f32> {
        let world_to_camera: Matrix4<f32> = self.transform.inverse_transform().unwrap().into();
        self.projection_matrix * world_to_camera
    }

    /// Update the camera position for the current frame. Returns true if the camera moved in this
    /// step.
    pub fn update(&mut self) -> bool {
        let mut moved = self.moved;
        self.moved = false;

        // Handle keyboard input
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
            self.pan += pan.normalize();
        }

        const TURNING_SPEED : Rad<f32> = Rad(0.015);
        if self.turning_left {
            self.delta_theta += TURNING_SPEED;
        }
        if self.turning_right {
            self.delta_theta -= TURNING_SPEED;
        }
        if self.turning_up {
            self.delta_phi += TURNING_SPEED;
        }
        if self.turning_down {
            self.delta_phi -= TURNING_SPEED;
        }

        // Apply changes
        if self.pan.magnitude2() > 0. {
            moved = true;
            let translation = self.transform
                .rot
                .rotate_vector(self.pan * self.movement_speed);
            self.transform.disp += translation;
        }

        if !self.delta_theta.is_zero() || !self.delta_phi.is_zero() {
            moved = true;
            self.theta += self.delta_theta;
            self.phi += self.delta_phi;
            let rotation_z = Quaternion::from_angle_z(self.theta);
            let rotation_x = Quaternion::from_angle_x(self.phi);
            self.transform.rot = rotation_z * rotation_x;
        }

        self.pan = Vector3::zero();
        self.delta_theta = Rad::zero();
        self.delta_phi = Rad::zero();
        moved
    }

    pub fn mouse_drag_pan(&mut self, delta_x: i32, delta_y: i32) {
        self.pan.x -= 100. * delta_x as f32 / self.width as f32;
        self.pan.y += 100. * delta_y as f32 / self.height as f32;
    }

    pub fn mouse_drag_rotate(&mut self, delta_x: i32, delta_y: i32) {
        self.delta_theta -= Rad(2. * f32::consts::PI * delta_x as f32 / self.width as f32);
        self.delta_phi -= Rad(2. * f32::consts::PI * delta_y as f32 / self.height as f32);
    }

    pub fn mouse_wheel(&mut self, delta: i32) {
        let sign = delta.signum() as f32;
        self.movement_speed += sign * 0.1 * self.movement_speed;
        self.movement_speed = self.movement_speed.max(0.01);
    }

    pub fn pan(&mut self, x: f32, y: f32, z: f32) {
        self.pan.x += x * self.movement_speed;
        self.pan.y += y * self.movement_speed;
        self.pan.z += z * self.movement_speed;
    }

    pub fn rotate(&mut self, up: f32, around: f32) {
        self.delta_phi += Rad(up * self.movement_speed);
        self.delta_theta += Rad(around * self.movement_speed);
    }
}
