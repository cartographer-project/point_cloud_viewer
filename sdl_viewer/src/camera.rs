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

use cgmath::{Angle, Decomposed, Deg, InnerSpace, Matrix4, One, Quaternion, Rad, Rotation,
             Rotation3, Transform, Vector3, Zero};

use gl;
use std::f32;

// Constructs a projection matrix. Math lifted from ThreeJS.
fn make_projection_matrix<A: Into<Rad<f32>>>(
    near: f32,
    far: f32,
    fov: A,
    zoom: f32,
    aspect_ratio: f32,
) -> Matrix4<f32> {
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
        x,
        0.,
        0.,
        0., // Column 0
        0.,
        y,
        0.,
        0., // Column 1
        a,
        b,
        c,
        -1., // Column 2
        0.,
        0.,
        d,
        0., // Column 3
    )
}

#[derive(Debug)]
pub struct Camera {
    pub moving_backward: bool,
    pub moving_forward: bool,
    pub moving_left: bool,
    pub moving_right: bool,
    pub moving_down: bool,
    pub moving_up: bool,
    pub width: i32,
    pub height: i32,

    movement_speed: f32,
    theta: Rad<f32>,
    phi: Rad<f32>,

    moved: bool,
    transform: Decomposed<Vector3<f32>, Quaternion<f32>>,

    projection_matrix: Matrix4<f32>,
}

impl Camera {
    pub fn new(width: i32, height: i32) -> Self {
        let mut camera = Camera {
            movement_speed: 1.5,
            moving_backward: false,
            moving_forward: false,
            moving_left: false,
            moving_right: false,
            moving_down: false,
            moving_up: false,
            moved: true,
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

    pub fn set_size(&mut self, width: i32, height: i32) {
        self.width = width;
        self.height = height;
        self.projection_matrix =
            make_projection_matrix(0.1, 10000., Deg(45.), 1., width as f32 / height as f32);
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
            moved = true;
            let translation = self.transform
                .rot
                .rotate_vector(pan.normalize() * self.movement_speed);
            self.transform.disp += translation;
        }

        let rotation_z = Quaternion::from_angle_z(self.theta);
        let rotation_x = Quaternion::from_angle_x(self.phi);
        self.transform.rot = rotation_z * rotation_x;
        moved
    }

    pub fn mouse_drag(&mut self, delta_x: i32, delta_y: i32) {
        self.moved = true;
        self.theta -= Rad(2. * f32::consts::PI * delta_x as f32 / self.width as f32);
        self.phi -= Rad(2. * f32::consts::PI * delta_y as f32 / self.height as f32);
    }

    pub fn mouse_wheel(&mut self, delta: i32) {
        let sign = delta.signum() as f32;
        self.movement_speed += sign * 0.1 * self.movement_speed;
        self.movement_speed = self.movement_speed.max(0.01);
    }
}
