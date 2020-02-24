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

use crate::opengl;
use nalgebra::{Isometry3, Matrix4, Perspective3, UnitQuaternion, Vector3};
use num_traits::One;
use point_viewer::math::collision::Perspective;
use serde_derive::{Deserialize, Serialize};
use std::f64;

#[derive(Debug)]
struct RotationAngle {
    /// Horizontal angle in radians
    theta: f64,
    /// Vertical angle in radians
    phi: f64,
}

impl RotationAngle {
    pub fn zero() -> Self {
        RotationAngle {
            theta: 0.0,
            phi: 0.0,
        }
    }
}

#[derive(Debug)]
struct CtMode {
    pub enabled: bool,
    near_plane: f32,
    far_plane: f32,
}
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
    ct_mode: CtMode,

    movement_speed: f64,
    theta: f64,
    phi: f64,
    pan: Vector3<f64>,

    // The speed we currently want to rotate at. This is multiplied with the seconds since the last
    // frame to get to an absolute rotation.
    rotation_speed: RotationAngle,

    // An absolute value that we should rotate around. This is used when the user is clicking and
    // dragging with the mouse, at which point we want to follow the mouse and ignore rotation
    // speed from the Joystick.
    delta_rotation: RotationAngle,

    moved: bool,
    transform: Isometry3<f64>,

    projection_matrix: Matrix4<f32>,
    local_from_global: Isometry3<f64>,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct State {
    transform: Isometry3<f64>,
    phi: f64,
    theta: f64,
}

const FAR_PLANE: f32 = 10000.;
const NEAR_PLANE: f32 = 0.1;

impl Camera {
    pub fn new(
        gl: &opengl::Gl,
        width: i32,
        height: i32,
        local_from_global: Option<Isometry3<f64>>,
    ) -> Self {
        let local_from_global = local_from_global.unwrap_or_else(One::one);
        let mut camera = Camera {
            movement_speed: 10.,
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
            theta: 0.0,
            phi: 0.0,
            pan: nalgebra::zero(),
            rotation_speed: RotationAngle::zero(),
            delta_rotation: RotationAngle::zero(),
            transform: Isometry3::translation(0., 0., 150.),
            local_from_global,

            // These will be set by set_size().
            projection_matrix: Matrix4::identity(),
            width: 0,
            height: 0,
            ct_mode: CtMode {
                enabled: false,
                near_plane: 2.,
                far_plane: 5.,
            },
        };
        camera.set_size(gl, width, height);
        camera
    }

    pub fn move_ct(&mut self, delta: f32, gl: &opengl::Gl) {
        if self.ct_mode.near_plane + delta > 0. {
            self.ct_mode.near_plane += delta;
            self.ct_mode.far_plane += delta;
            self.update_viewport(gl);
        }
    }

    pub fn move_far_plane_ct(&mut self, delta: f32, gl: &opengl::Gl) {
        self.ct_mode.far_plane =
            (self.ct_mode.near_plane + 0.5).max(self.ct_mode.far_plane + delta);
        self.update_viewport(gl);
    }

    pub fn state(&self) -> State {
        State {
            transform: self.transform,
            phi: self.phi,
            theta: self.theta,
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
        self.update_viewport(gl);
    }

    pub fn update_viewport(&mut self, gl: &opengl::Gl) {
        let (near, far) = if self.ct_mode.enabled {
            (self.ct_mode.near_plane, self.ct_mode.far_plane)
        } else {
            (NEAR_PLANE, FAR_PLANE)
        };

        self.projection_matrix = Perspective3::new(
            self.width as f32 / self.height as f32,
            std::f32::consts::FRAC_PI_4,
            near,
            far,
        )
        .to_homogeneous();
        unsafe {
            gl.Viewport(0, 0, self.width, self.height);
        }
        self.moved = true;
    }

    pub fn toggle_ct_mode(&mut self, gl: &opengl::Gl) {
        self.ct_mode.enabled = !self.ct_mode.enabled;
        self.update_viewport(gl);
    }

    pub fn get_camera_to_world(&self) -> Isometry3<f64> {
        self.local_from_global.inverse() * self.transform
    }

    pub fn get_world_to_gl(&self) -> Matrix4<f64> {
        let camera_from_global = self.transform.inverse() * self.local_from_global;
        nalgebra::convert::<Matrix4<f32>, Matrix4<f64>>(self.projection_matrix)
            * camera_from_global.to_homogeneous()
    }

    /// Update the camera position for the current frame. Returns true if the camera moved in this
    /// step.
    pub fn update(&mut self, elapsed: time::Duration) -> bool {
        let mut moved = self.moved;
        self.moved = false;

        // Handle keyboard input
        let mut pan: Vector3<f64> = nalgebra::zero();
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
        if pan.norm_squared() > 0. {
            self.pan += pan.normalize();
        }

        let elapsed_seconds = elapsed.as_seconds_f64();

        const TURNING_SPEED: f64 = 0.5;
        if self.turning_left {
            self.rotation_speed.theta += TURNING_SPEED;
        }
        if self.turning_right {
            self.rotation_speed.theta -= TURNING_SPEED;
        }
        if self.turning_up {
            self.rotation_speed.phi += TURNING_SPEED;
        }
        if self.turning_down {
            self.rotation_speed.phi -= TURNING_SPEED;
        }

        // Apply changes
        if self.pan.norm_squared() > 0. {
            moved = true;
            let translation = self
                .transform
                .rotation
                .transform_vector(&(self.pan * self.movement_speed * elapsed_seconds));
            self.transform.append_translation_mut(&translation.into());
        }

        if self.rotation_speed.theta != 0.0
            || self.rotation_speed.phi != 0.0
            || self.delta_rotation.theta != 0.0
            || self.delta_rotation.phi != 0.0
        {
            moved = true;
            if self.delta_rotation.theta != 0.0 || self.delta_rotation.phi != 0.0 {
                self.theta += self.delta_rotation.theta;
                self.phi += self.delta_rotation.phi;
            } else {
                self.theta += self.rotation_speed.theta * elapsed_seconds;
                self.phi += self.rotation_speed.phi * elapsed_seconds;
            }
            let rotation_z = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), self.theta);
            let rotation_x = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), self.phi);
            self.transform.rotation = rotation_z * rotation_x;
        }

        self.pan = nalgebra::zero();
        self.rotation_speed.theta = 0.0;
        self.rotation_speed.phi = 0.0;
        self.delta_rotation.theta = 0.0;
        self.delta_rotation.phi = 0.0;
        moved
    }

    pub fn mouse_drag_pan(&mut self, delta_x: i32, delta_y: i32) {
        self.pan.x -= 100. * f64::from(delta_x) / f64::from(self.width);
        self.pan.y += 100. * f64::from(delta_y) / f64::from(self.height);
    }

    pub fn mouse_drag_rotate(&mut self, delta_x: i32, delta_y: i32) {
        self.delta_rotation.theta -=
            2. * f64::consts::PI * f64::from(delta_x) / f64::from(self.width);
        self.delta_rotation.phi -=
            2. * f64::consts::PI * f64::from(delta_y) / f64::from(self.height);
    }

    pub fn mouse_wheel(&mut self, delta: i32) {
        let sign = f64::from(delta.signum());
        self.movement_speed += sign * 0.1 * self.movement_speed;
        self.movement_speed = self.movement_speed.max(0.01);
    }

    pub fn pan(&mut self, x: f64, y: f64, z: f64) {
        self.pan.x += x;
        self.pan.y += y;
        self.pan.z += z;
    }

    pub fn rotate(&mut self, up: f64, around: f64) {
        self.rotation_speed.phi += up;
        self.rotation_speed.theta += around;
    }
}
