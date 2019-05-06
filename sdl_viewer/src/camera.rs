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
use cgmath::{
    Decomposed, Deg, InnerSpace, Matrix3, Matrix4, One, PerspectiveFov, Quaternion, Rad, Rotation,
    Rotation3, Transform, Vector3, Zero,
};
use serde_derive::{Deserialize, Serialize};
use std::f64;
use time;

#[derive(Debug)]
struct RotationAngle {
    theta: Rad<f64>,
    phi: Rad<f64>,
}

impl RotationAngle {
    pub fn zero() -> Self {
        RotationAngle {
            theta: Rad::zero(),
            phi: Rad::zero(),
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
    pan: Vector3<f64>,
    // The speed we currently want to rotate at. This is multiplied with the seconds since the last
    // frame to get to an absolute rotation.
    rotation_speed: RotationAngle,

    // An absolute value that we should rotate around. This is used when the user is clicking and
    // dragging with the mouse, at which point we want to follow the mouse and ignore rotation
    // speed from the Joystick.
    delta_rotation: RotationAngle,

    moved: bool,
    transform: Decomposed<Vector3<f64>, Quaternion<f64>>,

    projection_matrix: Matrix4<f64>,
}

pub enum CameraPose {
    AlignToGravity,
    Init,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
pub struct State {
    transform: Decomposed<Vector3<f64>, Quaternion<f64>>,
    phi: Rad<f64>,
    theta: Rad<f64>,
}

const FAR_PLANE: f32 = 10000.;
const NEAR_PLANE: f32 = 0.1;

impl Camera {
    pub fn new(gl: &opengl::Gl, width: i32, height: i32) -> Self {
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

            pan: Vector3::zero(),
            rotation_speed: RotationAngle::zero(),
            delta_rotation: RotationAngle::zero(),
            transform: Decomposed {
                scale: 1.,
                rot: Quaternion::one(),
                disp: Vector3::new(0., 0., 150.),
            },

            // These will be set by set_size().
            projection_matrix: One::one(),
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

    /// sets the camera 50 m higher than the point, looking to the earth center
    /// it alignes the up side with north
    /// East/west equations (local latitude vector)
    /// E [x1 x2 0] east orthonormal to ECEF Z axis
    /// p1*x1+p2*x2 = 0 E ortho U
    /// x1^2 + x2^2 = 1
    ///
    /// Compute North vector by using cross product,  UxE = N
    /// and take the solution that makes z component of north (x5) >0
    pub fn set_displacement(&mut self, earth_vector: Vector3<f64>, setting: CameraPose) {
        let magnitude = earth_vector.magnitude();
        let hover_height = match setting {
            CameraPose::Init => 150.0,
            CameraPose::AlignToGravity => 0.0,
        };

        self.transform.disp = earth_vector.normalize_to(magnitude + hover_height);
        let earth_normal = earth_vector.normalize();
        let p1 = earth_normal.x;
        let p2 = earth_normal.y;

        // compute east-west vector (local latitude)
        // y_lat_component = p1/(p1squared + p2squared).sqrt();
        let squared_sum = (p1 * p1 + p2 * p2).sqrt();
        let lat_vector = Vector3::new(-p2 / squared_sum, p1 / squared_sum, 0.0);

        // compute north vector (local longitude direction) as cross product
        let up_north = Vector3::cross(earth_normal, lat_vector);
        //self.transform = Transform::look_at(eye, , up_north);
        let rotation: Matrix3<f64> = Matrix3::from_cols(lat_vector, up_north, earth_normal);
        self.transform.rot = Quaternion::from(rotation);

        self.moved = true;
    }

    pub fn align_with_gravity(&mut self) {
        self.set_displacement(self.transform.disp, CameraPose::AlignToGravity);
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
            phi: Rad(0.0),
            theta: Rad(0.0), //removing redundant parts? backward compatibility?
        }
    }

    pub fn set_state(&mut self, state: State) {
        self.transform = state.transform;
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

        self.projection_matrix = Matrix4::from(PerspectiveFov {
            fovy: Rad::from(Deg(45.)),
            aspect: f64::from(self.width) / f64::from(self.height),
            near: f64::from(near),
            far: f64::from(far),
        });
        unsafe {
            gl.Viewport(0, 0, self.width, self.height);
        }
        self.moved = true;
    }

    pub fn toggle_ct_mode(&mut self, gl: &opengl::Gl) {
        self.ct_mode.enabled = !self.ct_mode.enabled;
        self.update_viewport(gl);
    }

    pub fn get_world_to_gl(&mut self) -> Matrix4<f64> {
        let world_to_camera: Matrix4<f64> = self.transform.inverse_transform().unwrap().into();
        self.projection_matrix.cast::<f64>().unwrap() * world_to_camera
    }

    /// Update the camera position for the current frame. Returns true if the camera moved in this
    /// step.
    pub fn update(&mut self, elapsed: time::Duration) -> bool {
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

        let elapsed_seconds = elapsed.num_milliseconds() as f64 / 1000.;

        const TURNING_SPEED: Rad<f64> = Rad(0.15);
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
        if self.pan.magnitude2() > 0. {
            moved = true;
            let translation = self
                .transform
                .rot
                .rotate_vector(self.pan * self.movement_speed * elapsed_seconds);
            self.transform.disp += translation;
        }

        let mut theta = Rad(0.0);
        let mut phi = Rad(0.0);
        if !self.rotation_speed.theta.is_zero()
            || !self.rotation_speed.phi.is_zero()
            || !self.delta_rotation.theta.is_zero()
            || !self.delta_rotation.phi.is_zero()
        {
            moved = true;
            if !self.delta_rotation.theta.is_zero() || !self.delta_rotation.phi.is_zero() {
                theta += self.delta_rotation.theta;
                phi += self.delta_rotation.phi;
            } else {
                theta += self.rotation_speed.theta * elapsed_seconds;
                phi += self.rotation_speed.phi * elapsed_seconds;
            }
            let rotation_z = Quaternion::from_angle_z(theta);
            let rotation_x = Quaternion::from_angle_x(phi);
            self.transform.rot = rotation_z * rotation_x * self.transform.rot;
        }

        self.pan = Vector3::zero();
        self.rotation_speed.theta = Rad::zero();
        self.rotation_speed.phi = Rad::zero();
        self.delta_rotation.theta = Rad::zero();
        self.delta_rotation.phi = Rad::zero();
        moved
    }

    pub fn mouse_drag_pan(&mut self, delta_x: i32, delta_y: i32) {
        self.pan.x -= 100. * f64::from(delta_x) / f64::from(self.width);
        self.pan.y += 100. * f64::from(delta_y) / f64::from(self.height);
    }

    pub fn mouse_drag_rotate(&mut self, delta_x: i32, delta_y: i32) {
        self.delta_rotation.theta -=
            Rad(2. * f64::consts::PI * f64::from(delta_x) / f64::from(self.width));
        self.delta_rotation.phi -=
            Rad(2. * f64::consts::PI * f64::from(delta_y) / f64::from(self.height));
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
        self.rotation_speed.phi += Rad(up);
        self.rotation_speed.theta += Rad(around);
    }
}
