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

// Entries follow GL semantics: they are in [0.; 1.] with 1. being fully saturated.
#[derive(Debug, Copy, Clone)]
pub struct Color<T> {
    pub red: T,
    pub green: T,
    pub blue: T,
    pub alpha: T,
}

impl Color<f32> {
    pub fn to_u8(&self) -> Color<u8> {
        Color {
            red: (self.red * 255.) as u8,
            green: (self.green * 255.) as u8,
            blue: (self.blue * 255.) as u8,
            alpha: (self.alpha * 255.) as u8,
        }
    }
}

impl Color<u8> {
    pub fn to_f32(&self) -> Color<f32> {
        Color {
            red: (self.red as f32) / 255.,
            green: (self.green as f32) / 255.,
            blue: (self.blue as f32) / 255.,
            alpha: (self.alpha as f32) / 255.,
        }
    }
}

pub const RED: Color<f32> = Color {
    red: 1.,
    green: 0.,
    blue: 0.,
    alpha: 1.,
};
pub const GREEN: Color<f32> = Color {
    red: 0.,
    green: 1.,
    blue: 0.,
    alpha: 1.,
};
pub const BLUE: Color<f32> = Color {
    red: 0.,
    green: 0.,
    blue: 1.,
    alpha: 1.,
};
pub const YELLOW: Color<f32> = Color {
    red: 1.,
    green: 1.,
    blue: 0.,
    alpha: 1.,
};
pub const CYAN: Color<f32> = Color {
    red: 0.,
    green: 1.,
    blue: 1.,
    alpha: 1.,
};
pub const MAGENTA: Color<f32> = Color {
    red: 1.,
    green: 0.,
    blue: 1.,
    alpha: 1.,
};
pub const WHITE: Color<f32> = Color {
    red: 1.,
    green: 1.,
    blue: 1.,
    alpha: 1.,
};
pub const TRANSPARENT: Color<f32> = Color {
    red: 1.,
    green: 1.,
    blue: 1.,
    alpha: 0.,
};
