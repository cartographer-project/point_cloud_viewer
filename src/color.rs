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

use num_traits::Float;
use std::iter::Sum;
use std::ops::{Add, AddAssign, Div};

// Entries follow GL semantics: they are in [0.; 1.] with 1. being fully saturated.
#[derive(Debug, Default, Copy, Clone, PartialEq)]
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
    pub fn to_f32(self) -> Color<f32> {
        Color {
            red: f32::from(self.red) / 255.,
            green: f32::from(self.green) / 255.,
            blue: f32::from(self.blue) / 255.,
            alpha: f32::from(self.alpha) / 255.,
        }
    }
}

impl<T: Add<Output = T>> Add for Color<T>
where
    T: Float,
{
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            red: self.red + other.red,
            green: self.green + other.green,
            blue: self.blue + other.blue,
            alpha: self.alpha + other.alpha,
        }
    }
}

impl<T> AddAssign for Color<T>
where
    T: Float,
{
    fn add_assign(&mut self, other: Self) {
        *self = *self + other;
    }
}

impl<T> Sum<Color<T>> for Color<T>
where
    T: Float + Default,
{
    fn sum<I>(iter: I) -> Self
    where
        I: Iterator<Item = Self>,
    {
        iter.fold(Color::default(), |acc, x| acc + x)
    }
}

impl<T> Div<T> for Color<T>
where
    T: Float,
{
    type Output = Self;

    fn div(self, rhs: T) -> Self::Output {
        Self {
            red: self.red / rhs,
            green: self.green / rhs,
            blue: self.blue / rhs,
            alpha: self.alpha / rhs,
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
