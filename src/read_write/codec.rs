// Copyright 2016 Google Inc.
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

use num::clamp;

pub fn fixpoint_encode<T>(value: f64, min: f64, edge_length: f64) -> T
where
    T: num_traits::PrimInt + num_traits::Bounded + num_traits::NumCast,
{
    let value =
        clamp((value - min) / edge_length, 0., 1.) * num::cast::<T, f64>(T::max_value()).unwrap();
    num::cast(value).unwrap()
}

pub fn encode<T>(value: f64, min: f64, edge_length: f64) -> T
where
    T: num_traits::NumCast,
{
    num::cast(clamp((value - min) / edge_length, 0., 1.)).unwrap()
}

pub fn fixpoint_decode<T>(value: T, min: f64, edge_length: f64) -> f64
where
    T: num_traits::PrimInt + num_traits::Bounded + num_traits::NumCast,
{
    let max: f64 = num::cast(T::max_value()).unwrap();
    let v: f64 = num::cast(value).unwrap();
    v / max * edge_length + min
}

pub fn decode<T>(value: T, min: f64, edge_length: f64) -> f64
where
    T: num_traits::NumCast,
{
    num::cast::<T, f64>(value).unwrap() * edge_length + min
}
