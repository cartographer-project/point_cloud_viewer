use crate::math::clamp;

pub fn fixpoint_encode<T>(value: f32, min: f32, edge_length: f32) -> T
where
    T: num_traits::PrimInt + num_traits::Bounded + num_traits::NumCast,
{
    let value =
        clamp((value - min) / edge_length, 0., 1.) * num::cast::<T, f32>(T::max_value()).unwrap();
    num::cast(value).unwrap()
}

pub fn encode(value: f32, min: f32, edge_length: f32) -> f32 {
    clamp((value - min) / edge_length, 0., 1.)
}

pub fn fixpoint_decode<T>(value: T, min: f32, edge_length: f32) -> f32
where
    T: num_traits::PrimInt + num_traits::Bounded + num_traits::NumCast,
{
    let max: f32 = num::cast(T::max_value()).unwrap();
    let v: f32 = num::cast(value).unwrap();
    v / max * edge_length + min
}

pub fn decode(value: f32, min: f32, edge_length: f32) -> f32 {
    value * edge_length + min
}
