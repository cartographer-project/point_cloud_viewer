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
