use crate::errors::*;
use crate::{AllPointsIterator, FilteredPointsIterator}
const NUM_POINTS_PER_BATCH: usize = 500_000;

/// trait that extends the std iterator to operate on point batches
pub trait BatchIterator: Iterator{
    type BatchContainer;

    fn try_for_each<F>(&mut self, f: F) -> Result<()>
    where
       F: FnMut(Self::BatchContainer) -> Result<()>
}

impl BatchIterator for AllPointsIterator{
    type BatchIterator = PointData;

    fn try_for_each<F>(&mut self, f: F) -> Result<()>
    {
        
    }


}

// TODO(mfeuerstein): Move layer declarations and functionality to the open source repo.
pub enum LayerData {
    F32(Vec<f32>),
    F64Vec3(Vec<Vector3<f64>>),
    U8Vec4(Vec<Vector4<u8>>),
}

impl LayerData {
    pub fn len(&self) -> usize {
        match self {
            LayerData::F32(data) => data.len(),
            LayerData::F64Vec3(data) => data.len(),
            LayerData::U8Vec4(data) => data.len(),
        }
    }
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    pub fn dim(&self) -> usize {
        match self {
            LayerData::F32(_) => 1,
            LayerData::F64Vec3(_) => 3,
            LayerData::U8Vec4(_) => 4,
        }
    }
}

pub struct PointData {
    pub position: Vec<Vector3<f64>>,
    pub layers: FnvHashMap<String, LayerData>,
}

struct PointStream<'a, F> {
    position: Vec<Vector3<f64>>,
    color: Vec<Vector4<u8>>,
    intensity: Vec<f32>,
    func: &'a mut F,
}

impl<'a, F> PointStream<'a, F>
where
    F: FnMut(PointData) -> Result<()>,
{
    fn new(num_points_per_batch: usize, func: &'a mut F) -> Self {
        PointStream {
            position: Vec::with_capacity(num_points_per_batch),
            color: Vec::with_capacity(num_points_per_batch),
            intensity: Vec::with_capacity(num_points_per_batch),
            func,
        }
    }

    fn push_point(&mut self, point: Point) {
        self.position.push(point.position);
        self.color.push(Vector4::new(
            point.color.red,
            point.color.green,
            point.color.blue,
            point.color.alpha,
        ));
        if let Some(point_intensity) = point.intensity {
            self.intensity.push(point_intensity);
        };
    }

    fn callback(&mut self) -> Result<()> {
        if self.position.is_empty() {
            return Ok(());
        }

        let mut layers = FnvHashMap::default();
        layers.insert(
            "color".to_string(),
            LayerData::U8Vec4(self.color.split_off(0)),
        );
        if !self.intensity.is_empty() {
            layers.insert(
                "intensity".to_string(),
                LayerData::F32(self.intensity.split_off(0)),
            );
        }
        let point_data = PointData {
            position: self.position.split_off(0),
            layers,
        };
        (self.func)(point_data)
    }

    fn push_point_and_callback(&mut self, point: Point) -> Result<()> {
        self.push_point(point);
        if self.position.len() == self.position.capacity() {
            return self.callback();
        }
        Ok(())
    }
}
