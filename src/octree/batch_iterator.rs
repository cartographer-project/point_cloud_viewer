use crate::{LayerData, Point, PointData};
use crate::errors::*;
use crate::math::OrientedBeam;
use crate::octree::{self, Octree};
use cgmath::{Matrix4, Vector3, Vector4};
use collision::Aabb3;
use fnv::FnvHashMap;


/// size for batch
pub const NUM_POINTS_PER_BATCH: usize = 500_000;

/// enum to disambiguate the kind of iterator needed in batch
#[allow(clippy::large_enum_variant)]
pub enum PointLocation {
    Any(),
    Aabb(Aabb3<f64>),
    Frustum(Matrix4<f64>),
    OrientedBeam(OrientedBeam),
}

/// current implementation of the stream of points used in batch
struct PointStream<'a, F> 
where
    F: FnMut(PointData) -> Result<()>,
{
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

    /// push point in batch
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

    /// execute function on batch of points
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


struct BatchIterator<'a>{
    octree: &'a Octree,
    iterator: Box<Iterator<Item = Point> + 'a>,
    batch_size : usize
}


impl<'a> BatchIterator<'a>{
    pub fn new(octree: &'a octree::Octree, location: &'a octree::batch_iterator::PointLocation, size: usize) -> Self
    {
        BatchIterator{
            octree : octree,
            iterator: match location {
                PointLocation::Any() => Box::new(octree.all_points()),
                PointLocation::Aabb(aabb) => Box::new(octree.points_in_box(aabb)),
                PointLocation::Frustum(frustum) => Box::new(octree.points_in_frustum(frustum)),
                PointLocation::OrientedBeam(beam) => Box::new(octree.points_in_oriented_beam(beam)),
            },
        batch_size : size,
        }
        
        
    }


    pub fn try_for_each_batch<F>(&mut self, mut func: F) -> Result<()>
    where
    F: FnMut(PointData) -> Result<()>,
    {
        let mut point_stream = PointStream::new(NUM_POINTS_PER_BATCH, &mut func);
        'octree_loop: loop {
            let mut n = 0;
            'batch : while n < NUM_POINTS_PER_BATCH {
                match self.iterator.next(){
                  Some(point) =>  {n+=1; point_stream.push_point(point);},
                  None => { break 'octree_loop;}
                }
            }
            //call at every batch, return if error
            match point_stream.callback(){
                Ok(()) => continue,
                Err(e) => return Err(e),
            }
        } 
        //call on the last batch
        point_stream.callback()
    }
}

#[cfg(test)]
#[test]
