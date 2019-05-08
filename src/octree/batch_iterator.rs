use crate::errors::*;
use crate::{BatchIterator, PointData, PointStream};
use crate::octree::{AllPointsIterator, FilteredPointsIterator};



const NUM_POINTS_PER_BATCH: usize = 500_000;

impl<'a> BatchIterator for FilteredPointsIterator<'a>{
    type BatchContainer = PointData;

    fn try_for_each_batch<F:FnMut<(PointData,)>(&mut self, func: F) -> Result<()>
    {
        let mut point_stream = PointStream::new(NUM_POINTS_PER_BATCH, &mut func);
         // get a batch of data
        let (size, _) = self.size_hint();
        loop {
            let mut n = 0;
            while n < NUM_POINTS_PER_BATCH {
                match self.next(){
                  Some(point) =>  {n+=1; point_stream.push_point(point);},
                  None => break,
                }
            }
            point_stream.callback();
        } 
    }
}

impl<'a> BatchIterator for AllPointsIterator<'a>{
    type BatchContainer = PointData;

    fn try_for_each_batch<F>(&mut self, func: F) -> Result<()>
    {
        let mut point_stream = PointStream::new(NUM_POINTS_PER_BATCH, &mut func);
         // get a batch of data
        let (size, _) = self.size_hint();
        loop {
            let mut n = 0;
            while n < NUM_POINTS_PER_BATCH {
                match self.next(){
                  Some(point) =>  {n+=1; point_stream.push_point(point);},
                  None => break,
                }
            }
            point_stream.callback();
        } 
    }
}

 