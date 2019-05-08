use crate::errors::*;
use crate::{BatchIterator, Point, PointData, PointStream};
use crate::octree::{AllPointsIterator, FilteredPointsIterator};



const NUM_POINTS_PER_BATCH: usize = 500_000;

/// trait that extends the std iterator to operate on point batches




impl<'a> BatchIterator for FilteredPointsIterator<'a>{
    type BatchContainer = PointData;

    fn try_for_each_batch<F>(&mut self, func: F) -> Result<()>
    {
        let mut point_stream = PointStream::new(self.num_points_per_batch, &mut func);
         // get a batch of data
        let (size, _) = self.size_hint();
        for 0..size
        {
            match self.next(){
              Some(point) ->  point_stream.push_point(point);
              None -> continue; 
            }
        } 
        point_stream.callback();
    }
}

impl<'a> BatchIterator for AllPointsIterator<'a>{
    type BatchIterator = PointData;

    fn try_for_each_batch<F>(&mut self, func: F) -> Result<()>
    {
        let mut point_stream = PointStream::new(self.num_points_per_batch, &mut func);
         // get a batch of data
         let n = 0;
        while (self.empty())
        {
            match self.next(){
              Some(point) -> point_stream.push_point(point);
              None -> continue; 
            }
            n++;
            if(n>= NUM_POINTS_PER_BATCH)
            {
                point_stream.callback();
            }
        } 
        point_stream.callback();
    }
}

 