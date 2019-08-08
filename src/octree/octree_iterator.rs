use crate::errors::*;
use crate::math::{Cube, PointCulling};
use crate::octree::{ChildIndex, DataProvider, NodeId, Octree, OctreeMeta, PositionEncoding};
use crate::read_write::{NodeIterator, RawNodeReader};
use crate::Point;
use cgmath::{EuclideanSpace, Point3};
use std::collections::{HashMap, VecDeque};

impl NodeIterator<RawNodeReader> {
    pub fn from_data_provider(
        octree_data_provider: &dyn DataProvider,
        octree_meta: &OctreeMeta,
        id: &NodeId,
        num_points: usize,
    ) -> Result<Self> {
        if num_points == 0 {
            return Ok(NodeIterator::default());
        }

        let bounding_cube = id.find_bounding_cube(&Cube::bounding(&octree_meta.bounding_box));
        let position_encoding = PositionEncoding::new(&bounding_cube, octree_meta.resolution);

        let mut attributes = HashMap::new();

        let mut position_color_reads =
            octree_data_provider.data(&id.to_string(), &["position", "color"])?;
        match position_color_reads.remove("position") {
            Some(position_data) => {
                attributes.insert("position".to_string(), position_data);
            }
            None => return Err("No position reader available.".into()),
        }
        match position_color_reads.remove("color") {
            Some(color_data) => {
                attributes.insert("color".to_string(), color_data);
            }
            None => return Err("No color reader available.".into()),
        }

        if let Ok(mut data_map) = octree_data_provider.data(&id.to_string(), &["intensity"]) {
            match data_map.remove("intensity") {
                Some(intensity_data) => {
                    attributes.insert("intensity".to_string(), intensity_data);
                }
                None => return Err("No intensity reader available.".into()),
            }
        };

        Ok(Self::new(
            RawNodeReader::new(attributes, position_encoding, bounding_cube)?,
            num_points,
        ))
    }
}

/// returns an Iterator over the points of the current node
fn get_node_iterator(octree: &Octree, node_id: &NodeId) -> NodeIterator<RawNodeReader> {
    // TODO(sirver): This crashes on error. We should bubble up an error.
    NodeIterator::from_data_provider(
        &*octree.data_provider,
        &octree.meta,
        &node_id,
        octree.nodes[&node_id].num_points as usize,
    )
    .expect("Could not read node points")
}

/// iterator over the points of a octree node that satisfy the condition expressed by a boolean function
pub struct FilteredPointsIterator {
    culling: Box<dyn PointCulling<f64>>,
    node_iterator: NodeIterator<RawNodeReader>,
}

impl FilteredPointsIterator {
    pub fn new(
        octree: &Octree,
        node_id: NodeId,
        culling: Box<dyn PointCulling<f64>>,
    ) -> FilteredPointsIterator {
        FilteredPointsIterator {
            culling,
            node_iterator: get_node_iterator(octree, &node_id),
        }
    }
}

impl Iterator for FilteredPointsIterator {
    type Item = Point;

    fn next(&mut self) -> Option<Point> {
        while let Some(point) = self.node_iterator.next() {
            if self.culling.contains(&Point3::from_vec(point.position)) {
                return Some(point);
            }
        }
        None
    }
}

pub struct NodeIdsIterator<'a, F> {
    octree: &'a Octree,
    filter_func: F,
    node_ids: VecDeque<NodeId>,
}

impl<'a, F> NodeIdsIterator<'a, F>
where
    F: Fn(&NodeId, &Octree) -> bool,
{
    pub fn new(octree: &'a Octree, filter_func: F) -> NodeIdsIterator<'a, F> {
        NodeIdsIterator {
            octree,
            node_ids: vec![NodeId::from_level_index(0, 0)].into(),
            filter_func,
        }
    }
}

impl<'a, F> Iterator for NodeIdsIterator<'a, F>
where
    F: Fn(&NodeId, &'a Octree) -> bool,
{
    type Item = NodeId;

    fn next(&mut self) -> Option<NodeId> {
        while let Some(current) = self.node_ids.pop_front() {
            if (self.filter_func)(&current, &self.octree) {
                for child_index in 0..8 {
                    let child_id = current.get_child_id(ChildIndex::from_u8(child_index));
                    if self.octree.nodes.contains_key(&child_id) {
                        self.node_ids.push_back(child_id);
                    }
                }
                return Some(current);
            }
        }
        None
    }
}
