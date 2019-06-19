use crate::octree::{ChildIndex, NodeId, NodeIterator, Octree};
use crate::Point;
use std::collections::VecDeque;

/// returns an Iterator over the points of the current node
fn get_node_iterator(octree: &Octree, node_id: &NodeId) -> NodeIterator {
    // TODO(sirver): This crashes on error. We should bubble up an error.
    NodeIterator::from_data_provider(
        &*octree.data_provider,
        &octree.meta,
        &node_id,
        octree.nodes[&node_id].num_points,
    )
    .expect("Could not read node points")
}

/// iterator over the points of a octree node that satisfy the condition expressed by a boolean function
pub struct FilteredPointsIterator<F> {
    filter_func: F,
    node_iterator: NodeIterator,
}

impl<F> FilteredPointsIterator<F>
where
    F: Fn(&Point) -> bool,
{
    pub fn new(octree: &Octree, node_id: NodeId, filter_func: F) -> FilteredPointsIterator<F> {
        FilteredPointsIterator {
            filter_func,
            node_iterator: get_node_iterator(octree, &node_id),
        }
    }
}

impl<F> Iterator for FilteredPointsIterator<F>
where
    F: Fn(&Point) -> bool,
{
    type Item = Point;

    fn next(&mut self) -> Option<Point> {
        while let Some(point) = self.node_iterator.next() {
            if (self.filter_func)(&point) {
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
