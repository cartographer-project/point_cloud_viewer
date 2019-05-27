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

/// iterator over the points of the octree that satisfy the condition expressed by a boolean function
pub struct FilteredPointsIterator<'a> {
    octree: &'a Octree,
    filter_func: Box<Fn(&Point) -> bool + 'a>,
    node_id: NodeId,
    node_iterator: NodeIterator,
}

impl<'a> FilteredPointsIterator<'a> {
    pub fn new(
        octree: &'a Octree,
        node_id: NodeId,
        filter_func: Box<Fn(&Point) -> bool + 'a>,
    ) -> Self {
        FilteredPointsIterator {
            octree,
            filter_func,
            node_id,
            node_iterator: get_node_iterator(octree, &node_id),
        }
    }
}

impl<'a> Iterator for FilteredPointsIterator<'a> {
    type Item = Point;

    fn next(&mut self) -> Option<Point> {
        loop {
            while let Some(point) = self.node_iterator.next() {
                if (self.filter_func)(&point) {
                    return Some(point);
                }
            }
        }
    }
}
///iterator for all points in an octree
pub struct AllPointsIterator<'a> {
    octree: &'a Octree,
    node_iterator: NodeIterator,
    open_list: VecDeque<NodeId>,
}

impl<'a> AllPointsIterator<'a> {
    pub fn new(octree: &'a Octree) -> Self {
        AllPointsIterator {
            octree,
            node_iterator: NodeIterator::Empty,
            open_list: vec![NodeId::from_level_index(0, 0)].into(),
        }
    }
}

impl<'a> Iterator for AllPointsIterator<'a> {
    type Item = Point;

    fn next(&mut self) -> Option<Point> {
        loop {
            if let Some(point) = self.node_iterator.next() {
                return Some(point);
            }
            match self.open_list.pop_front() {
                Some(current) => {
                    for child_index in 0..8 {
                        let child_id = current.get_child_id(ChildIndex::from_u8(child_index));
                        if self.octree.nodes.contains_key(&child_id) {
                            self.open_list.push_back(child_id);
                        }
                    }
                    self.node_iterator = get_node_iterator(self.octree, &current);
                }
                None => return None,
            }
        }
    }
}

pub struct NodeIdIterator<'a> {
    octree: &'a Octree,
    filter_func: Box<Fn(&NodeId, &Octree) -> bool + 'a>,
    node_ids: VecDeque<NodeId>,
}

impl<'a> NodeIdIterator<'a> {
    pub fn new(octree: &'a Octree, filter_func: Box<Fn(&NodeId, &Octree) -> bool + 'a>) -> Self {
        NodeIdIterator {
            octree,
            node_ids: vec![NodeId::from_level_index(0, 0)].into(),
            filter_func,
        }
    }
}

impl<'a> Iterator for NodeIdIterator<'a> {
    type Item = NodeId;

    fn next(&mut self) -> Option<NodeId> {
        loop {
            match self.node_ids.pop_front() {
                Some(current) => {
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
                None => return None,
            }
        }
    }
}
