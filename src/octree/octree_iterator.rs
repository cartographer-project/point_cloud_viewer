use crate::math::Cube;

use crate::octree::{ChildIndex, Node, NodeId, NodeIterator, Octree};
use crate::Point;
use cgmath::{Matrix4, Point3, Vector4};
use collision::Aabb3;
use std::collections::VecDeque;

/// returns an Iterator over the points of the current node
pub fn get_node_iterator(octree: &Octree, node_id: &NodeId) -> NodeIterator {
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
    node_ids: VecDeque<NodeId>,
    node_iterator: NodeIterator,
}

impl<'a> FilteredPointsIterator<'a> {
    pub fn new(
        octree: &'a Octree,
        node_ids: VecDeque<NodeId>,
        filter_func: Box<Fn(&Point) -> bool + 'a>,
    ) -> Self {
        FilteredPointsIterator {
            octree,
            filter_func,
            node_ids,
            node_iterator: NodeIterator::Empty,
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
            self.node_iterator = match self.node_ids.pop_front() {
                Some(node_id) => get_node_iterator(self.octree, &node_id),
                None => return None,
            };
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

// TODO(ksavinash9) update after https://github.com/rustgd/collision-rs/issues/101 is resolved.
pub fn contains(projection_matrix: &Matrix4<f64>, point: &Point3<f64>) -> bool {
    let v = Vector4::new(point.x, point.y, point.z, 1.);
    let clip_v = projection_matrix * v;
    clip_v.x.abs() < clip_v.w && clip_v.y.abs() < clip_v.w && 0. < clip_v.z && clip_v.z < clip_v.w
}

pub fn intersecting_node_ids(
    octree: &Octree,
    intersects: &Fn(&Aabb3<f64>) -> bool,
) -> VecDeque<NodeId> {
    let mut node_ids = VecDeque::new();
    let mut open_list: VecDeque<Node> = vec![Node::root_with_bounding_cube(Cube::bounding(
        &octree.meta.bounding_box,
    ))]
    .into();
    while !open_list.is_empty() {
        let current = open_list.pop_front().unwrap();
        if !intersects(&current.bounding_cube.to_aabb3()) {
            continue;
        }
        node_ids.push_back(current.id);
        for child_index in 0..8 {
            let child = current.get_child(ChildIndex::from_u8(child_index));
            if octree.nodes.contains_key(&child.id) {
                open_list.push_back(child);
            }
        }
    }
    node_ids
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
