// Copyright 2016 The Cartographer Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use crate::geometry::Cube;
use crate::proto;
use crate::read_write::PositionEncoding;
use nalgebra::Point3;
use std::num::ParseIntError;
use std::str::FromStr;
use std::{fmt, result};

/// Represents a child of an octree Node.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub struct ChildIndex(u8);

impl ChildIndex {
    pub fn from_u8(index: u8) -> Self {
        assert!(index < 8);
        ChildIndex(index)
    }

    /// Returns the ChildId of the child containing 'v'.
    pub fn from_bounding_cube(bounding_cube: &Cube, v: &Point3<f64>) -> ChildIndex {
        // This is a bit flawed: it is not guaranteed that 'child_bounding_box.contains(&v)' is true
        // using this calculated index due to floating point precision.
        let center = bounding_cube.center();
        let gt_x = v.x > center.x;
        let gt_y = v.y > center.y;
        let gt_z = v.z > center.z;
        ChildIndex((gt_x as u8) << 2 | (gt_y as u8) << 1 | gt_z as u8)
    }

    pub fn as_u8(self) -> u8 {
        self.0
    }
}

/// A unique identifier to a node. Currently this is implemented as 'r' being the root and r[0-7]
/// being the children, r[0-7][0-7] being the grand children and so on. The actual representation
/// might change though.
// Top 8 bits of the value are level, the rest is the index.
// The root has level = 0, its children 1 and so on. Multiple nodes can have the same index,
// but none can have the same index and level.
#[derive(Debug, Hash, Clone, Copy, PartialEq, Eq)]
pub struct NodeId(u128);

impl FromStr for NodeId {
    type Err = ParseIntError;

    /// Construct a NodeId. No checking is done if this is a valid Id.
    fn from_str(name: &str) -> std::result::Result<Self, Self::Err> {
        let level = (name.len() - 1) as u8;
        let index = if level > 0 {
            u128::from_str_radix(&name[1..], 8)?
        } else {
            0
        };
        Ok(NodeId::from_level_index(level, index))
    }
}

impl fmt::Display for NodeId {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> result::Result<(), fmt::Error> {
        if self.level() == 0 {
            "r".fmt(formatter)
        } else {
            write!(
                formatter,
                "r{index:0width$o}",
                index = self.index(),
                width = self.level() as usize
            )
        }
    }
}

impl NodeId {
    pub fn from_proto(proto: &proto::NodeId) -> Self {
        let deprecated_level = proto.deprecated_level as u8;
        let deprecated_index = proto.deprecated_index;
        let high = proto.high;
        let low = proto.low;
        if deprecated_level != 0 || deprecated_index != 0 {
            NodeId::from_level_index(deprecated_level, deprecated_index as u128)
        } else {
            NodeId((u128::from(high) << 64) | u128::from(low))
        }
    }

    pub fn to_proto(&self) -> proto::NodeId {
        let mut proto = proto::NodeId::new();
        proto.set_high((self.0 >> 64) as u64);
        proto.set_low(self.0 as u64);
        proto
    }

    pub fn from_level_index(level: u8, index: u128) -> Self {
        let value = (u128::from(level) << 120) | index;
        NodeId(value)
    }

    /// Returns the root node of the octree.
    fn root() -> Self {
        NodeId(0)
    }

    /// Returns the NodeId for the corresponding 'child_index'.
    #[inline]
    pub fn get_child_id(&self, child_index: ChildIndex) -> Self {
        NodeId::from_level_index(
            self.level() + 1,
            (self.index() << 3) + u128::from(child_index.0),
        )
    }

    /// The child index of this node in its parent.
    fn child_index(&self) -> Option<ChildIndex> {
        if self.level() == 0 {
            return None;
        }
        Some(ChildIndex(self.index() as u8 & 7))
    }

    /// Returns the parents id or None if this is the root.
    pub fn parent_id(&self) -> Option<NodeId> {
        if self.level() == 0 {
            return None;
        }
        Some(NodeId::from_level_index(
            self.level() - 1,
            self.index() >> 3,
        ))
    }

    /// Returns the level of this node in the octree, with 0 being the root.
    pub fn level(&self) -> u8 {
        (self.0 >> 120) as u8
    }

    /// Returns the index of this node at the current level.
    pub fn index(&self) -> u128 {
        self.0 & 0x00ff_ffff_ffff_ffff_ffff_ffff_ffff_ffff
    }

    /// Computes the bounding cube from a NodeID.
    pub fn find_bounding_cube(&self, root_bounding_cube: &Cube) -> Cube {
        let mut edge_length = root_bounding_cube.edge_length();
        let mut min = root_bounding_cube.min();
        for level in (0..self.level()).rev() {
            edge_length /= 2.;
            // Reverse order: process from root to leaf nodes.
            let child_index = (self.0 >> (3 * level)) & 7;
            let z = child_index & 1;
            let y = (child_index >> 1) & 1;
            let x = (child_index >> 2) & 1;
            min.x += x as f64 * edge_length;
            min.y += y as f64 * edge_length;
            min.z += z as f64 * edge_length;
        }
        Cube::new(min, edge_length)
    }
}

#[derive(Debug)]
pub struct Node {
    pub id: NodeId,
    pub bounding_cube: Cube,
}

impl Node {
    pub fn root_with_bounding_cube(cube: Cube) -> Self {
        Node {
            id: NodeId::root(),
            bounding_cube: cube,
        }
    }

    #[inline]
    pub fn get_child(&self, child_index: ChildIndex) -> Node {
        let child_bounding_cube = {
            let half_edge_length = self.bounding_cube.edge_length() / 2.;
            let mut min = self.bounding_cube.min();
            if (child_index.0 & 0b001) != 0 {
                min.z += half_edge_length;
            }

            if (child_index.0 & 0b010) != 0 {
                min.y += half_edge_length;
            }

            if (child_index.0 & 0b100) != 0 {
                min.x += half_edge_length;
            }
            Cube::new(min, half_edge_length)
        };
        Node {
            id: self.id.get_child_id(child_index),
            bounding_cube: child_bounding_cube,
        }
    }

    // TODO(hrapp): This function could use some testing.
    pub fn parent(&self) -> Option<Node> {
        let maybe_parent_id = self.id.parent_id();
        maybe_parent_id?;

        let parent_cube = {
            let child_index = self.id.child_index().unwrap().0;
            let mut min = self.bounding_cube.min();
            let edge_length = self.bounding_cube.edge_length();
            if (child_index & 0b001) != 0 {
                min.z -= edge_length;
            }

            if (child_index & 0b010) != 0 {
                min.y -= edge_length;
            }

            if (child_index & 0b100) != 0 {
                min.x -= edge_length;
            }
            Cube::new(min, edge_length * 2.)
        };
        Some(Node {
            id: maybe_parent_id.unwrap(),
            bounding_cube: parent_cube,
        })
    }

    /// Returns the level of this node in the octree, with 0 being the root.
    pub fn level(&self) -> u8 {
        self.id.level()
    }
}

#[derive(Clone, Debug)]
pub struct NodeMeta {
    pub num_points: i64,
    pub position_encoding: PositionEncoding,
    pub bounding_cube: Cube,
}

impl NodeMeta {
    pub fn num_points_for_level_of_detail(&self, level_of_detail: i32) -> i64 {
        (self.num_points as f32 / level_of_detail as f32).ceil() as i64
    }
}

pub fn to_node_proto(
    node_id: &NodeId,
    num_points: i64,
    position_encoding: &PositionEncoding,
) -> proto::OctreeNode {
    let mut proto = proto::OctreeNode::new();
    *proto.mut_id() = node_id.to_proto();
    proto.set_num_points(num_points);
    proto.set_position_encoding(position_encoding.to_proto());
    proto
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn test_parent_node_name() {
        assert_eq!(
            Some(NodeId::from_str("r12345").unwrap()),
            NodeId::from_str("r123456").unwrap().parent_id()
        );
    }

    #[test]
    fn test_child_index() {
        assert_eq!(
            Some(ChildIndex(1)),
            NodeId::from_str("r123451").unwrap().child_index()
        );
        assert_eq!(
            Some(ChildIndex(7)),
            NodeId::from_str("r123457").unwrap().child_index()
        );
        assert_eq!(None, NodeId::from_str("r").unwrap().child_index());
    }

    #[test]
    fn test_bounding_box() {
        let root_bounding_cube = Cube::new(Point3::new(-5., -5., -5.), 10.);

        let bounding_cube = NodeId::from_str("r0")
            .unwrap()
            .find_bounding_cube(&root_bounding_cube);
        assert_eq!(-5., bounding_cube.min().x);
        assert_eq!(-5., bounding_cube.min().y);
        assert_eq!(-5., bounding_cube.min().z);
        assert_eq!(5., bounding_cube.edge_length());

        let bounding_cube = NodeId::from_str("r13")
            .unwrap()
            .find_bounding_cube(&root_bounding_cube);
        assert_eq!(-5., bounding_cube.min().x);
        assert_eq!(-2.5, bounding_cube.min().y);
        assert_eq!(2.5, bounding_cube.min().z);
        assert_eq!(2.5, bounding_cube.edge_length());
    }
}
