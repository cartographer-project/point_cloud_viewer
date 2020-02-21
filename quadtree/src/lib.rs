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

use cgmath::{Point2, Vector2};
use std::fmt::{self, Write};
use std::num::ParseIntError;
use std::str::FromStr;

#[derive(Debug, Clone)]
pub struct Rect {
    min: Point2<f64>,
    edge_length: f64,
}

impl Rect {
    pub fn new(min: Point2<f64>, edge_length: f64) -> Self {
        Rect { min, edge_length }
    }

    pub fn edge_length(&self) -> f64 {
        self.edge_length
    }

    pub fn min(&self) -> Point2<f64> {
        self.min
    }

    pub fn max(&self) -> Point2<f64> {
        Point2::new(self.min.x + self.edge_length, self.min.y + self.edge_length)
    }

    /// The center of the box.
    pub fn center(&self) -> Vector2<f64> {
        let min = self.min();
        let max = self.max();
        Vector2::new((min.x + max.x) / 2., (min.y + max.y) / 2.)
    }
}

#[derive(Debug)]
pub struct Node {
    pub id: NodeId,
    pub bounding_rect: Rect,
}

impl Node {
    pub fn root_with_bounding_rect(rect: Rect) -> Self {
        Node {
            id: NodeId::root(),
            bounding_rect: rect,
        }
    }

    pub fn get_child(&self, child_index: &ChildIndex) -> Node {
        let child_bounding_rect = {
            let half_edge_length = self.bounding_rect.edge_length() / 2.;
            let mut min = self.bounding_rect.min();
            if (child_index.0 & 0b01) != 0 {
                min.y += half_edge_length;
            }

            if (child_index.0 & 0b10) != 0 {
                min.x += half_edge_length;
            }
            Rect::new(min, half_edge_length)
        };
        Node {
            id: self.id.get_child_id(child_index),
            bounding_rect: child_bounding_rect,
        }
    }

    // TODO(hrapp): This function could use some testing.
    pub fn parent(&self) -> Option<Node> {
        let maybe_parent_id = self.id.parent_id()?;

        let parent_rect = {
            let child_index = self.id.child_index().unwrap().0;
            let mut min = self.bounding_rect.min();
            let edge_length = self.bounding_rect.edge_length();
            if (child_index & 0b01) != 0 {
                min.y -= edge_length;
            }

            if (child_index & 0b10) != 0 {
                min.x -= edge_length;
            }
            Rect::new(min, edge_length * 2.)
        };
        Some(Node {
            id: maybe_parent_id,
            bounding_rect: parent_rect,
        })
    }

    /// Returns the level of this node in the quadtree, with 0 being the root.
    pub fn level(&self) -> u8 {
        self.id.level()
    }
}

/// Represents a child of a quadtree Node.
#[derive(Debug, PartialEq, Eq)]
pub struct ChildIndex(u8);

impl ChildIndex {
    pub fn from_u8(index: u8) -> Self {
        assert!(index < 4);
        ChildIndex(index)
    }

    pub fn as_u8(&self) -> u8 {
        self.0
    }
}

#[derive(Debug, Hash, Clone, Copy, PartialEq, Eq)]
pub struct NodeId {
    // The root is level = 0, its children 1 and so on.
    level: u8,
    // The index of this node. Multiple nodes can have the same index, but none can have the same
    // index and level.
    index: u64,
}

impl NodeId {
    pub fn new(level: u8, index: u64) -> Self {
        NodeId { level, index }
    }

    /// Returns the root node.
    pub fn root() -> Self {
        NodeId { index: 0, level: 0 }
    }

    /// Returns the NodeId for the corresponding 'child_index'.
    pub fn get_child_id(&self, child_index: &ChildIndex) -> Self {
        NodeId {
            level: self.level + 1,
            index: (self.index << 2) + u64::from(child_index.0),
        }
    }

    /// The child index of this node in its parent.
    pub fn child_index(&self) -> Option<ChildIndex> {
        if self.level() == 0 {
            return None;
        }
        Some(ChildIndex(self.index as u8 & 3))
    }

    /// Returns the parents id or None if this is the root.
    pub fn parent_id(&self) -> Option<NodeId> {
        if self.level() == 0 {
            return None;
        }
        Some(NodeId {
            level: self.level - 1,
            index: (self.index >> 2),
        })
    }

    /// Returns the level of this node in the quadtree, with 0 being the root.
    pub fn level(&self) -> u8 {
        self.level
    }

    pub fn index(&self) -> u64 {
        self.index
    }
}

impl FromStr for NodeId {
    type Err = ParseIntError;

    /// Construct a NodeId. No checking is done if this is a valid Id.
    fn from_str(name: &str) -> Result<Self, Self::Err> {
        let level = (name.len() - 1) as u8;
        let index = if level > 0 {
            u64::from_str_radix(&name[1..], 4)?
        } else {
            0
        };
        Ok(NodeId { level, index })
    }
}

impl fmt::Display for NodeId {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_char('r')?;
        let index = self.index;
        for level in (0..self.level).rev() {
            let c = match (index >> (2 * level)) & 3 {
                0 => '0',
                1 => '1',
                2 => '2',
                3 => '3',
                _ => unimplemented!(),
            };
            formatter.write_char(c)?;
        }
        Ok(())
    }
}

pub enum Direction {
    Left,
    TopLeft,
    Top,
    TopRight,
    Right,
    BottomRight,
    Bottom,
    BottomLeft,
}

#[derive(Debug, Hash, Clone, Copy, PartialEq, Eq)]
pub struct SpatialNodeId {
    level: u8,
    x: u64,
    y: u64,
}

impl SpatialNodeId {
    pub fn new(level: u8, x: u64, y: u64) -> Self {
        Self { level, x, y }
    }

    pub fn level(&self) -> u8 {
        self.level
    }

    pub fn x(&self) -> u64 {
        self.x
    }

    pub fn y(&self) -> u64 {
        self.y
    }

    pub fn neighbor(&self, direction: Direction) -> Option<Self> {
        let cur_x = self.x as i64;
        let cur_y = self.y as i64;
        let (x, y) = match direction {
            Direction::Left => (cur_x - 1, cur_y),
            Direction::TopLeft => (cur_x - 1, cur_y + 1),
            Direction::Top => (cur_x, cur_y + 1),
            Direction::TopRight => (cur_x + 1, cur_y + 1),
            Direction::Right => (cur_x + 1, cur_y),
            Direction::BottomRight => (cur_x + 1, cur_y - 1),
            Direction::Bottom => (cur_x, cur_y - 1),
            Direction::BottomLeft => (cur_x - 1, cur_y - 1),
        };
        let max_dim = 2i64.pow(self.level as u32);
        if 0 <= x && x < max_dim && 0 <= y && y < max_dim {
            Some(Self::new(self.level, x as u64, y as u64))
        } else {
            None
        }
    }
}

/// See e.g. https://docs.microsoft.com/en-us/bingmaps/articles/bing-maps-tile-system
/// on how to convert between coordinates and the quadkey.
impl From<NodeId> for SpatialNodeId {
    fn from(node_id: NodeId) -> Self {
        let level = node_id.level;
        let mut x = 0;
        let mut y = 0;
        for i in 1..=level {
            let mask = 1 << (level - i);
            let index = node_id.index >> ((level - i) * 2);
            if 0b01 & index != 0 {
                y |= mask;
            }
            if 0b10 & index != 0 {
                x |= mask;
            }
        }
        Self::new(level, x, y)
    }
}

impl From<SpatialNodeId> for NodeId {
    fn from(spatial_node_id: SpatialNodeId) -> Self {
        let level = spatial_node_id.level;
        let mut index = 0;
        for i in 1..=level {
            index <<= 2;
            let mask = 1 << (level - i);
            if (spatial_node_id.y & mask) != 0 {
                index += 0b01;
            }
            if (spatial_node_id.x & mask) != 0 {
                index += 0b10;
            }
        }
        Self::new(level, index)
    }
}

#[cfg(test)]
mod tests {
    use super::{ChildIndex, NodeId, SpatialNodeId};
    use std::str::FromStr;

    #[test]
    fn test_parent_node_name() {
        assert_eq!(
            Some(NodeId::from_str("r12321").unwrap()),
            NodeId::from_str("r123210").unwrap().parent_id()
        );
    }

    #[test]
    fn test_child_index() {
        assert_eq!(
            Some(ChildIndex(1)),
            NodeId::from_str("r123321").unwrap().child_index()
        );
        assert_eq!(
            Some(ChildIndex(3)),
            NodeId::from_str("r123323").unwrap().child_index()
        );
        assert_eq!(None, NodeId::from_str("r").unwrap().child_index());
    }

    #[test]
    fn test_to_string() {
        for id in &["r", "r0", "r123323"] {
            assert_eq!(&NodeId::from_str(id).unwrap().to_string(), id);
        }
    }

    #[test]
    fn test_spatial_node_id_from_node_id() {
        assert_eq!(
            SpatialNodeId::new(3, 4, 5),
            SpatialNodeId::from(NodeId::from_str("r301").unwrap())
        );
    }

    #[test]
    fn test_conversion() {
        for id in &["r", "r0", "r123323"] {
            let node_id = NodeId::from_str(id).unwrap();
            let spatial_node_id = SpatialNodeId::from(node_id);
            assert_eq!(NodeId::from(spatial_node_id), node_id);
        }
    }
}
