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

extern crate cgmath;

use cgmath::{Point2, Vector2};
use std::fmt::{self, Write};

#[derive(Debug, Clone)]
pub struct Rect {
    min: Point2<f32>,
    edge_length: f32,
}

impl Rect {
    pub fn new(min: Point2<f32>, edge_length: f32) -> Self {
        Rect {
            min: min,
            edge_length: edge_length,
        }
    }

    pub fn edge_length(&self) -> f32 {
        self.edge_length
    }

    pub fn min(&self) -> Point2<f32> {
        self.min
    }

    pub fn max(&self) -> Point2<f32> {
        Point2::new(self.min.x + self.edge_length, self.min.y + self.edge_length)
    }

    /// The center of the box.
    pub fn center(&self) -> Vector2<f32> {
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

    pub fn get_child(&self, child_index: ChildIndex) -> Node {
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
        let maybe_parent_id = self.id.parent_id();
        if maybe_parent_id.is_none() {
            return None;
        }

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
            id: maybe_parent_id.unwrap(),
            bounding_rect: parent_rect,
        })
    }

    /// Returns the level of this node in the quadtree, with 0 being the root.
    pub fn level(&self) -> u8 {
        self.id.level()
    }
}

/// Represents a child of an quadree Node.
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

    /// Construct a NodeId. No checking is done if this is a valid Id.
    pub fn from_str(name: &str) -> Self {
        let level = (name.len() - 1) as u8;
        let index = if level > 0 {
            u64::from_str_radix(&name[1..], 4).unwrap()
        } else {
            0
        };
        NodeId { level, index }
    }

    /// Returns the root node.
    pub fn root() -> Self {
        NodeId { index: 0, level: 0 }
    }

    /// Returns the NodeId for the corresponding 'child_index'.
    pub fn get_child_id(&self, child_index: ChildIndex) -> Self {
        NodeId {
            level: self.level + 1,
            index: (self.index << 2) + child_index.0 as u64,
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

impl fmt::Display for NodeId {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_char('r')?;
        let index = self.index;
        for level in (0..self.level).rev() {
            let c = match (index >> 2 * level) & 3 {
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

#[cfg(test)]
mod tests {
    use super::{ChildIndex, NodeId};

    #[test]
    fn test_parent_node_name() {
        assert_eq!(
            Some(NodeId::from_str("r12321")),
            NodeId::from_str("r123210").parent_id()
        );
    }

    #[test]
    fn test_child_index() {
        assert_eq!(
            Some(ChildIndex(1)),
            NodeId::from_str("r123321").child_index()
        );
        assert_eq!(
            Some(ChildIndex(3)),
            NodeId::from_str("r123323").child_index()
        );
        assert_eq!(None, NodeId::from_str("r").child_index());
    }

    #[test]
    fn test_to_string() {
        for id in &["r", "r0", "r123323"] {
            assert_eq!(&NodeId::from_str(id).to_string(), id);
        }
    }
}
