use crate::codec::{decode, fixpoint_decode};
use crate::color;
use crate::errors::*;
use crate::math::Cube;
use crate::octree::{NodeLayer, PositionEncoding};
use crate::Point;
use byteorder::{LittleEndian, ReadBytesExt};
use cgmath::Vector3;
use num_traits::identities::Zero;
use std::collections::HashMap;
use std::io::{BufReader, Read};

pub trait NodeReader {
    fn read(&mut self) -> Point;
}

pub struct CubeNodeReader {
    xyz_reader: BufReader<Box<dyn Read>>,
    layer_readers: Vec<BufReader<Box<dyn Read>>>,
    position_encoding: PositionEncoding,
    bounding_cube: Cube,
}

impl NodeReader for CubeNodeReader {
    fn read(&mut self) -> Point {
        let mut point = Point {
            position: Vector3::zero(),
            color: color::RED.to_u8(), // is overwritten
            intensity: None,
        };

        let edge_length = self.bounding_cube.edge_length();
        let min = self.bounding_cube.min();

        // I tried pulling out this match by taking a function pointer to a 'decode_position'
        // function. This replaces a branch per point vs a function call per point and turned
        // out to be marginally slower.
        match self.position_encoding {
            PositionEncoding::Uint8 => {
                point.position.x =
                    fixpoint_decode(self.xyz_reader.read_u8().unwrap(), min.x, edge_length);
                point.position.y =
                    fixpoint_decode(self.xyz_reader.read_u8().unwrap(), min.y, edge_length);
                point.position.z =
                    fixpoint_decode(self.xyz_reader.read_u8().unwrap(), min.z, edge_length);
            }
            PositionEncoding::Uint16 => {
                point.position.x = fixpoint_decode(
                    self.xyz_reader.read_u16::<LittleEndian>().unwrap(),
                    min.x,
                    edge_length,
                );
                point.position.y = fixpoint_decode(
                    self.xyz_reader.read_u16::<LittleEndian>().unwrap(),
                    min.y,
                    edge_length,
                );
                point.position.z = fixpoint_decode(
                    self.xyz_reader.read_u16::<LittleEndian>().unwrap(),
                    min.z,
                    edge_length,
                );
            }
            PositionEncoding::Float32 => {
                point.position.x = decode(
                    self.xyz_reader.read_f32::<LittleEndian>().unwrap(),
                    min.x,
                    edge_length,
                );
                point.position.y = decode(
                    self.xyz_reader.read_f32::<LittleEndian>().unwrap(),
                    min.y,
                    edge_length,
                );
                point.position.z = decode(
                    self.xyz_reader.read_f32::<LittleEndian>().unwrap(),
                    min.z,
                    edge_length,
                );
            }
            PositionEncoding::Float64 => {
                point.position.x = decode(
                    self.xyz_reader.read_f64::<LittleEndian>().unwrap(),
                    min.x,
                    edge_length,
                );
                point.position.y = decode(
                    self.xyz_reader.read_f64::<LittleEndian>().unwrap(),
                    min.y,
                    edge_length,
                );
                point.position.z = decode(
                    self.xyz_reader.read_f64::<LittleEndian>().unwrap(),
                    min.z,
                    edge_length,
                );
            }
        }

        point.color.red = self.layer_readers[0].read_u8().unwrap();
        point.color.green = self.layer_readers[0].read_u8().unwrap();
        point.color.blue = self.layer_readers[0].read_u8().unwrap();

        point.intensity = self
            .layer_readers
            .get_mut(1)
            .map(|ir| ir.read_f32::<LittleEndian>().unwrap());

        point
    }
}

impl CubeNodeReader {
    pub fn new(
        mut layers: HashMap<NodeLayer, Box<dyn Read>>,
        position_encoding: PositionEncoding,
        bounding_cube: Cube,
    ) -> Result<Self> {
        let xyz_reader = BufReader::new(
            layers
                .remove(&NodeLayer::Position)
                .ok_or_else(|| "No position reader available.")?,
        );
        let rgb_reader = BufReader::new(
            layers
                .remove(&NodeLayer::Color)
                .ok_or_else(|| "No color reader available.")?,
        );
        let mut layer_readers = vec![rgb_reader];

        if let Some(intensity_read) = layers.remove(&NodeLayer::Intensity) {
            layer_readers.push(BufReader::new(intensity_read));
        };

        Ok(Self {
            xyz_reader,
            layer_readers,
            position_encoding,
            bounding_cube,
        })
    }
}

/// Streams points from our data provider representation.
pub struct NodeIteratorWithData {
    reader: Box<dyn NodeReader>,
    num_points: usize,
    point_count: usize,
}

pub enum NodeIterator {
    WithData(NodeIteratorWithData),
    Empty,
}

impl NodeIterator {
    pub fn new(reader: Box<dyn NodeReader>, num_points: usize) -> Result<Self> {
        if num_points == 0 {
            return Ok(NodeIterator::Empty);
        }

        let iter = NodeIteratorWithData {
            reader,
            num_points,
            point_count: 0,
        };
        Ok(NodeIterator::WithData(iter))
    }
}

impl Iterator for NodeIterator {
    type Item = Point;

    fn size_hint(&self) -> (usize, Option<usize>) {
        let num_points = match self {
            NodeIterator::WithData(ref iter) => iter.num_points,
            NodeIterator::Empty => 0,
        };
        (num_points, Some(num_points))
    }
    fn next(&mut self) -> Option<Point> {
        let mut iter = match self {
            NodeIterator::WithData(iter) => iter,
            NodeIterator::Empty => return None,
        };

        if iter.point_count == iter.num_points {
            return None;
        }

        let point = iter.reader.read();

        iter.point_count += 1;

        Some(point)
    }
}
