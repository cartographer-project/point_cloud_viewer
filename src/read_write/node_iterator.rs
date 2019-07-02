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
use std::io::{self, BufReader, Read};

pub trait NodeReader {
    fn read(&mut self) -> io::Result<Point>;
    fn num_points(&self) -> usize;
}

pub struct CubeNodeReader {
    xyz_reader: BufReader<Box<dyn Read>>,
    layer_readers: Vec<BufReader<Box<dyn Read>>>,
    num_points: usize,
    position_encoding: PositionEncoding,
    bounding_cube: Cube,
}

impl NodeReader for CubeNodeReader {
    fn read(&mut self) -> io::Result<Point> {
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
                point.position.x = fixpoint_decode(self.xyz_reader.read_u8()?, min.x, edge_length);
                point.position.y = fixpoint_decode(self.xyz_reader.read_u8()?, min.y, edge_length);
                point.position.z = fixpoint_decode(self.xyz_reader.read_u8()?, min.z, edge_length);
            }
            PositionEncoding::Uint16 => {
                point.position.x = fixpoint_decode(
                    self.xyz_reader.read_u16::<LittleEndian>()?,
                    min.x,
                    edge_length,
                );
                point.position.y = fixpoint_decode(
                    self.xyz_reader.read_u16::<LittleEndian>()?,
                    min.y,
                    edge_length,
                );
                point.position.z = fixpoint_decode(
                    self.xyz_reader.read_u16::<LittleEndian>()?,
                    min.z,
                    edge_length,
                );
            }
            PositionEncoding::Float32 => {
                point.position.x = decode(
                    self.xyz_reader.read_f32::<LittleEndian>()?,
                    min.x,
                    edge_length,
                );
                point.position.y = decode(
                    self.xyz_reader.read_f32::<LittleEndian>()?,
                    min.y,
                    edge_length,
                );
                point.position.z = decode(
                    self.xyz_reader.read_f32::<LittleEndian>()?,
                    min.z,
                    edge_length,
                );
            }
            PositionEncoding::Float64 => {
                point.position.x = decode(
                    self.xyz_reader.read_f64::<LittleEndian>()?,
                    min.x,
                    edge_length,
                );
                point.position.y = decode(
                    self.xyz_reader.read_f64::<LittleEndian>()?,
                    min.y,
                    edge_length,
                );
                point.position.z = decode(
                    self.xyz_reader.read_f64::<LittleEndian>()?,
                    min.z,
                    edge_length,
                );
            }
        }

        point.color.red = self.layer_readers[0].read_u8()?;
        point.color.green = self.layer_readers[0].read_u8()?;
        point.color.blue = self.layer_readers[0].read_u8()?;

        if let Some(ir) = self.layer_readers.get_mut(1) {
            point.intensity = Some(ir.read_f32::<LittleEndian>()?);
        }

        Ok(point)
    }

    fn num_points(&self) -> usize {
        self.num_points
    }
}

impl CubeNodeReader {
    pub fn new(
        mut layers: HashMap<NodeLayer, Box<dyn Read>>,
        num_points: usize,
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
            num_points,
            position_encoding,
            bounding_cube,
        })
    }
}

/// Streams points from our data provider representation.
pub enum NodeIterator<R> {
    WithData(R),
    Empty,
}

impl<R> NodeIterator<R>
where
    R: NodeReader,
{
    pub fn new(reader: R) -> Self {
        if reader.num_points() == 0 {
            return NodeIterator::Empty;
        }

        NodeIterator::WithData(reader)
    }
}

impl<R> Iterator for NodeIterator<R>
where
    R: NodeReader,
{
    type Item = Point;

    fn size_hint(&self) -> (usize, Option<usize>) {
        let num_points = match self {
            NodeIterator::WithData(ref reader) => reader.num_points(),
            NodeIterator::Empty => 0,
        };
        (num_points, Some(num_points))
    }
    fn next(&mut self) -> Option<Point> {
        let reader = match self {
            NodeIterator::WithData(reader) => reader,
            NodeIterator::Empty => return None,
        };
        reader.read().ok()
    }
}
