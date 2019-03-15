use crate::codec::{decode, fixpoint_decode};
use crate::color;
use crate::errors::*;
use crate::math::Cube;
use crate::octree::{
    NodeId, NodeLayer, NodeMeta, OctreeDataProvider, OctreeMeta, PositionEncoding,
};
use crate::{InternalIterator, Point};
use byteorder::{LittleEndian, ReadBytesExt};
use cgmath::Vector3;
use num_traits::identities::Zero;
use std::io::{BufReader, Read};

/// Streams points from our data provider representation.
pub struct NodeIterator {
    xyz_reader: BufReader<Box<dyn Read>>,
    rgb_reader: BufReader<Box<dyn Read>>,
    intensity_reader: Option<BufReader<Box<dyn Read>>>,
    meta: NodeMeta,
}

impl NodeIterator {
    pub fn from_data_provider(
        octree_data_provider: &dyn OctreeDataProvider,
        octree_meta: &OctreeMeta,
        id: &NodeId,
        num_points: i64,
    ) -> Result<Self> {
        let bounding_cube = id.find_bounding_cube(&Cube::bounding(&octree_meta.bounding_box));
        let position_encoding = PositionEncoding::new(&bounding_cube, octree_meta.resolution);
        let intensity_reader = match octree_data_provider.data(id, vec![NodeLayer::Intensity]) {
            Ok(mut data_map) => match data_map.remove(&NodeLayer::Intensity) {
                Some(intensity_data) => Some(BufReader::new(intensity_data)),
                None => {
                    return Err("No intensity reader available.".into());
                }
            },
            Err(_) => None,
        };

        let mut position_color_reads =
            octree_data_provider.data(id, vec![NodeLayer::Position, NodeLayer::Color])?;
        Ok(NodeIterator {
            xyz_reader: BufReader::new(
                position_color_reads
                    .remove(&NodeLayer::Position)
                    .ok_or_else(|| "No position reader available.")?,
            ),
            rgb_reader: BufReader::new(
                position_color_reads
                    .remove(&NodeLayer::Color)
                    .ok_or_else(|| "No color reader available.")?,
            ),
            intensity_reader,
            meta: NodeMeta {
                bounding_cube,
                position_encoding,
                num_points,
            },
        })
    }
}

impl InternalIterator for NodeIterator {
    fn size_hint(&self) -> Option<usize> {
        Some(self.meta.num_points as usize)
    }

    fn for_each<F: FnMut(&Point)>(mut self, mut f: F) {
        let mut point = Point {
            position: Vector3::zero(),
            color: color::RED.to_u8(), // is overwritten
            intensity: None,
        };

        let edge_length = self.meta.bounding_cube.edge_length();
        let min = self.meta.bounding_cube.min();
        for _ in 0..self.meta.num_points {
            // I tried pulling out this match by taking a function pointer to a 'decode_position'
            // function. This replaces a branch per point vs a function call per point and turned
            // out to be marginally slower.
            match self.meta.position_encoding {
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
            }

            point.color.red = self.rgb_reader.read_u8().unwrap();
            point.color.green = self.rgb_reader.read_u8().unwrap();
            point.color.blue = self.rgb_reader.read_u8().unwrap();
            if let Some(ir) = self.intensity_reader.as_mut() {
                point.intensity = Some(ir.read_f32::<LittleEndian>().unwrap());
            }
            f(&point);
        }
    }
}
