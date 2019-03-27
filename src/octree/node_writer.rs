use crate::codec::{encode, fixpoint_encode};
use crate::math::Cube;
use crate::octree::{NodeId, NodeLayer, OctreeMeta, OnDiskOctreeDataProvider, PositionEncoding};
use crate::Point;
use byteorder::{LittleEndian, WriteBytesExt};
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::PathBuf;

#[derive(Debug)]
pub struct NodeWriter {
    xyz_writer: BufWriter<File>,
    rgb_writer: BufWriter<File>,
    intensity_writer: Option<BufWriter<File>>,
    bounding_cube: Cube,
    position_encoding: PositionEncoding,
    stem: PathBuf,
    num_written: i64,
}

impl NodeWriter {
    pub fn new(
        octree_data_provider: &OnDiskOctreeDataProvider,
        octree_meta: &OctreeMeta,
        node_id: &NodeId,
    ) -> Self {
        let stem = octree_data_provider.stem(node_id);
        let bounding_cube = node_id.find_bounding_cube(&Cube::bounding(&octree_meta.bounding_box));
        NodeWriter {
            xyz_writer: BufWriter::new(
                File::create(&stem.with_extension(NodeLayer::Position.extension())).unwrap(),
            ),
            rgb_writer: BufWriter::new(
                File::create(&stem.with_extension(NodeLayer::Color.extension())).unwrap(),
            ),
            intensity_writer: None, // Will be created if needed on first point with intensities.
            stem,
            position_encoding: PositionEncoding::new(&bounding_cube, octree_meta.resolution),
            bounding_cube,
            num_written: 0,
        }
    }

    pub fn write(&mut self, p: &Point) {
        // Note that due to floating point rounding errors while calculating bounding boxes, it
        // could be here that 'p' is not quite inside the bounding box of our node.
        let edge_length = self.bounding_cube.edge_length();
        let min = self.bounding_cube.min();
        match self.position_encoding {
            PositionEncoding::Float32 => {
                self.xyz_writer
                    .write_f32::<LittleEndian>(encode(p.position.x, min.x, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_f32::<LittleEndian>(encode(p.position.y, min.y, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_f32::<LittleEndian>(encode(p.position.z, min.z, edge_length))
                    .unwrap();
            }
            PositionEncoding::Float64 => {
                self.xyz_writer
                    .write_f64::<LittleEndian>(encode(p.position.x, min.x, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_f64::<LittleEndian>(encode(p.position.y, min.y, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_f64::<LittleEndian>(encode(p.position.z, min.z, edge_length))
                    .unwrap();
            }
            PositionEncoding::Uint8 => {
                self.xyz_writer
                    .write_u8(fixpoint_encode(p.position.x, min.x, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_u8(fixpoint_encode(p.position.y, min.y, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_u8(fixpoint_encode(p.position.z, min.z, edge_length))
                    .unwrap();
            }
            PositionEncoding::Uint16 => {
                self.xyz_writer
                    .write_u16::<LittleEndian>(fixpoint_encode(p.position.x, min.x, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_u16::<LittleEndian>(fixpoint_encode(p.position.y, min.y, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_u16::<LittleEndian>(fixpoint_encode(p.position.z, min.z, edge_length))
                    .unwrap();
            }
        }

        self.rgb_writer.write_u8(p.color.red).unwrap();
        self.rgb_writer.write_u8(p.color.green).unwrap();
        self.rgb_writer.write_u8(p.color.blue).unwrap();

        // TODO(sirver): This is expensive. It would be preferable if we needn't branch on
        // every point.
        if let Some(intensity) = p.intensity {
            if self.intensity_writer.is_none() {
                self.intensity_writer = Some(BufWriter::new(
                    File::create(&self.stem.with_extension(NodeLayer::Intensity.extension()))
                        .unwrap(),
                ));
            }
            self.intensity_writer
                .as_mut()
                .unwrap()
                .write_f32::<LittleEndian>(intensity)
                .unwrap();
        }

        self.num_written += 1;
    }

    pub fn num_written(&self) -> i64 {
        self.num_written
    }
}
