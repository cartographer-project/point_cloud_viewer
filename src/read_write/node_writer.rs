// TODO(feuerste): Remove, once we use all writers;
#![allow(dead_code)]

use crate::color::Color;
use crate::math::Cube;
use crate::octree::PositionEncoding;
use crate::read_write::{encode, fixpoint_encode};
use crate::{NodeLayer, Point};
use byteorder::{LittleEndian, WriteBytesExt};
use cgmath::Vector3;
use std::fs::{remove_file, File};
use std::io::{BufWriter, Result, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

pub struct DataWriter {
    inner: BufWriter<File>,
    bytes_written: usize,
    path: PathBuf,
}

impl DataWriter {
    fn new(path: impl Into<PathBuf>) -> Result<Self> {
        let path = path.into();
        File::create(&path).map(|w| DataWriter {
            inner: BufWriter::new(w),
            bytes_written: 0,
            path,
        })
    }

    fn bytes_written(&self) -> usize {
        self.bytes_written
    }
}

impl Write for DataWriter {
    fn write(&mut self, buf: &[u8]) -> Result<usize> {
        let res = self.inner.write(buf);
        if let Ok(size) = res {
            self.bytes_written += size;
        }
        res
    }

    fn flush(&mut self) -> Result<()> {
        self.inner.flush()
    }
}

impl Seek for DataWriter {
    fn seek(&mut self, pos: SeekFrom) -> Result<u64> {
        self.inner.seek(pos)
    }
}

impl Drop for DataWriter {
    fn drop(&mut self) {
        // If we did not write anything into this node, it should not exist.
        if self.bytes_written == 0 {
            // We are ignoring deletion errors here in case the file is already gone.
            let _ = remove_file(&self.path);
        }

        // TODO(hrapp): Add some sanity checks that we do not have nodes with ridiculously low
        // amount of points laying around?
    }
}

trait WriteLE {
    fn write_le(self, writer: &mut DataWriter);
}

impl WriteLE for f32 {
    fn write_le(self, writer: &mut DataWriter) {
        writer.write_f32::<LittleEndian>(self).unwrap()
    }
}

impl WriteLE for Vector3<f64> {
    fn write_le(self, writer: &mut DataWriter) {
        writer.write_f64::<LittleEndian>(self.x).unwrap();
        writer.write_f64::<LittleEndian>(self.y).unwrap();
        writer.write_f64::<LittleEndian>(self.z).unwrap();
    }
}

impl WriteLE for Color<u8> {
    fn write_le(self, writer: &mut DataWriter) {
        writer.write_u8(self.red).unwrap();
        writer.write_u8(self.green).unwrap();
        writer.write_u8(self.blue).unwrap();
    }
}

fn ensure_intensity_layer(layer_writers: &mut Vec<DataWriter>, stem: &Path) {
    if layer_writers.len() < 2 {
        layer_writers
            .push(DataWriter::new(&stem.with_extension(NodeLayer::Intensity.extension())).unwrap());
    }
}

pub trait NodeWriter {
    fn write(&mut self, p: &Point);
}

pub struct CubeNodeWriter {
    xyz_writer: DataWriter,
    layer_writers: Vec<DataWriter>,
    stem: PathBuf,
    position_encoding: PositionEncoding,
    bounding_cube: Cube,
}

impl NodeWriter for CubeNodeWriter {
    fn write(&mut self, p: &Point) {
        // Note that due to floating point rounding errors while calculating bounding boxes, it
        // could be here that 'p' is not quite inside the bounding box of our node.
        let edge_length = self.bounding_cube.edge_length();
        let min = self.bounding_cube.min();
        match self.position_encoding {
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
        }

        p.color.write_le(&mut self.layer_writers[0]);
        if let Some(i) = p.intensity {
            ensure_intensity_layer(&mut self.layer_writers, &self.stem);
            i.write_le(&mut self.layer_writers[1]);
        };
    }
}

impl CubeNodeWriter {
    pub fn new(
        path: impl Into<PathBuf>,
        position_encoding: PositionEncoding,
        bounding_cube: Cube,
    ) -> Self {
        let stem: PathBuf = path.into();
        let xyz_writer =
            DataWriter::new(&stem.with_extension(NodeLayer::Position.extension())).unwrap();
        let layer_writers =
            vec![DataWriter::new(&stem.with_extension(NodeLayer::Color.extension())).unwrap()];
        Self {
            xyz_writer,
            layer_writers,
            stem,
            position_encoding,
            bounding_cube,
        }
    }

    pub fn num_written(&self) -> i64 {
        self.layer_writers[0].bytes_written() as i64 / 3
    }
}

pub struct RawNodeWriter {
    xyz_writer: DataWriter,
    layer_writers: Vec<DataWriter>,
    stem: PathBuf,
}

impl NodeWriter for RawNodeWriter {
    fn write(&mut self, p: &Point) {
        p.position.write_le(&mut self.xyz_writer);
        p.color.write_le(&mut self.layer_writers[0]);
        if let Some(i) = p.intensity {
            ensure_intensity_layer(&mut self.layer_writers, &self.stem);
            i.write_le(&mut self.layer_writers[1]);
        };
    }
}

impl RawNodeWriter {
    pub fn new(path: impl Into<PathBuf>) -> Self {
        let stem: PathBuf = path.into();
        let xyz_writer =
            DataWriter::new(&stem.with_extension(NodeLayer::Position.extension())).unwrap();
        let layer_writers =
            vec![DataWriter::new(&stem.with_extension(NodeLayer::Color.extension())).unwrap()];
        Self {
            xyz_writer,
            layer_writers,
            stem,
        }
    }
}

pub struct PlyNodeWriter {
    writer: DataWriter,
    point_count: usize,
}

impl NodeWriter for PlyNodeWriter {
    fn write(&mut self, p: &Point) {
        if self.point_count == 0 {
            let mut layers = vec![NodeLayer::Position, NodeLayer::Color];
            if p.intensity.is_some() {
                layers.push(NodeLayer::Intensity);
            }
            self.create_header(&layers);
        }

        p.position.write_le(&mut self.writer);
        p.color.write_le(&mut self.writer);
        if let Some(i) = p.intensity {
            i.write_le(&mut self.writer);
        };

        self.point_count += 1;
    }
}

impl Drop for PlyNodeWriter {
    fn drop(&mut self) {
        self.writer.write_all(b"\n").unwrap();
        if self.writer.seek(SeekFrom::Start(4 + 32 + 15)).is_ok() {
            let _res = write!(&mut self.writer, "{:020}", self.point_count);
        }
    }
}

impl PlyNodeWriter {
    pub fn new(filename: impl AsRef<Path>) -> Self {
        let writer = DataWriter::new(filename.as_ref()).unwrap();
        Self {
            writer,
            point_count: 0,
        }
    }

    fn create_header(&mut self, layers: &[NodeLayer]) {
        self.writer.write_all(b"ply\n").unwrap();
        self.writer
            .write_all(b"format binary_little_endian 1.0\n")
            .unwrap();
        self.writer
            .write_all(b"element vertex 00000000000000000000\n")
            .unwrap();
        for layer in layers {
            match layer {
                NodeLayer::Position => {
                    self.writer.write_all(b"property double x\n").unwrap();
                    self.writer.write_all(b"property double y\n").unwrap();
                    self.writer.write_all(b"property double z\n").unwrap();
                }
                NodeLayer::Color => {
                    self.writer.write_all(b"property uchar red\n").unwrap();
                    self.writer.write_all(b"property uchar green\n").unwrap();
                    self.writer.write_all(b"property uchar blue\n").unwrap();
                }
                NodeLayer::Intensity => {
                    self.writer
                        .write_all(b"property float intensity\n")
                        .unwrap();
                }
            }
        }
        self.writer.write_all(b"end_header\n").unwrap();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::read_write::PlyIterator;
    use tempdir::TempDir;

    #[test]
    fn test_ply_read_write() {
        let tmp_dir = TempDir::new("test_ply_read_write").unwrap();
        let file_path_test = tmp_dir.path().join("out.ply");
        let file_path_gt = "src/test_data/xyz_f32_rgb_u8_intensity_f32.ply";
        {
            let mut ply_writer = PlyNodeWriter::new(&file_path_test);
            PlyIterator::from_file(file_path_gt).unwrap().for_each(|p| {
                ply_writer.write(&p);
            });
        }
        let ply_gt = PlyIterator::from_file(file_path_gt).unwrap();
        let ply_test = PlyIterator::from_file(&file_path_test).unwrap();
        ply_gt.zip(ply_test).for_each(|(gt, test)| {
            assert_eq!(gt.position, test.position);
            assert_eq!(gt.color, test.color);
            // All intensities in this file are NaN, but set.
            assert_eq!(gt.intensity.is_some(), test.intensity.is_some());
        });
    }
}
