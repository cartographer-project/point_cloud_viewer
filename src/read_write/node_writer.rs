use crate::math::Cube;
use crate::octree::PositionEncoding;
use crate::read_write::{encode, fixpoint_encode};
use crate::{NodeLayer, Point};
use byteorder::{LittleEndian, WriteBytesExt};
use std::fs::{remove_file, File};
use std::io::{BufWriter, Result, Write};
use std::path::PathBuf;

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

        self.layer_writers[0].write_u8(p.color.red).unwrap();
        self.layer_writers[0].write_u8(p.color.green).unwrap();
        self.layer_writers[0].write_u8(p.color.blue).unwrap();

        // TODO(sirver): This is expensive. It would be preferable if we needn't branch on
        // every point.
        if let Some(intensity) = p.intensity {
            if self.layer_writers.len() < 2 {
                self.layer_writers.push(
                    DataWriter::new(&self.stem.with_extension(NodeLayer::Intensity.extension()))
                        .unwrap(),
                );
            }
            self.layer_writers[1]
                .write_f32::<LittleEndian>(intensity)
                .unwrap();
        }
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
