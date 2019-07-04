use crate::errors::*;
use crate::octree::{NodeId, Octree, OctreeDataProvider};
use crate::{proto, NodeLayer};
use protobuf;
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{Cursor, Read};
use std::path::PathBuf;

pub struct OnDiskOctreeDataProvider {
    pub directory: PathBuf,
}

impl OnDiskOctreeDataProvider {
    /// Returns the path on disk where the data for this node is saved.
    pub fn stem(&self, node_id: &NodeId) -> PathBuf {
        self.directory.join(node_id.to_string())
    }

    // Get number of points from the file size of the color data.
    // Color data is required and always present.
    pub fn number_of_points(&self, node_id: &NodeId) -> Result<i64> {
        let stem = self.stem(node_id);
        let file_meta_data_opt = fs::metadata(stem.with_extension(NodeLayer::Color.extension()));
        if file_meta_data_opt.is_err() {
            return Err(ErrorKind::NodeNotFound.into());
        }

        let file_size_bytes = file_meta_data_opt.unwrap().len();
        // color has 3 bytes per point
        Ok((file_size_bytes / 3) as i64)
    }
}

impl OctreeDataProvider for OnDiskOctreeDataProvider {
    fn meta_proto(&self) -> Result<proto::Meta> {
        // We used to use JSON earlier.
        if self.directory.join("meta.json").exists() {
            return Err(ErrorKind::InvalidVersion(3).into());
        }

        let mut data = Vec::new();
        File::open(&self.directory.join("meta.pb"))?.read_to_end(&mut data)?;
        Ok(
            protobuf::parse_from_reader::<proto::Meta>(&mut Cursor::new(data))
                .chain_err(|| "Could not parse meta.pb")?,
        )
    }

    fn data(
        &self,
        node_id: &NodeId,
        node_layers: Vec<NodeLayer>,
    ) -> Result<HashMap<NodeLayer, Box<dyn Read>>> {
        let stem = self.stem(node_id);
        let mut readers = HashMap::<NodeLayer, Box<dyn Read>>::new();
        for node_layer in node_layers {
            let file = match File::open(&stem.with_extension(node_layer.extension())) {
                Err(ref err) if err.kind() == ::std::io::ErrorKind::NotFound => {
                    return Err(ErrorKind::NodeNotFound.into());
                }
                e => e,
            }?;
            readers.insert(node_layer.to_owned(), Box::new(file));
        }
        Ok(readers)
    }
}

//  TODO(catevita): refactor function for octree factory
pub fn octree_from_directory(directory: impl Into<PathBuf>) -> Result<Box<Octree>> {
    let data_provider = OnDiskOctreeDataProvider {
        directory: directory.into(),
    };
    let octree = Octree::from_data_provider(Box::new(data_provider))?;
    Ok(Box::new(octree))
}
