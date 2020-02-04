use crate::attribute_extension;
use crate::data_provider::DataProvider;
use crate::errors::*;
use crate::proto;
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{Cursor, Read};
use std::path::PathBuf;

pub struct OnDiskDataProvider {
    pub directory: PathBuf,
}

impl OnDiskDataProvider {
    /// Returns the path on disk where the data for this node is saved.
    pub fn stem(&self, node_id: &str) -> PathBuf {
        self.directory.join(node_id)
    }

    // Get number of points from the file size of the color data.
    // Color data is required and always present.
    pub fn number_of_points(&self, node_id: &str) -> Result<i64> {
        let stem = self.stem(node_id);
        let file_meta_data_opt = fs::metadata(stem.with_extension(attribute_extension("color")));
        if file_meta_data_opt.is_err() {
            return Err(ErrorKind::NodeNotFound.into());
        }

        let file_size_bytes = file_meta_data_opt.unwrap().len();
        // color has 3 bytes per point
        Ok((file_size_bytes / 3) as i64)
    }
}

impl DataProvider for OnDiskDataProvider {
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
        node_id: &str,
        node_attributes: &[&str],
    ) -> Result<HashMap<String, Box<dyn Read + Send>>> {
        let stem = self.stem(node_id);
        let mut readers = HashMap::<String, Box<dyn Read + Send>>::new();
        for node_attribute in node_attributes {
            let file = match File::open(&stem.with_extension(attribute_extension(node_attribute))) {
                Err(ref err) if err.kind() == ::std::io::ErrorKind::NotFound => {
                    return Err(ErrorKind::NodeNotFound.into());
                }
                e => e,
            }?;
            readers.insert((*node_attribute).to_string(), Box::new(file));
        }
        Ok(readers)
    }
}
