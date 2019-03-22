use crate::backend_error::PointsViewerError;
use point_viewer::octree;
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, RwLock};

/// path information for the octrees
#[derive(Clone)]
pub struct OctreeKeyParams {
    /// Location prefix
    prefix: PathBuf,
    /// Location suffix
    suffix: PathBuf,
}

impl OctreeKeyParams {
    pub fn get_octree_address(&self, octree_key: impl AsRef<Path>) -> PathBuf {
        let mut addr = PathBuf::new();
        addr.push(self.prefix.clone());
        addr.push(octree_key.as_ref());
        addr.push(self.suffix.clone());
        addr
    }
}

#[derive(Clone)]
pub struct AppState {
    /// Hash Map for Octrees
    pub octree_map: Arc<RwLock<HashMap<String, Arc<octree::Octree>>>>,
    /// information for retieving octree path
    pub key_params: OctreeKeyParams,
    /// backward compatibility to input arguments
    pub init_octree_id: String,
    /// octree factory to create octrees
    octree_factory: octree::OctreeFactory,
}

impl AppState {
    pub fn new(
        map_size: usize,
        prefix: impl Into<PathBuf>,
        suffix: impl Into<PathBuf>,
        octree_id: impl Into<String>,
        octree_factory: octree::OctreeFactory,
    ) -> Self {
        AppState {
            octree_map: Arc::new(RwLock::new(HashMap::with_capacity(map_size))),
            key_params: OctreeKeyParams {
                prefix: prefix.into(),
                suffix: suffix.into(),
            },
            init_octree_id: octree_id.into(),
            octree_factory: input_octree_factory,
        }
    }

    pub fn load_octree(
        &self,
        octree_id: impl AsRef<str>,
    ) -> Result<Arc<octree::Octree>, PointsViewerError> {
        // exists
        let octree_key = octree_id.as_ref();

        {
            // read access to state
            let map = self.octree_map.read().unwrap();
            let octree = map.get(octree_key);
            //some found
            if let Some(tree) = octree {
                return Ok(Arc::clone(&tree));
            }
        }
        // none found
        self.insert_octree(octree_key.to_string())
    }

    fn insert_octree(
        &self,
        octree_id: impl Into<String>,
    ) -> Result<Arc<octree::Octree>, PointsViewerError> {
        let octree_key = octree_id.into();
        let addr = &self.key_params.get_octree_address(&octree_key);
        let octree: Arc<octree::Octree> = Arc::from(
            self.octree_factory
                .generate_octree(addr.to_string_lossy())?,
        );
        {
            // write access to state
            let mut wmap = self.octree_map.write().unwrap();
            wmap.insert(octree_key.clone(), Arc::clone(&octree));
        }
        Ok(octree)
    }

    pub fn return_init_id(&self) -> Result<String, PointsViewerError> {
        Ok(self.init_octree_id.clone())
    }
}
