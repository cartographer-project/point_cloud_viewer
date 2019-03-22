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
    octree_map: Arc<RwLock<HashMap<String, Arc<octree::Octree>>>>,
    /// information for retieving octree path
    key_params: OctreeKeyParams,
    /// backward compatibility to input arguments
    init_octree_id: String,
}

impl AppState {
    pub fn new(
        map_size: usize,
        prefix: impl Into<PathBuf>,
        suffix: impl Into<PathBuf>,
        octree_id: impl Into<String>,
    ) -> Self {
        AppState {
            octree_map: Arc::new(RwLock::new(HashMap::with_capacity(map_size))),
            key_params: OctreeKeyParams {
                prefix: prefix.into(),
                suffix: suffix.into(),
            },
            init_octree_id: octree_id.into(),
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
        println!("Current tree address to insert:{}", addr.to_str().unwrap());
        let octree: Arc<octree::Octree> = Arc::from(octree::octree_from_directory(&addr)?);
        {
            // write access to state
            let mut wmap = self.octree_map.write().unwrap();
            wmap.insert(octree_key.clone(), Arc::clone(&octree));
        }
        Ok(octree)
    }

    pub fn get_init_id(&self) -> String {
        self.init_octree_id.clone()
    }
}
