use crate::backend_error::PointsViewerError;
use point_viewer::octree;
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

/// path information for the octrees
#[derive(Clone)]
pub struct OctreeKeyParams {
    /// Location prefix, including
    prefix: String,
    /// Tree ID
    suffix: String,
}

impl OctreeKeyParams {
    pub fn get_octree_address(&self, octree_key: &String) -> Result<String, PointsViewerError> {
        let mut join_prefix = "";
        let mut join_suffix = "";

        if !self.prefix.ends_with("/") {
            join_prefix = "/";
        }
        if !self.suffix.starts_with("/") {
            join_suffix = "/";
        }

        Ok(format!(
            "{}{}{}{}{}",
            self.prefix, join_prefix, octree_key, join_suffix, self.suffix
        ))
    }
}

#[derive(Clone)]
pub struct AppState {
    /// LRU Cache for Octrees
    pub octree_map: Arc<RwLock<HashMap<String, Arc<octree::Octree>>>>,
    /// information for retieving octree path
    pub key_params: OctreeKeyParams,
    /// backward compatibility to input arguments
    pub init_uuid: String,
}

impl AppState {
    pub fn new(
        map_size: usize,
        prefix: impl Into<String>,
        suffix: impl Into<String>,
        uuid: impl Into<String>,
    ) -> Self {
        AppState {
            octree_map: Arc::new(RwLock::new(HashMap::with_capacity(map_size))),
            key_params: OctreeKeyParams {
                prefix: prefix.into(),
                suffix: suffix.into(),
            },
            init_uuid: uuid.into(),
        }
    }

    pub fn load_octree(
        &self,
        uuid: impl AsRef<str>,
    ) -> Result<Arc<octree::Octree>, PointsViewerError> {
        // exists
        let octree_id = uuid.as_ref();
        // taking care of initial octree
        if octree_id.len() == 9 && octree_id.starts_with("init_uuid") {
            let uuid = &self.init_uuid.clone();
            return self.load_octree(&uuid);
        }

        {
            // read access to state
            let map = self.octree_map.read().unwrap();
            let octree = map.get(octree_id);

            //some found
            if let Some(tree) = octree {
                return Ok(Arc::clone(&tree));
            }
        }
        // none found
        let octree_key: String = octree_id.to_string();
        self.insert_octree(octree_key)
    }

    fn insert_octree(
        &self,
        uuid: impl Into<String>,
    ) -> Result<Arc<octree::Octree>, PointsViewerError> {
        let octree_key = uuid.into();
        let addr = &self.key_params.get_octree_address(&octree_key)?.clone();
        println!("Current tree address to insert:{}", addr);
        let octree: Arc<octree::Octree> = Arc::from(octree::octree_from_directory(&addr)?);
        {
            // write access to state
            let mut wmap = self.octree_map.write().unwrap();
            wmap.insert(octree_key.clone(), Arc::clone(&octree));
        }
        Ok(octree)
    }
}
