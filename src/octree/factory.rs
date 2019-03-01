use crate::errors::*;
use crate::octree::{octree_from_directory, Octree};
use fnv::FnvHashMap;

type OctreeFactoryFunction = fn(&str) -> Result<Box<Octree>>;

#[derive(Default, Clone)]
pub struct OctreeFactory {
    octree_fn_map: FnvHashMap<String, OctreeFactoryFunction>,
}

impl OctreeFactory {
    pub fn new() -> Self {
        OctreeFactory {
            octree_fn_map: FnvHashMap::default(),
        }
    }

    pub fn register(
        mut self,
        prefix: impl Into<String>,
        function: OctreeFactoryFunction,
    ) -> OctreeFactory {
        self.octree_fn_map.insert(prefix.into(), function);
        self
    }

    pub fn generate_octree(&self, octree_argument: impl AsRef<str>) -> Result<Box<Octree>> {
        let octree_argument = octree_argument.as_ref();
        for (prefix, octree_factory_function) in &self.octree_fn_map {
            if !octree_argument.starts_with(prefix) {
                continue;
            }
            if let Ok(o) = octree_factory_function(octree_argument) {
                return Ok(o);
            }
        }

        // If no octree was generated, create it from disk
        octree_from_directory(octree_argument)
    }
}
