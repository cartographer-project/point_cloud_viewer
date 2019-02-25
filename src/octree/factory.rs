use crate::errors::*;
use crate::octree::{octree_from_directory, Octree};
use fnv::FnvHashMap;

type OctreeFactoryFunction = fn(&str) -> Result<Box<Octree>>;

#[derive(Default)]
pub struct OctreeFactory {
    octree_factories: FnvHashMap<String, OctreeFactoryFunction>,
}

impl OctreeFactory {
    pub fn new() -> Self {
        OctreeFactory {
            octree_factories: FnvHashMap::default(),
        }
    }

    pub fn register_octree_factory(
        mut self,
        prefix: impl Into<String>,
        function: OctreeFactoryFunction,
    ) -> OctreeFactory {
        self.octree_factories.insert(prefix.into(), function);
        self
    }

    pub fn generate_octree(&self, octree_argument: impl AsRef<String>) -> Result<Box<Octree>> {
        let octree_argument = octree_argument.as_ref();
        for (prefix, octree_factory_function) in &self.octree_factories {
            if !octree_argument.starts_with(prefix) {
                continue;
            }
            let no_prefix = &octree_argument[prefix.len()..].to_string();
            if let Ok(o) = octree_factory_function(no_prefix) {
                return Ok(o);
            }
        }

        // If no octree was generated, create it from disk
        Ok(Box::new(octree_from_directory(octree_argument)?))
    }
}
