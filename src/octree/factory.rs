use crate::octree::{Octree}

type OctreeFactory = fn(&String) -> Result<Box<Octree>, Box<Error>>;

#[derive(Default)]
pub struct OctreeFactoryMap {
    octree_factories: FnvHashMap<String, OctreeFactory>,
}

pub fn register_octree_factory(mut self, prefix: Into<String>, function: OctreeFactory) -> OctreeFactoryMap {
        self.octree_factories.insert(prefix, function);
        self
    }

pub fn generate_octree(octree_argument: Into<String>)->Option<Box<Octree>>{
    let mut octree_opt: Option<Box<Octree>> = None;
    let mut pose_path = None;
    for (prefix, octree_factory_function) in &self.octree_factories {
            if !octree_argument.starts_with(prefix) {
                continue;
            }
            let no_prefix = &octree_argument[prefix.len()..].to_string();
            if let Ok(o) = octree_factory_function(no_prefix) {
                octree_opt = Some(o);
                break;
            }
        }

        // If no octree was generated, create it from disk
         let octree = Arc::new(octree_opt.unwrap_or_else(|| {
            Box::new(octree_from_directory(octree_argument).unwrap()) as Box<Octree>
        }));
}
