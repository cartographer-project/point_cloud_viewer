use point_viewer::octree::OctreeFactory;
use point_viewer_grpc::octree_from_grpc_address;
use xray::build_quadtree::run;

pub fn main() {
    let octree_factory = OctreeFactory::new().register("grpc://", octree_from_grpc_address);
    run(octree_factory);
}
