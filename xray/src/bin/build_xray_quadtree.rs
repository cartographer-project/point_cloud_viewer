use point_viewer::octree::OctreeFactory;
use point_viewer_grpc::octree_from_grpc_address;
use xray::build_quadtree::{parse_arguments, point_cloud_client, run};

pub fn main() {
    let args = parse_arguments();
    let octree_factory = OctreeFactory::new().register("grpc://", octree_from_grpc_address);
    let point_cloud_client = point_cloud_client(&args, octree_factory);
    run(&args, &point_cloud_client, None);
}
