use point_viewer::octree::OctreeFactory;
use point_viewer_grpc::octree_from_grpc_address;
use xray::build_quadtree::{parse_arguments, point_cloud_client, run, Extension};

struct NullExtension;

impl Extension for NullExtension {
    fn pre_init<'a, 'b>(app: clap::App<'a, 'b>) -> clap::App<'a, 'b> {
        app
    }
}

pub fn main() {
    let args = parse_arguments::<NullExtension>();
    let octree_factory = OctreeFactory::new().register("grpc://", octree_from_grpc_address);
    let point_cloud_client = point_cloud_client(&args, octree_factory);
    run(&args, &point_cloud_client, None);
}
