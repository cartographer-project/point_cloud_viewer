use point_cloud_client::PointCloudClient;
use point_viewer::math::Isometry3;
use point_viewer::octree::OctreeFactory;
use point_viewer_grpc::octree_from_grpc_address;
use xray::build_quadtree::{run, Extension};

struct NullExtension;

impl Extension for NullExtension {
    fn pre_init<'a, 'b>(app: clap::App<'a, 'b>) -> clap::App<'a, 'b> {
        app
    }
    fn local_from_global(_: &clap::ArgMatches, _: &PointCloudClient) -> Option<Isometry3<f64>> {
        None
    }
}

pub fn main() {
    let octree_factory = OctreeFactory::new().register("grpc://", octree_from_grpc_address);
    run::<NullExtension>(octree_factory);
}
