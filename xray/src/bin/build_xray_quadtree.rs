use nalgebra::Isometry3;
use point_viewer::data_provider::DataProviderFactory;
use xray::build_quadtree::{run, Extension};

struct NullExtension;

impl Extension for NullExtension {
    fn pre_init(app: clap::App) -> clap::App {
        app
    }
    fn query_from_global(_: &clap::ArgMatches) -> Option<Isometry3<f64>> {
        None
    }
}

pub fn main() {
    let data_provider_factory = DataProviderFactory::new();
    run::<NullExtension>(data_provider_factory);
}
