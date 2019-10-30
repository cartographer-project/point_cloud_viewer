use point_cloud_client::PointCloudClient;
use point_viewer::data_provider::DataProviderFactory;
use point_viewer::errors::{ErrorKind, Result};
use point_viewer::iterator::{PointLocation, PointQuery};
use structopt::StructOpt;

#[derive(StructOpt)]
#[structopt(about = "S2 Cells from hull around ECEF positions.")]
struct CommandlineArguments {
    /// The locations containing the s2 cell data.
    #[structopt(parse(from_str), raw(required = "true"))]
    locations: Vec<String>,

    /// Point belonging to the bounding box to be queried.
    #[structopt(long, parse(try_from_str = "point3f64_from_str"))]
    minLatLon: Vec<Point3<f64>>,

    /// Point belonging to the bounding box to be queried.
    #[structopt(long, parse(try_from_str = "point3f64_from_str"))]
    maxLatLon: Vec<Point3<f64>>,

    /// max altitude [m] belonging to the bounding box to be queried.
    #[structopt(
        long,
        default = 8848.0, // [m] everest
        parse(try_from_str = "point3f64_from_str")
    )]
    max_alt_m: float,

    /// min altitude [m] belonging to the bounding box to be queried.
    #[structopt(
        long,
        default = -400.0, // [m] Dead sea
        parse(try_from_str = "point3f64_from_str")
    )]
    min_alt_m: float,
    /// The minimal tolerance in meters around the specified points.
    #[structopt(long, default_value = "100")]
    margin_m: uint,
}

fn main() -> Result<()> {
    let args = CommandlineArguments::from_args();
    let mut point_cloud_client = PointCloudClient::new(&args.locations, DataProviderFactory::new())
        .expect("Couldn't create point cloud client.");
    point_cloud_client.num_threads = args.num_threads;

    // create query:
    // get ECEF local normal
    // convert to ECEF with max altitude (everest) and bottom (salt sea)

    // read the Attributes
    // use the meta pb to pass over the cell ids and stuff?

    // get cells for min/max s
    // check the difference with height / local normal
    // do some experiments here

    // intersecting bounding box when defined?
    // TODO: read from bounding box in the pointcloud meta the min/max altitudes,
    // convert altitude with lat lon to ecef
    // grow an obb`

    let point_location = PointQuery {
        attributes: Vec::new(), // not needed
        location: PointLocation::Aabb(Aabb3::new(args.min, args.max)),
        global_from_query: None,
    };
}
