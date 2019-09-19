use point_viewer::octree::build_octree;
use point_viewer::read_write::{Encoding, NodeWriter, OpenMode, RawNodeWriter, S2Splitter};
use random_points::{Batched, RandomPointsOnEarth};
use std::path::PathBuf;
use std::time::Instant;
use structopt::StructOpt;

mod random_points;

#[derive(StructOpt, Debug)]
struct CommandlineArguments {
    /// Output directory to write the octree into.
    #[structopt(long = "output_directory", parse(from_os_str))]
    output_directory: PathBuf,

    /// Minimal precision that this point cloud should have.
    /// This decides on the number of bits used to encode each node.
    #[structopt(long = "resolution", default_value = "0.001")]
    resolution: f64,

    /// Width of the area to generate points for (used for local x and y)
    #[structopt(long = "width", default_value = "200.0")]
    width: f64,

    /// Height of the area to generate points for
    #[structopt(long = "height", default_value = "20.0")]
    height: f64,

    /// The number of threads used to shard octree building. Set this as high as possible for SSDs.
    #[structopt(long = "num_threads", default_value = "10")]
    num_threads: usize,

    /// The number of points in the test point cloud
    #[structopt(long = "num_points", default_value = "1000000")]
    num_points: usize,

    /// The number of points in the test point cloud
    #[structopt(long = "batch_size", default_value = "1000")]
    batch_size: usize,
}

fn main() {
    let args = CommandlineArguments::from_args();
    let mut points_oct = RandomPointsOnEarth::new(args.width, args.height);
    let points_s2 = points_oct.clone();
    let pool = scoped_pool::Pool::new(args.num_threads);
    std::fs::create_dir(&args.output_directory).expect("Could not create output directory.");

    let start_oct = Instant::now();
    build_octree(
        &pool,
        args.output_directory.join("octree"),
        args.resolution,
        points_oct.bbox(),
        points_oct.take(args.num_points),
    );
    let elapsed_oct = start_oct.elapsed();

    std::fs::create_dir(&args.output_directory.join("s2"))
        .expect("Could not create output directory.");

    let start_s2 = Instant::now();
    let mut s2_writer: S2Splitter<RawNodeWriter> = S2Splitter::new(
        args.output_directory.join("s2"),
        Encoding::Plain,
        OpenMode::Truncate,
    );
    let points_s2 = points_s2.take(args.num_points);
    Batched::new(points_s2, args.batch_size)
        .try_for_each(|batch| s2_writer.write(&batch))
        .expect("Writing failed");
    let elapsed_s2 = start_s2.elapsed();

    println!("###################################################");
    println!(
        "Octree creation ({} threads) took {:?}",
        args.num_threads, elapsed_oct
    );
    println!("S2 cells creation (single thread) took {:?}", elapsed_s2);
}
