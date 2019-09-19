use point_viewer::octree::build_octree;
use point_viewer::Point;
use point_viewer::color::RED;
use rand::rngs::ThreadRng;
use rand::Rng;
use cgmath::{
    BaseFloat, BaseNum, Decomposed, EuclideanSpace, Matrix3, Matrix4, Point3, Quaternion,
    Rotation, Vector3, Deg, Transform,
};
use collision::{Aabb3};
use nav_types::{ECEF, WGS84};
use structopt::StructOpt;
use std::path::PathBuf;

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
}

fn main() {
	let args = CommandlineArguments::from_args();
	let mut points = RandomPointsOnEarth::new(args.width, args.height);
  let pool = scoped_pool::Pool::new(args.num_threads);
  std::fs::create_dir(&args.output_directory).expect("Could not create output directory.");
	build_octree(
    &pool,
    args.output_directory.join("octree"),
    args.resolution,
    points.bbox(),
    points.take(args.num_points)
   );
}


struct RandomPointsOnEarth {
	rng: ThreadRng,
	half_width: f64,
	half_height: f64,
	ecef_from_local: Matrix4<f64>,
}

impl RandomPointsOnEarth {
	pub fn new(width: f64, height: f64) -> Self {
		let mut rng = ThreadRng::default();
		let lat = dbg!(rng.gen_range(-90.0, 90.0));
		let lon = dbg!(rng.gen_range(-180.0, 180.0));
		let ecef_from_local = local_frame_from_lat_lng(lat, lon);
		RandomPointsOnEarth {
			rng,
			half_width: width * 0.5,
			half_height: height * 0.5,
			ecef_from_local,
		}
	}

	pub fn next_pos(&mut self) -> Point3<f64> {
		let x = self.rng.gen_range(-self.half_width, self.half_width);
		let y = self.rng.gen_range(-self.half_width, self.half_width);
		let z = self.rng.gen_range(-self.half_height, self.half_height);
		let pt_local = Point3::new(x, y, z);
		self.ecef_from_local.transform_point(pt_local)
	}

	pub fn bbox(&mut self) -> Aabb3<f64> {
		let local_min = Point3::new(-self.half_width, -self.half_width, -self.half_height);
		let local_max = Point3::new(self.half_width, self.half_width, self.half_height);
		// take center of global bbox as initial value for min and max corner
		let mut ecef_min = Point3::from_vec(self.ecef_from_local.w.clone().truncate());
		let mut ecef_max = Point3::from_vec(self.ecef_from_local.w.clone().truncate());
		Aabb3::new(local_min, local_max)
			.to_corners()
			.iter()
			.map(|pt| self.ecef_from_local.transform_point(*pt))
			.for_each(|corner| {
				ecef_min.x = ecef_min.x.min(corner.x);
				ecef_min.y = ecef_min.y.min(corner.y);
				ecef_min.z = ecef_min.z.min(corner.z);
				ecef_max.x = ecef_max.x.max(corner.x);
				ecef_max.y = ecef_max.y.max(corner.y);
				ecef_max.z = ecef_max.z.max(corner.z);
			});
		Aabb3::new(ecef_min, ecef_max)
	}
}

impl Iterator for RandomPointsOnEarth {
	type Item = Point;

	fn next(&mut self) -> Option<Point> {
		let pos = self.next_pos();
		// println!("pos {:?}, bbox {:?}", pos, self.bbox());
		Some(Point {
			position: pos.to_vec(),
			color: RED.to_u8(),
			intensity: None,
		})
	}
}

// Returns transform needed to go from ECEF to local frame with the specified origin where
// the axes are ENU (east, north, up <in the direction normal to the oblate spheroid
// used as Earth's ellipsoid, which does not generally pass through the center of the Earth>)
// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU
// transcribed from `localFrameFromEcefWgs84()` in avsoftware/src/common/math/geo_math.cc
pub fn local_frame_from_lat_lng(
    lat: f64, lon: f64
) -> Matrix4<f64> {
    const PI_HALF: Deg<f64> = Deg(90.0);
    let lat_lng_alt = dbg!(WGS84::new(lat, lon, 0.0));
    let origin = ECEF::from(lat_lng_alt);
    let origin_vector = Vector3::new(origin.x(), origin.y(), origin.z());
    println!("origin_vector {:?}", origin_vector);
    let rotation_matrix = Matrix3::from_angle_z(-PI_HALF)
        * Matrix3::from_angle_y(Deg(lat_lng_alt.latitude_degrees()) - PI_HALF)
        * Matrix3::from_angle_z(Deg(-lat_lng_alt.longitude_degrees()));
    let rotation = Quaternion::from(rotation_matrix);

    let frame = Decomposed {
        scale: 1.0,
        rot: rotation,
        disp: rotation.rotate_vector(-origin_vector),
    };
    dbg!(Matrix4::from(frame).transform_point(Point3::new(0.0, 0.0, 0.0)));
    dbg!(Matrix4::from(frame))
}