use cgmath::{BaseFloat, EuclideanSpace, Point3, Vector3};
use collision::Aabb3;
use num_traits::Float;

pub fn intersects_aabb3<S: BaseFloat>(
    corners: &[Point3<S>],
    separating_axes: &[Vector3<S>],
    aabb: &Aabb3<S>,
) -> bool {
    // SAT algorithm
    // https://gamedev.stackexchange.com/questions/44500/how-many-and-which-axes-to-use-for-3d-obb-collision-with-sat
    for sep_axis in separating_axes.iter() {
        // Project the cube and the box onto that axis
        let mut cube_min_proj: S = Float::max_value();
        let mut cube_max_proj: S = Float::min_value();
        for corner in aabb.to_corners().iter() {
            let corner_proj = corner.dot(*sep_axis);
            cube_min_proj = cube_min_proj.min(corner_proj);
            cube_max_proj = cube_max_proj.max(corner_proj);
        }
        // Project corners of the box onto that axis
        let mut box_min_proj: S = Float::max_value();
        let mut box_max_proj: S = Float::min_value();
        for corner in corners.iter() {
            let corner_proj = corner.dot(*sep_axis);
            box_min_proj = box_min_proj.min(corner_proj);
            box_max_proj = box_max_proj.max(corner_proj);
        }
        if box_min_proj > cube_max_proj || box_max_proj < cube_min_proj {
            return false;
        }
    }
    true
}
