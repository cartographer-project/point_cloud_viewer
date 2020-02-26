use super::aabb::AABB;
use super::base::{Relation};
use nalgebra::{Point3, Scalar, RealField, Vector3};
use num_traits::{Bounded, Zero};
use arrayvec::ArrayVec;

pub trait ConvexPolyhedron<S: Scalar + Zero> {
    /// For now, this is hardcoded to 8 corners and up to 6 edges.
    /// I think this is cheaper than allocating a vector.
    /// We could also parametrize this trait by two type-level numbers like nalgebra.
    fn compute_corners(&self) -> [Point3<S>; 8];
    fn compute_edges(&self) -> [Option<Vector3<S>>; 6];
    fn cached_sat_parameters(&self) -> CornersAndEdges<S> {
        let corners = self.compute_corners();
        let mut edges = [nalgebra::zero(); 6];
        let mut num_edges = 0;
        let maybe_edges = self.compute_edges();
        for maybe_edge in ArrayVec::from(maybe_edges) {
            match maybe_edge {
                Some(e) => { edges[num_edges] = e; },
                None => break,
            }
            num_edges += 1;
        }
        CornersAndEdges {corners, edges, num_edges }
    }
}

pub trait CachedConvexPolyhedron<S: Scalar> {
    fn corners(&self) -> &[Point3<S>];
    fn edges(&self) -> &[Vector3<S>];
}

pub struct CornersAndEdges<S: Scalar> {
    corners: [Point3<S>; 8],
    edges: [Vector3<S>; 6],
    num_edges: usize,
}

impl<S: Scalar> CachedConvexPolyhedron<S> for CornersAndEdges<S> {
    fn corners(&self) -> &[Point3<S>] {
        &self.corners
    }
    fn edges(&self) -> &[Vector3<S>] {
        &self.edges[..self.num_edges]
    }
}

/// Accepts anything that can be turned into a 
pub fn sat<S, A, B>(poly_a: A, poly_b: B) -> Relation
where A: CachedConvexPolyhedron<S>,
B: CachedConvexPolyhedron<S>,
S: Scalar + RealField + Bounded, 
{
    unimplemented!()
}
/// The possible separating axes between two convex polyhedra are:
/// The face normals of polyhedron A
/// The face normals of polyhedron B
/// The cross product between all edge combinations of A and B

/// See https://www.gamedev.net/forums/topic/694911-separating-axis-theorem-3d-polygons/ for more detail
pub fn intersects_aabb<S: RealField + Bounded>(
    corners: &[Point3<S>],
    separating_axes: &[Vector3<S>],
    aabb: &AABB<S>,
) -> Relation {
    // SAT algorithm
    // https://gamedev.stackexchange.com/questions/44500/how-many-and-which-axes-to-use-for-3d-obb-collision-with-sat
    for sep_axis in separating_axes.iter() {
        // Project the cube and the box onto that axis
        let mut cube_min_proj: S = Bounded::max_value();
        let mut cube_max_proj: S = Bounded::min_value();
        for corner in aabb.corners().iter() {
            let corner_proj = corner.coords.dot(sep_axis);
            cube_min_proj = cube_min_proj.min(corner_proj);
            cube_max_proj = cube_max_proj.max(corner_proj);
        }
        // Project corners of the box onto that axis
        let mut box_min_proj: S = Bounded::max_value();
        let mut box_max_proj: S = Bounded::min_value();
        for corner in corners.iter() {
            let corner_proj = corner.coords.dot(sep_axis);
            box_min_proj = box_min_proj.min(corner_proj);
            box_max_proj = box_max_proj.max(corner_proj);
        }
        if box_min_proj > cube_max_proj || box_max_proj < cube_min_proj {
            return Relation::Out;
        }
    }
    Relation::Cross
}


#[cfg(test)]
mod tests {
    use super::*;

    struct Dummy {}

    impl ConvexPolyhedron<f64> for Dummy {
        fn compute_corners(&self) -> [Point3<f64>; 8] {
            [Point3::origin(); 8]
        }
        fn compute_edges(&self) -> [Option<Vector3<f64>>; 6] {
            [Vector3::x(), Vector3::y(), Vector3::z(), None, None, None]
        }
    }

    #[test]
    fn test_corners_and_edges() {
        let cae = CornersAndEdges::from(Dummy {});
        assert_eq!(cae.num_edges, 3);
        assert_eq!(cae.edges[0], Vector3::x());
        assert_eq!(cae.edges[1], Vector3::y());
        assert_eq!(cae.edges[2], Vector3::z());
    }

}