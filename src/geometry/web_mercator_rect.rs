//! A Web Mercator axis-aligned rectangle.

use crate::math::base::{HasAabbIntersector, PointCulling};
use crate::math::sat::{CachedAxesIntersector, ConvexPolyhedron, Intersector};
use crate::math::web_mercator::WebMercatorCoord;
use arrayvec::ArrayVec;
use nalgebra::{Point3, RealField, Unit, Vector2};
use nav_types::{ECEF, WGS84};
use serde::{Deserialize, Serialize};

/// The dead sea is at -413m, but we use a more generous minimum
const MIN_ELEVATION_M: f64 = -1000.0;
/// Mt. Everest is at 8,848m, plus we need some safety margin
const MAX_ELEVATION_M: f64 = 9000.0;

/// A rectangle on a Web Mercator map, not rotated.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WebMercatorRect {
    north_west: WebMercatorCoord,
    south_east: WebMercatorCoord,
}

impl WebMercatorRect {
    /// Returns `None` when `z` is greater than [`MAX_ZOOM`](index.html#constant.max_zoom)
    /// or when the coordinates are out of bounds for the zoom level `z`. It also returns `None`
    /// when the rect is larger than
    /// Rects crossing the ±180° longitude line are not supported at the moment.
    pub fn from_zoomed_coordinates(min: Vector2<f64>, max: Vector2<f64>, z: u8) -> Option<Self> {
        // TODO(nnmm): Use inherent inf()/sup() when we switch to 0.21.0
        let north_west = WebMercatorCoord::from_zoomed_coordinate(nalgebra::inf(&min, &max), z)?;
        let south_east = WebMercatorCoord::from_zoomed_coordinate(nalgebra::sup(&min, &max), z)?;
        Some(Self {
            north_west,
            south_east,
        })
    }
}

/// This is calculating the volume of all points in space which, when projected
/// to Web Mercator, fall into the given rectangle.
/// Implemented by extruding the rectangle's four corners along their altitude
/// axis up and down, which results in a convex polyhedron.
impl ConvexPolyhedron<f64> for WebMercatorRect {
    fn compute_corners(&self) -> [Point3<f64>; 8] {
        let n_w = self.north_west.to_lat_lng();
        let s_e = self.south_east.to_lat_lng();
        let ecef_point = |lat: WGS84<f64>, lng: WGS84<f64>, elevation: f64| -> Point3<f64> {
            let lat_lng = WGS84::new(lat.latitude_degrees(), lng.longitude_degrees(), elevation);
            let ecef = ECEF::from(lat_lng);
            Point3::new(ecef.x(), ecef.y(), ecef.z())
        };
        [
            ecef_point(n_w, n_w, MIN_ELEVATION_M), // NW down
            ecef_point(n_w, s_e, MIN_ELEVATION_M), // NE down
            ecef_point(s_e, s_e, MIN_ELEVATION_M), // SE down
            ecef_point(s_e, n_w, MIN_ELEVATION_M), // SW down
            ecef_point(n_w, n_w, MAX_ELEVATION_M), // NW up
            ecef_point(n_w, s_e, MAX_ELEVATION_M), // NE up
            ecef_point(s_e, s_e, MAX_ELEVATION_M), // SE up
            ecef_point(s_e, n_w, MAX_ELEVATION_M), // SW up
        ]
    }

    fn intersector(&self) -> Intersector<f64> {
        let corners = self.compute_corners();
        let edges = ArrayVec::from([
            Unit::new_normalize(corners[1] - corners[0]), // N edge, down
            Unit::new_normalize(corners[2] - corners[1]), // E edge, down
            Unit::new_normalize(corners[3] - corners[2]), // S edge, down
            Unit::new_normalize(corners[0] - corners[3]), // W edge, down
            Unit::new_normalize(corners[5] - corners[4]), // N edge, up
            Unit::new_normalize(corners[6] - corners[5]), // E edge, up
            Unit::new_normalize(corners[7] - corners[6]), // S edge, up
            Unit::new_normalize(corners[4] - corners[7]), // W edge, up
            Unit::new_normalize(corners[4] - corners[0]), // NW edge
            Unit::new_normalize(corners[5] - corners[1]), // NE edge
            Unit::new_normalize(corners[6] - corners[2]), // SE edge
            Unit::new_normalize(corners[7] - corners[3]), // SW edge
        ]);

        let face_normals = ArrayVec::from([
            Unit::new_normalize(edges[0].cross(&edges[8])), // N face
            Unit::new_normalize(edges[1].cross(&edges[9])), // E face
            Unit::new_normalize(edges[2].cross(&edges[10])), // S face
            Unit::new_normalize(edges[3].cross(&edges[11])), // W face
            Unit::new_normalize(edges[1].cross(&edges[0])), // down face
            Unit::new_normalize(edges[5].cross(&edges[4])), // up face
        ]);

        Intersector {
            corners,
            edges,
            face_normals,
        }
    }
}

has_aabb_intersector_for_convex_polyhedron!(WebMercatorRect);

impl<S: RealField> PointCulling<S> for WebMercatorRect
where
    f64: From<S>,
{
    fn contains(&self, point: &Point3<S>) -> bool {
        let ll: WGS84<f64> =
            ECEF::new(f64::from(point.x), f64::from(point.y), f64::from(point.z)).into();
        let wmc = WebMercatorCoord::from_lat_lng(&ll);
        nalgebra::partial_le(&self.north_west, &wmc) && nalgebra::partial_lt(&wmc, &self.south_east)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::sat::Relation;

    #[test]
    fn intersection_test() {
        let rect_1 =
            WebMercatorRect::from_zoomed_coordinates(Vector2::new(1.0, 1.0), Vector2::new(3.0, 3.0), 1).unwrap();
        let rect_2 =
            WebMercatorRect::from_zoomed_coordinates(Vector2::new(4.0, 4.0), Vector2::new(5.0, 5.0), 1).unwrap();
        let rect_3 =
            WebMercatorRect::from_zoomed_coordinates(Vector2::new(2.0, 2.0), Vector2::new(6.0, 6.0), 1).unwrap();
        let rect_1_intersector = rect_1.intersector();
        let rect_2_intersector = rect_2.intersector();
        let rect_3_intersector = rect_3.intersector();
        assert_eq!(
            rect_1_intersector.intersect(&rect_2_intersector),
            Relation::Out
        );
        assert_eq!(
            rect_1_intersector.intersect(&rect_3_intersector),
            Relation::Cross
        );
        // Why Cross and not In? Because rect_2 is taller than rect_3.
        assert_eq!(
            rect_3_intersector.intersect(&rect_2_intersector),
            Relation::Cross
        );
    }
}
