//! A web mercator axis-aligned rectangle.

use crate::math::sat::{ConvexPolyhedron, Intersector};
use crate::math::web_mercator::WebMercatorCoord;
use arrayvec::ArrayVec;
use nalgebra::{Point3, Unit, Vector2};
use nav_types::{ECEF, WGS84};

/// The dead sea is at -413m, but we use a more generous minimum
const MIN_ELEVATION_M: f64 = -1000.0;
/// Mt. Everest is at 8,848m, plus we need some safety margin
const MAX_ELEVATION_M: f64 = 9000.0;

/// An axis-aligned rectangle on a Web Mercator map.
pub struct WebMercatorRect {
    north_west: WebMercatorCoord,
    south_east: WebMercatorCoord,
}

impl WebMercatorRect {
    /// Returns `None` when `z` is greater than [`MAX_ZOOM`](index.html#constant.max_zoom)
    /// or when the coordinates are out of bounds for the zoom level `z`.
    pub fn new(min: Vector2<f64>, max: Vector2<f64>, z: u8) -> Option<Self> {
        if z == 0 {
            return None;
        }
        let north_west = WebMercatorCoord::from_zoomed_coordinate(nalgebra::inf(&min, &max), z)?;
        let south_east = WebMercatorCoord::from_zoomed_coordinate(nalgebra::sup(&min, &max), z)?;
        Some(Self {
            north_west,
            south_east,
        })
    }
}

///
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

        let mut face_normals = ArrayVec::new();
        face_normals.push(Unit::new_normalize(edges[0].cross(&edges[8]))); // N face
        face_normals.push(Unit::new_normalize(edges[1].cross(&edges[9]))); // E face
        face_normals.push(Unit::new_normalize(edges[2].cross(&edges[10]))); // S face
        face_normals.push(Unit::new_normalize(edges[3].cross(&edges[11]))); // W face
        face_normals.push(Unit::new_normalize(edges[1].cross(&edges[0]))); // down face
        face_normals.push(Unit::new_normalize(edges[5].cross(&edges[4]))); // up face

        Intersector {
            corners,
            edges,
            face_normals,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::sat::Relation;

    #[test]
    fn sanity_test() {
        let rect_1 = WebMercatorRect::new(Vector2::new(1.0, 1.0), Vector2::new(2.0, 2.0), 1).unwrap();
        let rect_2 = WebMercatorRect::new(Vector2::new(3.0, 1.0), Vector2::new(4.0, 2.0), 1).unwrap();
        let rect_3 = WebMercatorRect::new(Vector2::new(1.5, 1.5), Vector2::new(2.5, 2.5), 1).unwrap();
        let rect_4 = WebMercatorRect::new(Vector2::new(0.0, 0.0), Vector2::new(2.5, 2.5), 1).unwrap();
        assert_eq!(rect_1.intersector().intersect(&rect_2.intersector()), Relation::Out);
        assert_eq!(rect_1.intersector().intersect(&rect_3.intersector()), Relation::Cross);
        assert_eq!(rect_4.intersector().intersect(&rect_1.intersector()), Relation::In);
    }
}
