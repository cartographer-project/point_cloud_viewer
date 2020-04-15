// Some synthetic queries for synthetic data. These are just examples, more can be added.
use crate::synthetic_data::SyntheticData;
use crate::S2_LEVEL;
use nalgebra::{Perspective3, Point3, Vector2, Vector3};
use nav_types::{ECEF, WGS84};
use point_viewer::geometry::{Aabb, CellUnion, Frustum, Obb, WebMercatorRect};
use point_viewer::iterator::PointLocation;
use point_viewer::math::{FromPoint3, WebMercatorCoord};
use s2::cellid::CellID;

pub fn get_aabb(data: SyntheticData) -> Aabb<f64> {
    let min_corner = data.bbox().min() + 0.2 * data.bbox().diag();
    let max_corner = data.bbox().min() + 0.8 * data.bbox().diag();
    Aabb::new(min_corner, max_corner)
}

pub fn get_aabb_query(data: SyntheticData) -> PointLocation {
    PointLocation::Aabb(get_aabb(data))
}

// An OBB that lies in the center of the point cloud and is aligned with gravity.
// Its half-extent is half of that of the data.
pub fn get_obb(data: SyntheticData) -> Obb<f64> {
    Obb::new(
        *data.ecef_from_local(),
        Vector3::new(
            0.5 * data.half_width,
            0.5 * data.half_width,
            0.5 * data.half_height,
        ),
    )
}

pub fn get_obb_query(data: SyntheticData) -> PointLocation {
    PointLocation::Obb(get_obb(data))
}

pub fn get_frustum(data: SyntheticData) -> Frustum<f64> {
    let ecef_from_local = *data.ecef_from_local();
    let perspective = Perspective3::new(
        /* aspect */ 1.0, /* fovy */ 1.2, /* near */ 0.1, /* far */ 10.0,
    );
    Frustum::new(ecef_from_local, perspective.into())
}
pub fn get_frustum_query(data: SyntheticData) -> PointLocation {
    PointLocation::Frustum(get_frustum(data))
}

pub fn get_cell_union(data: SyntheticData) -> CellUnion {
    let coords = data.ecef_from_local().translation.vector;
    let s2_cell_id = CellID::from_point(&Point3 { coords }).parent(S2_LEVEL);
    CellUnion(vec![s2_cell_id, s2_cell_id.next()])
}

pub fn get_cell_union_query(data: SyntheticData) -> PointLocation {
    PointLocation::S2Cells(get_cell_union(data))
}

pub fn get_web_mercator_rect(data: SyntheticData) -> WebMercatorRect {
    let center = data.ecef_from_local().translation.vector;
    let ll: WGS84<f64> = ECEF::new(center.x, center.y, center.z).into();
    let wm = WebMercatorCoord::from_lat_lng(&ll);
    // Define a rect with side length 256 (= one tile) at level 21, and a tile is
    // approx. 25m by 25m at level 21.
    let corner_1 = wm.to_zoomed_coordinate(21).unwrap() - Vector2::new(128.0, 128.0);
    let corner_2 = wm.to_zoomed_coordinate(21).unwrap() + Vector2::new(128.0, 128.0);
    WebMercatorRect::from_zoomed_coordinates(corner_1, corner_2, 21).unwrap()
}

pub fn get_web_mercator_rect_query(data: SyntheticData) -> PointLocation {
    PointLocation::WebMercatorRect(get_web_mercator_rect(data))
}
