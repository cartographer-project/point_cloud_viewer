// Some synthetic queries for synthetic data. These are just examples, more can be added.
use crate::synthetic_data::SyntheticData;
use crate::S2_LEVEL;
use nalgebra::Vector3;
use point_viewer::iterator::PointLocation;
use point_viewer::math::{collision::Perspective, Frustum, Obb, FromPoint3};
use s2::cellid::CellID;

pub fn get_abb_query(data: SyntheticData) -> PointLocation {
    let abb = data.bbox();
    PointLocation::Aabb(abb)
}

// An OBB that lies in the center of the point cloud and is aligned with gravity.
// Its half-extent is half of that of the data.
pub fn get_obb_query(data: SyntheticData) -> PointLocation {
    let obb = Obb::new(
        data.ecef_from_local().clone(),
        Vector3::new(
            0.5 * data.half_width,
            0.5 * data.half_width,
            0.5 * data.half_height,
        ),
    );
    PointLocation::Obb(obb)
}
pub fn get_frustum_query(data: SyntheticData) -> PointLocation {
    let ecef_from_local = data.ecef_from_local().clone();
    let perspective = Perspective::new_fov(
        /* fovy */ 1.2,
        /* aspect */ 1.0,
        /* near */ 0.1,
        /* far */ 10.0,
    );
    let frustum = Frustum::new(ecef_from_local, perspective);
    PointLocation::Frustum(frustum)
}

pub fn get_cell_union_query(data: SyntheticData) -> PointLocation {
    let coords = data.ecef_from_local().translation.vector;
    let s2_cell_id = CellID::from_point(&coords).parent(S2_LEVEL);
    let s2_cell_union = s2::cellunion::CellUnion(vec![s2_cell_id, s2_cell_id.next()]);
    PointLocation::S2Cells(s2_cell_union)
}
