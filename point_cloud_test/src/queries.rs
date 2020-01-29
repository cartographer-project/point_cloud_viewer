// Some synthetic queries for synthetic data. These are just examples, more can be added.
use crate::synthetic_data::SyntheticData;
use crate::S2_LEVEL;
use cgmath::{PerspectiveFov, Rad, Vector3};
use point_viewer::iterator::PointLocation;
use point_viewer::math::{Frustum, Obb, S2Point};

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
    let perspective = PerspectiveFov {
        fovy: Rad(1.2),
        aspect: 1.0,
        near: 0.1,
        far: 10.0,
    }
    .to_perspective();
    let frustum = Frustum::new(ecef_from_local, perspective);
    PointLocation::Frustum(frustum)
}

pub fn get_cell_union_query(data: SyntheticData) -> PointLocation {
    let coords = data.ecef_from_local().translation;
    let s2_point = S2Point::from(&coords);
    let s2_cell_id = s2::cellid::CellID::from(s2_point).parent(S2_LEVEL);
    let s2_cell_union = s2::cellunion::CellUnion(vec![s2_cell_id, s2_cell_id.next()]);
    PointLocation::S2Cells(s2_cell_union)
}
