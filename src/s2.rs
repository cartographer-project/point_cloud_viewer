use crate::math::Isometry3;
use cgmath::{Point3, Rotation, Vector2};
use nav_types::{ECEF, WGS84};
use s2::cellid::CellID;
use s2::latlng::LatLng;
use s2::rect::Rect;
use s2::region::RegionCoverer;
use s2::s1::angle::{Angle, Rad};
use s2::s1::interval::Interval;
use std::vec::Vec;

const LEVEL_MOD: u8 = 1;

fn lat_lng_from_ecef(ecef: ECEF<f64>) -> LatLng {
    let wgs = WGS84::from(ecef);
    let lat = Angle::from(Rad(wgs.latitude()));
    let lng = Angle::from(Rad(wgs.longitude()));
    LatLng { lat, lng }
}

fn lat_lng_from_point3(point: Point3<f64>) -> LatLng {
    lat_lng_from_ecef(ECEF::new(point.x, point.y, point.z))
}

pub fn cell_id(ecef: ECEF<f64>, level: u8) -> CellID {
    CellID::from(lat_lng_from_ecef(ecef)).parent(level as u64)
}

pub fn cell_ids(
    ecef_from_local: &Isometry3<f64>,
    half_extent: &Vector2<f64>,
    level: u8,
    max_cells: usize,
) -> Vec<CellID> {
    let corner_from = |x: f64, y: f64| {
        ecef_from_local
            .rotation
            .rotate_point(Point3::new(x, y, 0.0))
            + ecef_from_local.translation
    };
    let lat_lng_bl = lat_lng_from_point3(corner_from(-half_extent.x, -half_extent.y));
    let lat_lng_tr = lat_lng_from_point3(corner_from(half_extent.x, half_extent.y));
    let lat_interval = Interval::new(lat_lng_bl.lat.rad(), lat_lng_tr.lat.rad());
    let lng_interval = Interval::new(lat_lng_bl.lng.rad(), lat_lng_tr.lng.rad());
    let center = LatLng::new(
        Angle::from(Rad(lat_interval.center())),
        Angle::from(Rad(lng_interval.center())),
    );
    let size = LatLng::new(
        Angle::from(Rad(lat_interval.len())),
        Angle::from(Rad(lng_interval.len())),
    );
    let rect = Rect::from_center_size(center, size);
    let cov = RegionCoverer {
        min_level: level,
        max_level: level,
        level_mod: LEVEL_MOD,
        max_cells,
    };
    let cu = cov.covering(&rect);
    cu.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use cgmath::{Quaternion, Vector3};
    use s2::cellid::MAX_LEVEL;

    #[test]
    fn test_cell_id() {
        // Hofbräuhaus Munich.
        let ecef = ECEF::new(4_177_536., 855_997., 4_727_102.);
        assert_eq!(
            "479e758bb8ddeb63",
            cell_id(ecef, MAX_LEVEL as u8).to_token()
        );
    }

    #[test]
    fn test_region_coverer() {
        // Defines a 5x5 m region in Palo Alto.
        let ecef_from_pao = Isometry3::new(
            Quaternion::new(
                0.86146710865463172,
                0.42569826530078786,
                -0.12265672768724968,
                -0.24821509780669035,
            ),
            Vector3::new(-2699112.642037565, -4294985.7005343605, 3853368.5643771216),
        );
        let half_extent = Vector2::new(2.5, 2.5);
        let tokens: Vec<String> = cell_ids(&ecef_from_pao, &half_extent, 20, 1000)
            .into_iter()
            .map(|cell_id| cell_id.to_token())
            .collect();
        assert_eq!(vec!["808fba97497", "808fba97499"], tokens);
    }
}
