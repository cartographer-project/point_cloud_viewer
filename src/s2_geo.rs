use cgmath::Vector2;
use nav_types::{ECEF, WGS84};
use s2::cap::Cap;
use s2::cellid::CellID;
use s2::latlng::LatLng;
use s2::point::Point;
use s2::rect::Rect;
use s2::region::RegionCoverer;
use s2::s1::angle::{Angle, Rad};
use s2::s1::chordangle::ChordAngle;
use std::vec::Vec;

const LEVEL_MOD: u8 = 1;

// These consts are publicly defined in https://github.com/nordmoen/nav-types/blob/master/src/wgs84.rs,
// but unfortunately not exposed via the lib.
const INVERSE_FLATTENING: f64 = 298.257_223_563;
const FLATTENING: f64 = 1.0 / INVERSE_FLATTENING;
const SEMI_MAJOR_AXIS: f64 = 6_378_137.0;

fn radius_at(lat_rad: f64) -> f64 {
    let lat_rad_sin = lat_rad.sin();
    SEMI_MAJOR_AXIS * (1.0 - FLATTENING * lat_rad_sin * lat_rad_sin)
}

fn tangent_m_to_rad(meters: f64, lat_rad: f64) -> f64 {
    2.0 * (0.5 * meters / radius_at(lat_rad)).atan()
}

trait LatLngExt {
    fn from(_: Self) -> LatLng;
}
impl LatLngExt for (f64, f64) {
    fn from(lat_lng_rad: Self) -> LatLng {
        let lat = Angle::from(Rad(lat_lng_rad.0));
        let lng = Angle::from(Rad(lat_lng_rad.1));
        LatLng::new(lat, lng)
    }
}
impl LatLngExt for WGS84<f64> {
    fn from(wgs: Self) -> LatLng {
        LatLngExt::from((wgs.latitude(), wgs.longitude()))
    }
}
impl LatLngExt for ECEF<f64> {
    fn from(ecef: Self) -> LatLng {
        LatLngExt::from(std::convert::Into::<WGS84<f64>>::into(ecef))
    }
}

pub fn cell_id(ecef: ECEF<f64>, level: u8) -> CellID {
    CellID::from(LatLngExt::from(ecef)).parent(u64::from(level))
}

pub fn cell_ids_rect(
    ecef_m: ECEF<f64>,
    extent_m: Vector2<f64>,
    level: u8,
    max_num_cells: usize,
) -> Vec<CellID> {
    let center = LatLngExt::from(ecef_m);
    let size = LatLngExt::from((
        tangent_m_to_rad(extent_m.x, center.lat.rad()),
        tangent_m_to_rad(extent_m.y, center.lat.rad()),
    ));
    let rect = Rect::from_center_size(center, size);
    let cov = RegionCoverer {
        min_level: level,
        max_level: level,
        level_mod: LEVEL_MOD,
        max_cells: max_num_cells,
    };
    let cu = cov.covering(&rect);
    cu.0
}

pub fn cell_ids_radius(
    ecef_m: ECEF<f64>,
    radius_m: f64,
    level: u8,
    max_num_cells: usize,
) -> Vec<CellID> {
    let center = LatLngExt::from(ecef_m);
    let radius_rad = tangent_m_to_rad(radius_m, center.lat.rad());
    let length2 = radius_rad * radius_rad;
    let cap = Cap::from_center_chordangle(
        &Point::from(center),
        &ChordAngle::from_squared_length(length2),
    );
    let cov = RegionCoverer {
        min_level: level,
        max_level: level,
        level_mod: LEVEL_MOD,
        max_cells: max_num_cells,
    };
    let cu = cov.covering(&cap);
    cu.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use s2::cellid::MAX_LEVEL;

    #[test]
    fn test_radius() {
        assert_eq!(SEMI_MAJOR_AXIS, radius_at(0.0));
        assert_eq!(
            SEMI_MAJOR_AXIS * (1.0 - FLATTENING),
            radius_at(std::f64::consts::FRAC_PI_2)
        );
    }

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
        let length = 5.0;
        let ecef_pao = ECEF::new(-2699112.642037565, -4294985.7005343605, 3853368.5643771216);
        let extent = Vector2::new(length, length);

        let tokens_ground_truth = vec!["808fba97497", "808fba97499"];

        let tokens_rect: Vec<String> = cell_ids_rect(ecef_pao, extent, 20, 1000)
            .into_iter()
            .map(|cell_id| cell_id.to_token())
            .collect();
        assert_eq!(tokens_ground_truth, tokens_rect);

        let tokens_radius: Vec<String> = cell_ids_radius(ecef_pao, 0.5 * length, 20, 1000)
            .into_iter()
            .map(|cell_id| cell_id.to_token())
            .collect();
        assert_eq!(tokens_ground_truth, tokens_radius);
    }
}
