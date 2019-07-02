use nav_types::{ECEF, ENU, WGS84};
use s2::cellid::CellID;
use s2::latlng::LatLng;
use s2::rect::Rect;
use s2::region::RegionCoverer;
use s2::s1::angle::{Angle, Rad};
use std::vec::Vec;

const LEVEL_MOD: u8 = 1;

trait LatLngExt {
    fn from(_: Self) -> LatLng;
}

impl LatLngExt for WGS84<f64> {
    fn from(wgs: Self) -> LatLng {
        let lat = Angle::from(Rad(wgs.latitude()));
        let lng = Angle::from(Rad(wgs.longitude()));
        LatLng::new(lat, lng)
    }
}

impl LatLngExt for ECEF<f64> {
    fn from(ecef: Self) -> LatLng {
        LatLngExt::from(std::convert::Into::<WGS84<f64>>::into(ecef))
    }
}

pub fn cell_id(ecef_m: ECEF<f64>, level: u8) -> CellID {
    CellID::from(LatLngExt::from(ecef_m)).parent(u64::from(level))
}

pub fn cell_ids(
    ecef_m: ECEF<f64>,
    half_extent_m: ENU<f64>,
    level: u8,
    max_num_cells: usize,
) -> Vec<CellID> {
    let center = LatLngExt::from(ecef_m);
    let rect = vec![
        (-1., -1., -1.),
        (-1., -1., 1.),
        (-1., 1., -1.),
        (-1., 1., 1.),
        (1., -1., -1.),
        (1., -1., 1.),
        (1., 1., -1.),
        (1., 1., 1.),
    ]
    .into_iter()
    .map(|fac| {
        Rect::from(LatLngExt::from(
            ecef_m
                + ENU::new(
                    fac.0 * half_extent_m.east(),
                    fac.1 * half_extent_m.north(),
                    fac.2 * half_extent_m.up(),
                ),
        ))
    })
    .fold(Rect::from(center), |rect, corner| rect.union(&corner));
    let cov = RegionCoverer {
        min_level: level,
        max_level: level,
        level_mod: LEVEL_MOD,
        max_cells: max_num_cells,
    };
    let cu = cov.covering(&rect);
    cu.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use s2::cellid::MAX_LEVEL;

    #[test]
    fn test_cell_id() {
        // Hofbräuhaus Munich.
        let ecef = ECEF::new(4_177_536., 855_997., 4_727_102.);
        assert_eq!(
            "479e758bb8ddef5b",
            cell_id(ecef, MAX_LEVEL as u8).to_token()
        );
    }

    #[test]
    fn test_region_coverer() {
        // Defines a 5x5 m region in Palo Alto.
        let half_side_length = 2.5;
        let ecef_pao = ECEF::new(-2699112.642037565, -4294985.7005343605, 3853368.5643771216);
        let half_extent = ENU::new(half_side_length, half_side_length, half_side_length);

        let tokens_ground_truth = vec!["808fba97497", "808fba97499"];

        let tokens_rect: Vec<String> = cell_ids(ecef_pao, half_extent, 20, 1000)
            .into_iter()
            .map(|cell_id| cell_id.to_token())
            .collect();
        assert_eq!(tokens_ground_truth, tokens_rect);
    }
}
