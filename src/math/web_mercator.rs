//! Calculations with Web Mercator coordinates and tiles.

use nalgebra::Vector2;
use nav_types::WGS84;
use std::f64::consts::{FRAC_1_PI, PI};

/// 2.0 * E.powf(PI).arctan() - FRAC_PI_2;
/// In degrees, it's 85.051129 (cf. Wikipedia)
const LAT_BOUND_RAD: f64 = 1.484_422_229_745_332_444_394_989_579_450_34;

/// LAT_BOUND_SIN = sin(LAT_BOUND_RAD)
const LAT_BOUND_SIN: f64 = 0.996_272_076_220_749_980_279_833_835_083_99;

const TWO_PI: f64 = 2.0 * PI;
const FOUR_PI: f64 = 4.0 * PI;
const FOUR_PI_INV: f64 = 0.25 * FRAC_1_PI;
const TILE_SIZE: u32 = 256;

/// The max zoom level is currently 23 because it makes implementation easier,
/// but theoretically nothing stops us from going deeper.
pub const MAX_ZOOM: u8 = 23;

/// A Web Mercator coordinate. Essentially a position in a 2D map of the world.
#[derive(Copy, Clone, Debug, PartialEq, PartialOrd)]
pub struct WebMercatorCoord {
    /// Implementation detail: This is normalized to [0, 1), so not zoom level 0.
    /// This makes calculations a bit simpler.
    xy: Vector2<f64>,
}

impl WebMercatorCoord {
    /// Projects a lat/lng coordinate to Web Mercator.
    ///
    /// Equivalent to the formula on [Wikipedia](https://en.wikipedia.org/wiki/Web_Mercator_projection#Formulas).
    /// If the latitude is outside `[-85.051129, 85.051129]`, it is clamped to that interval first.
    pub fn from_lat_lng(lat_lng: &WGS84<f64>) -> Self {
        // Implemented according to
        // https://developers.google.com/maps/documentation/javascript/examples/map-coordinates?csw=1
        // but clamping is done before the sin() operation.
        let lat = nalgebra::clamp(lat_lng.latitude(), -LAT_BOUND_RAD, LAT_BOUND_RAD);
        let sin_y = lat.sin();

        let xy = Vector2::new(
            0.5 + lat_lng.longitude() / TWO_PI,
            0.5 - ((1.0 + sin_y) / (1.0 - sin_y)).ln() * FOUR_PI_INV,
        );
        Self { xy }
    }

    /// Convert the Web Mercator coordinate back to lat/lng.
    ///
    /// The altitude returned is always 0.
    pub fn to_lat_lng(&self) -> WGS84<f64> {
        let centered = self.xy - Vector2::new(0.5, 0.5);

        let sin_term = (-centered.y * FOUR_PI).exp();

        // Note that sin_term = -(2/(sin(y)-1)) - 1
        let one_over_sin_y = (sin_term + 1.0) * -0.5;
        let mut sin_y = (1.0 / one_over_sin_y) + 1.0;
        sin_y = nalgebra::clamp(sin_y, -LAT_BOUND_SIN, LAT_BOUND_SIN);
        let longitude = nalgebra::clamp(centered.x * TWO_PI, -PI, PI);
        WGS84::new(sin_y.asin() * 180.0 / PI, longitude * 180.0 / PI, 0.0)
    }

    /// To use a Web Mercator coordinate, specify a zoom level in which it
    /// should be represented.
    /// Zoom level Z means the map coordinates are in the interval `[0, 256*2^Z)`
    /// in both dimensions, i.e. map resolution doubles at each zoom level.
    pub fn to_zoomed_coordinate(&self, z: u8) -> Vector2<f64> {
        debug_assert!(z <= MAX_ZOOM);
        // 256 * 2^z
        let zoom = f64::from(TILE_SIZE << z);
        zoom * self.xy
    }

    /// The inverse of [`to_zoomed_coordinate`](#method.to_zoomed_coordinate).
    pub fn from_zoomed_coordinate(coord: Vector2<f64>, z: u8) -> Self {
        debug_assert!(z <= MAX_ZOOM);
        // 256 * 2^z
        let zoom = f64::from(TILE_SIZE << z);
        Self { xy: coord / zoom }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::{assert_abs_diff_eq, assert_relative_eq};
    use nalgebra::Vector2;
    use nav_types::WGS84;

    /// LAT_BOUND_RAD converted to degrees
    const LAT_BOUND_DEG: f64 = 85.05112877980658936394320335239172;

    #[test]
    fn projection_corners() {
        // Checks that the corners of the map are at the expected coordinates
        let lat_lng_lower = WGS84::new(LAT_BOUND_DEG, -180.0, 0.0);
        let lat_lng_upper = WGS84::new(-LAT_BOUND_DEG, 180.0, 0.0);
        let lower_corner = WebMercatorCoord::from_lat_lng(&lat_lng_lower);
        let upper_corner = WebMercatorCoord::from_lat_lng(&lat_lng_upper);
        let lower_corner_truth = Vector2::new(0.0, 0.0);
        let upper_corner_truth = Vector2::new(256.0, 256.0);
        assert_abs_diff_eq!(
            upper_corner.to_zoomed_coordinate(0),
            upper_corner_truth,
            epsilon = 10e-10
        );
        assert_abs_diff_eq!(
            lower_corner.to_zoomed_coordinate(0),
            lower_corner_truth,
            epsilon = 10e-10
        );
    }

    #[test]
    fn projection_roundtrip() {
        // Checks that unprojection of a projection returns the original coordinate,
        // except for altitude, which is 0
        let test_coordinate = WGS84::new(37.407204, -122.147604, 1300.0);
        let projected = WebMercatorCoord::from_lat_lng(&test_coordinate);
        let unprojected = projected.to_lat_lng();
        assert_relative_eq!(test_coordinate.longitude(), unprojected.longitude());
        assert_relative_eq!(test_coordinate.latitude(), unprojected.latitude());
        assert_eq!(unprojected.altitude(), 0.0);
    }

    #[test]
    fn projection_ground_truth() {
        let test_coordinate = WGS84::new(37.407204, -122.147604, 0.0);
        // This test coordinate is at approx. pixel (165, 18) on this OSM tile at level 19:
        // https://a.tile.openstreetmap.org/19/84253/203324.png
        // So the pixel coordinate in a map at zoom level 19 (resolution 256*2^19) is this:
        let test_coordinate_web_mercator_zoomed_truth =
            Vector2::new(84253.0 * 256.0 + 165.0, 203324.0 * 256.0 + 18.0);
        let test_coordinate_web_mercator_zoomed =
            WebMercatorCoord::from_lat_lng(&test_coordinate).to_zoomed_coordinate(19);
        // This was from eyeballing, so we shouldn't expect more than 20px accuracy at zoom 19.
        assert_abs_diff_eq!(
            test_coordinate_web_mercator_zoomed,
            test_coordinate_web_mercator_zoomed_truth,
            epsilon = 20.0
        );
    }
}
