use nalgebra::Vector3;

/// Convert Geodetic coordinates to ECEF
pub fn to_ecef(
    lat: f64,
    lon: f64,
    alt: f64,
    semi_major_axis: f64,
    flattening: f64,
) -> Vector3<f64> {
    let eccentricity_squared = flattening * (2.0 - flattening);
    let n = semi_major_axis / (1.0 - eccentricity_squared * lat.sin() * lat.sin()).sqrt();

    let x = (n + alt) * lat.cos() * lon.cos();
    let y = (n + alt) * lat.cos() * lon.sin();
    let z = (n * (1.0 - eccentricity_squared) + alt) * lat.sin();

    Vector3::new(x, y, z)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use map_3d;
    use rand::{self, Rng};

    /// Check against `map_3d` libary
    #[test]
    fn test_to_ecef_map_3d() {
        // Set up random number gen
        let mut rng = rand::rng();

        // Get GST and ECEF coordinates
        let lat: f64 = rng.random();
        let lon = rng.random();
        let alt = rng.random();
        let (semi_major_axis, flattening) = (6378137.0, 1.0 / 298.257223563); // WGS84

        // Convert
        let result = to_ecef(lat, lon, alt, semi_major_axis, flattening);

        // Use `map_3d` as refernce
        let result_map_3d = map_3d::geodetic2ecef(lat, lon, alt, map_3d::Ellipsoid::WGS84);
        let result_map_3d = Vector3::new(result_map_3d.0, result_map_3d.1, result_map_3d.2);

        // Compare
        assert_relative_eq!(result, result_map_3d, epsilon = 1e-9);
    }
}
