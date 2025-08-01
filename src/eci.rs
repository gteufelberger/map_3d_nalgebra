use nalgebra::{Matrix3, Rotation3, Vector3};

/// Convert ECI coordinates to ECEF
/// ECI coordinates are rotated around the Z-axis by the GST angle.
pub fn to_ecef<T>(gst: f64, eci: T) -> T
where
    Matrix3<f64>: std::ops::Mul<T, Output = T>,
{
    let rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), -gst);
    *rotation.matrix() * eci
}

#[cfg(test)]
mod tests {
    use super::*;
    use map_3d;
    use rand::{self, Rng};

    /// Check against `map_3d` libary
    #[test]
    fn test_to_ecef_map_3d() {
        // Set up random number gen
        let mut rng = rand::rng();

        // Get GST and ECEF coordinates
        let gst: f64 = rng.random();
        let x = rng.random();
        let y = rng.random();
        let z = rng.random();
        let ecef = Vector3::new(x, y, z);

        // Convert
        let result = to_ecef(gst, ecef);

        // Use `map_3d` as refernce
        let result_map_3d = map_3d::eci2ecef(gst, x, y, z);
        let result_map_3d = Vector3::new(result_map_3d.0, result_map_3d.1, result_map_3d.2);

        // Compare
        assert_eq!(result, result_map_3d);
    }

    /// Test matrices as input for `to_ecef` function
    #[test]
    fn test_to_ecef_matrix() {
        let gst = 1.0;
        let ecef = Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        let expected_result = Matrix3::new(
            0.5403023058681398,
            0.8414709848078965,
            0.0,
            -0.8414709848078965,
            0.5403023058681398,
            0.0,
            0.0,
            0.0,
            1.0,
        );
        let result = to_ecef(gst, ecef);
        assert_eq!(result, expected_result);
    }
}
