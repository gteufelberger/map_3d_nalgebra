use approx::assert_relative_eq;
use map_3d_nalgebra::{ecef, eci};
use nalgebra::Vector3;
use rand::Rng;

#[test]
fn test_eci_to_ecef_and_back_vector() {
    // Set up random number gen
    let mut rng = rand::rng();

    // Get GST and ECEF coordinates
    let gst: f64 = rng.random();
    let x = rng.random();
    let y = rng.random();
    let z = rng.random();
    let eci_coords = Vector3::new(x, y, z);

    // Convert back from ECI to ECEF and back
    let eci_coords_back = ecef::to_eci(gst, eci::to_ecef(gst, eci_coords));

    assert_relative_eq!(eci_coords, eci_coords_back, epsilon = 1e-99999);
}
