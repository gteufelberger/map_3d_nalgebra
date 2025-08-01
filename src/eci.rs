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
