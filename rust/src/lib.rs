#[cxx::bridge(namespace = "trajoptlibrust")]
mod ffi {
    #[derive(Debug, Deserialize, Serialize)]
    struct SwerveModule {
        x: f64,
        y: f64,
        wheel_radius: f64,
        wheel_max_angular_velocity: f64,
        wheel_max_torque: f64
    }

    #[derive(Debug, Deserialize, Serialize)]
    struct SwerveDrivetrain {
        mass: f64,
        moi: f64,
        modules: Vec<SwerveModule>
    }

    #[derive(Debug, Deserialize, Serialize)]
    struct HolonomicTrajectorySample {
        timestamp: f64,
        x: f64,
        y: f64,
        heading: f64,
        velocity_x: f64,
        velocity_y: f64,
        angular_velocity: f64
    }

    #[derive(Debug, Deserialize, Serialize)]
    struct HolonomicTrajectory {
        samples: Vec<HolonomicTrajectorySample>
    }

    unsafe extern "C++" {
        include!("trajoptlib/include/trajoptlib.h");

        type SwervePathBuilderImpl;

        fn set_drivetrain(self: Pin<&mut SwervePathBuilderImpl>, drivetrain: &SwerveDrivetrain);

        fn translation_wpt(self: Pin<&mut SwervePathBuilderImpl>, idx: usize, x: f64, y: f64, heading_guess: f64);
        fn pose_wpt(self: Pin<&mut SwervePathBuilderImpl>, idx: usize, x: f64, y: f64, heading: f64);

        fn wpt_zero_velocity(self: Pin<&mut SwervePathBuilderImpl>, idx: usize);
        fn wpt_zero_angular_velocity(self: Pin<&mut SwervePathBuilderImpl>, idx: usize);
        fn wpt_velocity_direction(self: Pin<&mut SwervePathBuilderImpl>, idx: usize, angle: f64);
        fn wpt_velocity_magnitude(self: Pin<&mut SwervePathBuilderImpl>, idx: usize, v: f64);
        fn wpt_velocity_polar(self: Pin<&mut SwervePathBuilderImpl>, idx: usize, vr: f64, vtheta: f64);
        fn wpt_angular_velocity(self: Pin<&mut SwervePathBuilderImpl>, idx: usize, angular_velocity: f64);

        fn generate(self: &SwervePathBuilderImpl) -> Result<HolonomicTrajectory>;

        fn new_swerve_path_builder_impl() -> UniquePtr<SwervePathBuilderImpl>;
    }
}


pub struct SwervePathBuilder {
    path: cxx::UniquePtr<crate::ffi::SwervePathBuilderImpl>
}

impl SwervePathBuilder {

    pub fn new() -> SwervePathBuilder {
        SwervePathBuilder {
            path: crate::ffi::new_swerve_path_builder_impl()
        }
    }

    pub fn set_drivetrain(&mut self, drivetrain: &crate::ffi::SwerveDrivetrain) {
        crate::ffi::SwervePathBuilderImpl::set_drivetrain(self.path.pin_mut(), drivetrain);
    }

    pub fn translation_wpt(&mut self, idx: usize, x: f64, y: f64, heading_guess: f64) {
        crate::ffi::SwervePathBuilderImpl::translation_wpt(self.path.pin_mut(), idx, x, y, heading_guess);
    }

    pub fn pose_wpt(&mut self, idx: usize, x: f64, y: f64, heading: f64) {
        crate::ffi::SwervePathBuilderImpl::pose_wpt(self.path.pin_mut(), idx, x, y, heading);
    }

    pub fn wpt_zero_velocity(&mut self, idx: usize) {
        crate::ffi::SwervePathBuilderImpl::wpt_zero_velocity(self.path.pin_mut(), idx);
    }

    pub fn wpt_zero_angular_velocity(&mut self, idx: usize) {
        crate::ffi::SwervePathBuilderImpl::wpt_zero_angular_velocity(self.path.pin_mut(), idx);
    }

    pub fn wpt_velocity_direction(&mut self, idx: usize, angle: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_velocity_direction(self.path.pin_mut(), idx, angle);
    }

    pub fn wpt_velocity_magnitude(&mut self, idx: usize, v: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_velocity_magnitude(self.path.pin_mut(), idx, v);
    }

    pub fn wpt_velocity_polar(&mut self, idx: usize, vr: f64, vtheta: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_velocity_polar(self.path.pin_mut(), idx, vr, vtheta);
    }

    pub fn wpt_angular_velocity(&mut self, idx: usize, angular_velocity: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_angular_velocity(self.path.pin_mut(), idx, angular_velocity);
    }

    pub fn generate(&self) -> Result<HolonomicTrajectory, String> {
        match self.path.generate() {
            Ok(traj) => Ok(traj),
            Err(msg) => Err(msg.what().to_string())
        }
    }
}

pub use ffi::HolonomicTrajectory;
pub use ffi::HolonomicTrajectorySample;
pub use ffi::SwerveDrivetrain;
pub use ffi::SwerveModule;
