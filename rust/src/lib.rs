// use autocxx::prelude::*;
// use ffi::trajopt::{SwerveSolution, OptimalTrajectoryGenerator, SwervePathBuilder};

// include_cpp! {
//     #include "trajopt/path/SwervePathBuilder.h"
//     #include "trajopt/OptimalTrajectoryGenerator.h"
//     safety!(unsafe_ffi)
//     generate!("trajopt::SwervePathBuilder") // allowlist a function
//     generate!("trajopt::OptimalTrajectoryGenerator") // allowlist a function
// }

// pub fn my_ffi_func() -> f64 {
//     let mut path = SwervePathBuilder::new().within_unique_ptr();
//     path.pin_mut().PoseWpt(0, 0.0, 0.0, 0.0);
//     path.pin_mut().PoseWpt(1, 1.0, 0.0, 0.0);
//     let mut val = OptimalTrajectoryGenerator::Generate(&path).within_unique_ptr();
//     println!("{:?}", val.pin_mut());
//     1.0
// }

#[cxx::bridge(namespace = "trajoptlibrust")]
mod ffi {
    #[derive(Debug)]
    struct SwerveModule {
        x: f64,
        y: f64,
        wheel_radius: f64,
        wheel_max_angular_velocity: f64,
        wheel_max_torque: f64
    }
    #[derive(Debug)]
    struct SwerveDrivetrain {
        mass: f64,
        moi: f64,
        modules: Vec<SwerveModule>
    }

    #[derive(Debug)]
    struct HolonomicTrajectorySample {
        timestamp: f64,
        x: f64,
        y: f64,
        heading: f64,
        velocity_x: f64,
        velocity_y: f64,
        angular_velocity: f64
    }

    #[derive(Debug)]
    struct HolonomicTrajectory {
        samples: Vec<HolonomicTrajectorySample>
    }

    unsafe extern "C++" {
        include!("trajoptlib/include/trajoptlib.h");

        type SwervePathBuilderImpl;
        
        fn set_drivetrain(self: Pin<&mut SwervePathBuilderImpl>, drivetrain: &SwerveDrivetrain);
        
        fn pose_wpt(self: Pin<&mut SwervePathBuilderImpl>, idx: usize, x: f64, y: f64, heading: f64);
        
        fn wpt_zero_velocity(self: Pin<&mut SwervePathBuilderImpl>, idx: usize);
        fn wpt_zero_angular_velocity(self: Pin<&mut SwervePathBuilderImpl>, idx: usize);
        
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

    pub fn pose_wpt(&mut self, idx: usize, x: f64, y: f64, heading: f64) {
        crate::ffi::SwervePathBuilderImpl::pose_wpt(self.path.pin_mut(), idx, x, y, heading);
    }

    pub fn wpt_zero_velocity(&mut self, idx: usize) {
        crate::ffi::SwervePathBuilderImpl::wpt_zero_velocity(self.path.pin_mut(), idx);
    }

    pub fn wpt_zero_angular_velocity(&mut self, idx: usize) {
        crate::ffi::SwervePathBuilderImpl::wpt_zero_angular_velocity(self.path.pin_mut(), idx);
    }

    pub fn generate(&self) -> Result<HolonomicTrajectory, cxx::Exception> {
        self.path.generate()
    }
}

pub use ffi::*;