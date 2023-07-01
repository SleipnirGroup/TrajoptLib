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

#[cxx::bridge]
mod ffi {
    struct SwerveModule {
        x: f64,
        y: f64,
        wheel_radius: f64,
        wheel_max_angular_velocity: f64,
        wheel_max_torque: f64
    }
    struct SwerveDrivetrain {
        mass: f64,
        moi: f64,
        modules: Vec<SwerveModule>
    }

    extern "C++" {
        
    }
}