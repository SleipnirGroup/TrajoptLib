use trajoptlib::{SwervePathBuilder, SwerveDrivetrain, SwerveModule};

fn main() {
  println!("{}", "no_impl_yet lol");

  let drivetrain = SwerveDrivetrain {
    mass: 45.0,
    moi: 20.0,
    modules: vec![
      SwerveModule {
        x: 1.0,
        y: 1.0,
        wheel_radius: 0.1,
        wheel_max_angular_velocity: 0.1,
        wheel_max_torque: 0.1
      },
      SwerveModule {
        x: -1.0,
        y: 1.0,
        wheel_radius: 0.1,
        wheel_max_angular_velocity: 0.1,
        wheel_max_torque: 0.1
      },
      SwerveModule {
        x: 1.0,
        y: -1.0,
        wheel_radius: 0.1,
        wheel_max_angular_velocity: 0.1,
        wheel_max_torque: 0.1
      },
      SwerveModule {
        x: -1.0,
        y: -1.0,
        wheel_radius: 0.1,
        wheel_max_angular_velocity: 0.1,
        wheel_max_torque: 0.1
      }
    ]
  };

  let mut path = SwervePathBuilder::new();
  path.set_drivetrain(&drivetrain);

}