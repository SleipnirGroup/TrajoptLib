use trajoptlib::{SwervePathBuilder, SwerveDrivetrain, SwerveModule};

fn main() {
  let drivetrain = SwerveDrivetrain {
    mass: 45.0,
    moi: 6.0,
    modules: vec![
      SwerveModule {
        x: 0.6,
        y: 0.6,
        wheel_radius: 0.04,
        wheel_max_angular_velocity: 70.0,
        wheel_max_torque: 2.0
      },
      SwerveModule {
        x: 0.6,
        y: -0.6,
        wheel_radius: 0.04,
        wheel_max_angular_velocity: 70.0,
        wheel_max_torque: 2.0
      },
      SwerveModule {
        x: -0.6,
        y: 0.6,
        wheel_radius: 0.04,
        wheel_max_angular_velocity: 70.0,
        wheel_max_torque: 2.0
      },
      SwerveModule {
        x: -0.6,
        y: -0.6,
        wheel_radius: 0.04,
        wheel_max_angular_velocity: 70.0,
        wheel_max_torque: 2.0
      }
    ]
  };

  let mut path = SwervePathBuilder::new();
  path.set_drivetrain(&drivetrain);
  path.pose_wpt(0, 0.0, 0.0, 0.0);
  path.pose_wpt(1, 1.0, 0.0, 0.0);
  path.wpt_linear_velocity_polar(0, 0.0, 0.0);
  path.wpt_linear_velocity_polar(1, 0.0, 0.0);
  path.wpt_angular_velocity(0, 0.0);
  path.wpt_angular_velocity(1, 0.0);
  println!("{:?}", path.generate());
}
