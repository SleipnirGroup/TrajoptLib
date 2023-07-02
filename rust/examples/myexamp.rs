use trajoptlib::{SwervePathBuilder, SwerveDrivetrain, SwerveModule};

fn main() {
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
  path.pose_wpt(0, 0.0, 0.0, 0.0);
  path.pose_wpt(1, 1.0, 0.0, 0.0);
  path.wpt_zero_velocity(0);
  path.wpt_zero_velocity(1);
  path.wpt_zero_angular_velocity(0);
  path.wpt_zero_angular_velocity(1);
  println!("{:?}", path.generate());

}