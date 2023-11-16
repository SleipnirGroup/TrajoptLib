#[cxx::bridge(namespace = "trajoptlibrust")]
mod ffi {
    #[derive(Debug, Deserialize, Serialize)]
    struct SwerveModule {
        x: f64,
        y: f64,
        wheel_radius: f64,
        wheel_max_angular_velocity: f64,
        wheel_max_torque: f64,
    }

    #[derive(Debug, Deserialize, Serialize)]
    struct SwerveDrivetrain {
        mass: f64,
        moi: f64,
        modules: Vec<SwerveModule>,
    }

    #[derive(Debug, Deserialize, Serialize)]
    struct InitialGuessPoint {
        x: f64,
        y: f64,
        heading: f64,
    }

    #[derive(Debug, Deserialize, Serialize)]
    struct HolonomicTrajectorySample {
        timestamp: f64,
        x: f64,
        y: f64,
        heading: f64,
        velocity_x: f64,
        velocity_y: f64,
        angular_velocity: f64,
    }

    #[derive(Debug, Deserialize, Serialize)]
    struct HolonomicTrajectory {
        samples: Vec<HolonomicTrajectorySample>,
    }

    unsafe extern "C++" {
        include!("trajoptlib/include/trajoptlib.h");

        type SwervePathBuilderImpl;

        fn cancel_all(self: Pin<&mut SwervePathBuilderImpl>);
        fn set_drivetrain(self: Pin<&mut SwervePathBuilderImpl>, drivetrain: &SwerveDrivetrain);
        fn set_bumpers(self: Pin<&mut SwervePathBuilderImpl>, length: f64, width: f64);
        fn set_control_interval_counts(self: Pin<&mut SwervePathBuilderImpl>, counts: Vec<usize>);

        fn pose_wpt(
            self: Pin<&mut SwervePathBuilderImpl>,
            idx: usize,
            x: f64,
            y: f64,
            heading: f64,
        );
        fn translation_wpt(
            self: Pin<&mut SwervePathBuilderImpl>,
            idx: usize,
            x: f64,
            y: f64,
            heading_guess: f64,
        );
        fn empty_wpt(
            self: Pin<&mut SwervePathBuilderImpl>,
            idx: usize,
            x_guess: f64,
            y_guess: f64,
            heading_guess: f64,
        );

        fn sgmt_initial_guess_points(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_idx: usize,
            guess_points: &Vec<InitialGuessPoint>,
        );

        fn wpt_linear_velocity_direction(
            self: Pin<&mut SwervePathBuilderImpl>,
            idx: usize,
            angle: f64,
        );
        fn wpt_linear_velocity_max_magnitude(
            self: Pin<&mut SwervePathBuilderImpl>,
            idx: usize,
            magnitude: f64,
        );
        fn wpt_linear_velocity_polar(
            self: Pin<&mut SwervePathBuilderImpl>,
            idx: usize,
            magnitude: f64,
            angle: f64,
        );
        fn wpt_angular_velocity(
            self: Pin<&mut SwervePathBuilderImpl>,
            idx: usize,
            angular_velocity: f64,
        );
        fn wpt_x(self: Pin<&mut SwervePathBuilderImpl>, idx: usize, x: f64);
        fn wpt_y(self: Pin<&mut SwervePathBuilderImpl>, idx: usize, y: f64);
        fn wpt_heading(self: Pin<&mut SwervePathBuilderImpl>, idx: usize, heading: f64);

        fn sgmt_linear_velocity_direction(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_idx: usize,
            to_idx: usize,
            angle: f64,
        );
        fn sgmt_linear_velocity_max_magnitude(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_idx: usize,
            to_idx: usize,
            magnitude: f64,
        );
        fn sgmt_linear_velocity_polar(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_idx: usize,
            to_idx: usize,
            magnitude: f64,
            angle: f64,
        );
        fn sgmt_angular_velocity(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_idx: usize,
            to_idx: usize,
            angular_velocity: f64,
        );
        fn sgmt_x(self: Pin<&mut SwervePathBuilderImpl>, from_idx: usize, to_idx: usize, x: f64);
        fn sgmt_y(self: Pin<&mut SwervePathBuilderImpl>, from_idx: usize, to_idx: usize, y: f64);
        fn sgmt_heading(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_idx: usize,
            to_idx: usize,
            heading: f64,
        );

        fn sgmt_circle_obstacle(
            self: Pin<&mut SwervePathBuilderImpl>,
            from_idx: usize,
            to_idx: usize,
            x: f64,
            y: f64,
            radius: f64,
        );

        fn generate(self: &SwervePathBuilderImpl) -> Result<HolonomicTrajectory>;

        fn new_swerve_path_builder_impl() -> UniquePtr<SwervePathBuilderImpl>;
    }
}

pub struct SwervePathBuilder {
    path: cxx::UniquePtr<crate::ffi::SwervePathBuilderImpl>,
}

impl SwervePathBuilder {
    pub fn new() -> SwervePathBuilder {
        SwervePathBuilder {
            path: crate::ffi::new_swerve_path_builder_impl(),
        }
    }

    pub fn set_drivetrain(&mut self, drivetrain: &crate::ffi::SwerveDrivetrain) {
        crate::ffi::SwervePathBuilderImpl::set_drivetrain(self.path.pin_mut(), drivetrain);
    }

    pub fn set_bumpers(&mut self, length: f64, width: f64) {
        crate::ffi::SwervePathBuilderImpl::set_bumpers(self.path.pin_mut(), length, width);
    }

    pub fn set_control_interval_counts(&mut self, counts: Vec<usize>) {
        crate::ffi::SwervePathBuilderImpl::set_control_interval_counts(self.path.pin_mut(), counts);
    }

    pub fn pose_wpt(&mut self, idx: usize, x: f64, y: f64, heading: f64) {
        crate::ffi::SwervePathBuilderImpl::pose_wpt(self.path.pin_mut(), idx, x, y, heading);
    }

    pub fn translation_wpt(&mut self, idx: usize, x: f64, y: f64, heading_guess: f64) {
        crate::ffi::SwervePathBuilderImpl::translation_wpt(
            self.path.pin_mut(),
            idx,
            x,
            y,
            heading_guess,
        );
    }

    pub fn empty_wpt(&mut self, idx: usize, x_guess: f64, y_guess: f64, heading_guess: f64) {
        crate::ffi::SwervePathBuilderImpl::empty_wpt(
            self.path.pin_mut(),
            idx,
            x_guess,
            y_guess,
            heading_guess,
        );
    }

    pub fn sgmt_initial_guess_points(
        &mut self,
        from_idx: usize,
        guess_points: &Vec<crate::ffi::InitialGuessPoint>,
    ) {
        crate::ffi::SwervePathBuilderImpl::sgmt_initial_guess_points(
            self.path.pin_mut(),
            from_idx,
            guess_points,
        );
    }

    pub fn wpt_linear_velocity_direction(&mut self, idx: usize, angle: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_linear_velocity_direction(
            self.path.pin_mut(),
            idx,
            angle,
        );
    }

    pub fn wpt_linear_velocity_max_magnitude(&mut self, idx: usize, magnitude: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_linear_velocity_max_magnitude(
            self.path.pin_mut(),
            idx,
            magnitude,
        );
    }

    pub fn wpt_linear_velocity_polar(&mut self, idx: usize, magnitude: f64, angle: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_linear_velocity_polar(
            self.path.pin_mut(),
            idx,
            magnitude,
            angle,
        );
    }

    pub fn wpt_angular_velocity(&mut self, idx: usize, angular_velocity: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_angular_velocity(
            self.path.pin_mut(),
            idx,
            angular_velocity,
        );
    }

    pub fn wpt_x(&mut self, idx: usize, x: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_x(self.path.pin_mut(), idx, x);
    }

    pub fn wpt_y(&mut self, idx: usize, y: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_y(self.path.pin_mut(), idx, y);
    }

    pub fn wpt_heading(&mut self, idx: usize, heading: f64) {
        crate::ffi::SwervePathBuilderImpl::wpt_heading(self.path.pin_mut(), idx, heading);
    }

    pub fn sgmt_linear_velocity_direction(&mut self, from_idx: usize, to_idx: usize, angle: f64) {
        crate::ffi::SwervePathBuilderImpl::sgmt_linear_velocity_direction(
            self.path.pin_mut(),
            from_idx,
            to_idx,
            angle,
        );
    }

    pub fn sgmt_linear_velocity_max_magnitude(
        &mut self,
        from_idx: usize,
        to_idx: usize,
        magnitude: f64,
    ) {
        crate::ffi::SwervePathBuilderImpl::sgmt_linear_velocity_max_magnitude(
            self.path.pin_mut(),
            from_idx,
            to_idx,
            magnitude,
        );
    }

    pub fn sgmt_linear_velocity_polar(
        &mut self,
        from_idx: usize,
        to_idx: usize,
        magnitude: f64,
        angle: f64,
    ) {
        crate::ffi::SwervePathBuilderImpl::sgmt_linear_velocity_polar(
            self.path.pin_mut(),
            from_idx,
            to_idx,
            magnitude,
            angle,
        );
    }

    pub fn sgmt_angular_velocity(&mut self, from_idx: usize, to_idx: usize, angular_velocity: f64) {
        crate::ffi::SwervePathBuilderImpl::sgmt_angular_velocity(
            self.path.pin_mut(),
            from_idx,
            to_idx,
            angular_velocity,
        );
    }

    pub fn sgmt_x(&mut self, from_idx: usize, to_idx: usize, x: f64) {
        crate::ffi::SwervePathBuilderImpl::sgmt_x(self.path.pin_mut(), from_idx, to_idx, x);
    }

    pub fn sgmt_y(&mut self, from_idx: usize, to_idx: usize, y: f64) {
        crate::ffi::SwervePathBuilderImpl::sgmt_y(self.path.pin_mut(), from_idx, to_idx, y);
    }

    pub fn sgmt_heading(&mut self, from_idx: usize, to_idx: usize, heading: f64) {
        crate::ffi::SwervePathBuilderImpl::sgmt_heading(
            self.path.pin_mut(),
            from_idx,
            to_idx,
            heading,
        );
    }

    pub fn sgmt_circle_obstacle(
        &mut self,
        from_idx: usize,
        to_idx: usize,
        x: f64,
        y: f64,
        radius: f64,
    ) {
        crate::ffi::SwervePathBuilderImpl::sgmt_circle_obstacle(
            self.path.pin_mut(),
            from_idx,
            to_idx,
            x,
            y,
            radius,
        );
    }

    pub fn generate(&self) -> Result<HolonomicTrajectory, String> {
        match self.path.generate() {
            Ok(traj) => Ok(traj),
            Err(msg) => Err(msg.what().to_string()),
        }
    }

    pub fn cancel_all(&mut self) {
        crate::ffi::SwervePathBuilderImpl::cancel_all(self.path.pin_mut());
    }
}

pub use ffi::HolonomicTrajectory;
pub use ffi::HolonomicTrajectorySample;
pub use ffi::InitialGuessPoint;
pub use ffi::SwerveDrivetrain;
pub use ffi::SwerveModule;
