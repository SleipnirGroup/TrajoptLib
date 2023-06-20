var trajopt = require('bindings')('trajoptlib-node');

let drive = {
  mass: 45,
  moi: 6,
  modules: [
    {
      x: 0.6,
      y: 0.6,
      wheelRadius: 0.04,
      wheelMaxAngularVelocity: 70,
      wheelMaxTorque: 2
    },
    {
      x: 0.6,
      y: -0.6,
      wheelRadius: 0.04,
      wheelMaxAngularVelocity: 70,
      wheelMaxTorque: 2
    },
    {
      x: -0.6,
      y: 0.6,
      wheelRadius: 0.04,
      wheelMaxAngularVelocity: 70,
      wheelMaxTorque: 2
    },
    {
      x: -0.6,
      y: -0.6,
      wheelRadius: 0.04,
      wheelMaxAngularVelocity: 70,
      wheelMaxTorque: 2
    }
  ]
}

let path = new trajopt.SwervePathBuilder();

path.setDrivetrain(drive);
path.poseWpt(0, 0.0, 0.0, 0.0);
path.poseWpt(1, 5.0, 0.0, 0.0);
path.wptZeroVelocity(0);
path.wptZeroVelocity(1);

console.log(path.generate()); // 'world'
