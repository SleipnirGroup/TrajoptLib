#include "SwervePathBuilderWrap.h"

#include <cmath>

#include <OptimalTrajectoryGenerator.h>
#include <drivetrain/SwerveDrivetrain.h>
#include <solution/SwerveSolution.h>
#include <trajectory/HolonomicTrajectory.h>
#include <napi.h>
#include <path/SwervePathBuilder.h>

SwervePathBuilderWrap::SwervePathBuilderWrap(const Napi::CallbackInfo& info) : ObjectWrap(info) {
}

trajopt::SwerveDrivetrain _ToSwerveDrivetrain(const Napi::Object& napiSwerveDrivetrain) {
  double mass = napiSwerveDrivetrain.Get("mass").ToNumber().DoubleValue();
  double moi = napiSwerveDrivetrain.Get("moi").ToNumber().DoubleValue();
  Napi::Array napiSwerveModules = napiSwerveDrivetrain.Get("modules").As<Napi::Array>();
  std::vector<trajopt::SwerveModule> swerveModules;
  swerveModules.reserve(napiSwerveModules.Length());
  for (size_t idx = 0; idx < napiSwerveModules.Length(); ++idx) {
    Napi::Object napiSwerveModule = napiSwerveModules.Get(idx).ToObject();
    swerveModules.emplace_back(trajopt::SwerveModule{
      napiSwerveModule.Get("x").ToNumber().DoubleValue(),
      napiSwerveModule.Get("y").ToNumber().DoubleValue(),
      napiSwerveModule.Get("wheelRadius").ToNumber().DoubleValue(),
      napiSwerveModule.Get("wheelMaxAngularVelocity").ToNumber().DoubleValue(),
      napiSwerveModule.Get("wheelMaxTorque").ToNumber().DoubleValue()});
  }
  return trajopt::SwerveDrivetrain{mass, moi, std::move(swerveModules)};
}


Napi::Value SwervePathBuilderWrap::SetDrivetrain(const Napi::CallbackInfo &info) {
  if (info.Length() != 1 || !info[0].IsObject()) {
    Napi::TypeError::New(info.Env(), "SwervePathBuilder.setDrivetrain() accepts exactly one object parameter.").ThrowAsJavaScriptException();
    return info.Env().Undefined();
  }
  auto swerveDrivetrain = _ToSwerveDrivetrain(info[0].ToObject());
  _path.SetDrivetrain(swerveDrivetrain);

  return info.Env().Undefined();
}

Napi::Value SwervePathBuilderWrap::PoseWpt(const Napi::CallbackInfo &info) {
  if (info.Length() != 4 ||
      !info[0].IsNumber() ||
      !info[1].IsNumber() ||
      !info[2].IsNumber() ||
      !info[3].IsNumber()) {
    Napi::TypeError::New(info.Env(), "SwervePathBuilder.poseWpt() accepts exactly four number parameters").ThrowAsJavaScriptException();
    return info.Env().Undefined();
  }
  uint32_t idx = info[0].ToNumber().Uint32Value();
  double x = info[1].ToNumber().DoubleValue();
  double y = info[2].ToNumber().DoubleValue();
  double heading = info[3].ToNumber().DoubleValue();
  _path.PoseWpt(idx, x, y, heading);

  return info.Env().Undefined();
}

Napi::Value SwervePathBuilderWrap::WptZeroVelocity(const Napi::CallbackInfo &info) {
  if (info.Length() != 1 ||
      !info[0].IsNumber()) {
    Napi::TypeError::New(info.Env(), "SwervePathBuilder.wptZeroVelocity() accepts exactly one number parameter").ThrowAsJavaScriptException();
    return info.Env().Undefined();
  }
  uint32_t idx = info[0].ToNumber().Uint32Value();
  _path.WptZeroVelocity(idx);

  return info.Env().Undefined();
}

Napi::Value SwervePathBuilderWrap::WptZeroAngularVelocity(const Napi::CallbackInfo &info) {
  if (info.Length() != 1 ||
      !info[0].IsNumber()) {
    Napi::TypeError::New(info.Env(), "SwervePathBuilder.wptZeroAngularVelocity() accepts exactly one number parameter").ThrowAsJavaScriptException();
    return info.Env().Undefined();
  }
  uint32_t idx = info[0].ToNumber().Uint32Value();
  _path.WptZeroAngularVelocity(idx);

  return info.Env().Undefined();
}

Napi::Value SwervePathBuilderWrap::Generate(const Napi::CallbackInfo &info) {
  if (info.Length() != 0) {
    Napi::TypeError::New(info.Env(), "SwervePathBuilder.generate() accepts exactly one number parameter").ThrowAsJavaScriptException();
    return info.Env().Undefined();
  }
  trajopt::SwerveSolution solution = trajopt::OptimalTrajectoryGenerator::Generate(_path);
  trajopt::HolonomicTrajectory trajectory{solution};
  auto napiSamples = Napi::Array::New(info.Env());
  size_t sampCnt = trajectory.samples.size();
  for (size_t idx = 0; idx < sampCnt; ++idx) {
    Napi::Object napiSample = Napi::Object::New(info.Env());
    napiSample.Set("timestamp", trajectory.samples.at(idx).timestamp);
    napiSample.Set("x", trajectory.samples.at(idx).x);
    napiSample.Set("y", trajectory.samples.at(idx).y);
    napiSample.Set("heading", trajectory.samples.at(idx).heading);
    napiSample.Set("velocityX", trajectory.samples.at(idx).velocityX);
    napiSample.Set("velocityY", trajectory.samples.at(idx).velocityY);
    napiSample.Set("angularVelocity", trajectory.samples.at(idx).angularVelocity);
    napiSamples.Set(idx, napiSample);
  }
  return napiSamples;
}

Napi::Function SwervePathBuilderWrap::GetClass(const Napi::Env env) {
  return DefineClass(
      env,
      "SwervePathBuilder",
      {
          SwervePathBuilderWrap::InstanceMethod("setDrivetrain", &SwervePathBuilderWrap::SetDrivetrain),
          SwervePathBuilderWrap::InstanceMethod("poseWpt", &SwervePathBuilderWrap::PoseWpt),
          SwervePathBuilderWrap::InstanceMethod("wptZeroVelocity", &SwervePathBuilderWrap::WptZeroVelocity),
          SwervePathBuilderWrap::InstanceMethod("wptZeroAngularVelocity", &SwervePathBuilderWrap::WptZeroAngularVelocity),
          SwervePathBuilderWrap::InstanceMethod("generate", &SwervePathBuilderWrap::Generate),
      });
}
