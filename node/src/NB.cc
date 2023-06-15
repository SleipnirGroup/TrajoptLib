#include <napi.h>
#include <path/Path.h>
#include <path/SwervePathBuilder.h>
#include <OptimalTrajectoryGenerator.h>

Napi::String Method(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  using namespace trajopt;
  SwerveDrivetrain swerveDrivetrain{.mass = 45,
                                    .moi = 6,
                                    .modules = {{+0.6, +0.6, 0.04, 70, 2},
                                                {+0.6, -0.6, 0.04, 70, 2},
                                                {-0.6, +0.6, 0.04, 70, 2},
                                                {-0.6, -0.6, 0.04, 70, 2}}};

  // One Meter Forward
  SwervePathBuilder path;
  path.SetDrivetrain(swerveDrivetrain);
  path.PoseWpt(0, 0.0, 0.0, 0.0);
  path.PoseWpt(1, 5.0, 0.0, 0.0);
  path.WptZeroVelocity(0);
  path.WptZeroVelocity(1);
  path.ControlIntervalCounts({4});

  // SOLVE
  try {
    SwerveSolution solution = OptimalTrajectoryGenerator::Generate(path);
    fmt::print("[{}]\n", fmt::join(path.CalculateInitialGuess().x, ","));
    fmt::print("{}\n", solution);
    // fmt::print("{}\n", HolonomicTrajectory(solution));
  } catch (const std::exception& e) {
    fmt::print("{}", e.what());
  }
  
  return Napi::String::New(env, "world");
}

Napi::Object Init(Napi::Env env, Napi::Object exports) {
  exports.Set(Napi::String::New(env, "hello"),
              Napi::Function::New(env, Method));
  return exports;
}

NODE_API_MODULE(hello, Init)