// Copyright (c) TrajoptLib contributors

#pragma once

#include <napi.h>
#include <path/SwervePathBuilder.h>

class SwervePathBuilderWrap : public Napi::ObjectWrap<SwervePathBuilderWrap> {
 public:
  explicit SwervePathBuilderWrap(const Napi::CallbackInfo& info);

  Napi::Value SetDrivetrain(const Napi::CallbackInfo& info);
  Napi::Value PoseWpt(const Napi::CallbackInfo& info);
  Napi::Value WptZeroVelocity(const Napi::CallbackInfo& info);
  Napi::Value WptZeroAngularVelocity(const Napi::CallbackInfo& info);
  Napi::Value Generate(const Napi::CallbackInfo& info);

  static Napi::Function GetClass(const Napi::Env env);

 private:
  trajopt::SwervePathBuilder _path;
};
