// Copyright (c) TrajoptLib contributors

#include <algorithm>
#include <cmath>
#include <memory>
#include <tuple>

#include "trajopt/InvalidPathException.h"
#include "trajopt/OptimalTrajectoryGenerator.h"
#include "trajopt/TrajectoryGenerationException.h"
#include "trajopt/constraint/AngularVelocityConstraint.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/constraint/HeadingConstraint.h"
#include "trajopt/constraint/holonomic/HolonomicConstraint.h"
#include "trajopt/constraint/holonomic/HolonomicVelocityConstraint.h"
#include "trajopt/drivetrain/SwerveDrivetrain.h"
#include "trajopt/drivetrain/SwerveModule.h"
#include "jni.h"
#include "trajopt/obstacle/Obstacle.h"
#include "trajopt/obstacle/ObstaclePoint.h"
#include "org_sleipnirgroup_trajopt_OptimalTrajectoryGenerator.h"
#include "trajopt/path/InitialGuessPoint.h"
#include "trajopt/path/Path.h"
#include "trajopt/path/SwervePathBuilder.h"
#include "trajopt/set/EllipticalSet2d.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/set/RectangularSet2d.h"
#include "trajopt/trajectory/HolonomicTrajectory.h"

using namespace trajopt;

template <typename Type, Type (*converter)(JNIEnv* env, jobject jObject)>
std::vector<Type> vectorFromJList(JNIEnv* env, jobject jList) {
  jclass jListClass = env->FindClass("java/util/List");
  jmethodID jListClassSizeMethod = env->GetMethodID(jListClass, "size", "()I");
  jmethodID jListClassGetMethod =
      env->GetMethodID(jListClass, "get", "(I)Ljava/lang/Object;");

  jint jListSize = env->CallIntMethod(jList, jListClassSizeMethod);
  std::vector<Type> vect;
  vect.reserve(jListSize);
  for (size_t index = 0; index < jListSize; index++) {
    jobject jListItem =
        env->CallObjectMethod(jList, jListClassGetMethod, index);
    vect.push_back(converter(env, jListItem));
  }

  return vect;
}

ObstaclePoint obstaclePointFromJObstaclePoint(JNIEnv* env,
                                              jobject jObstaclePoint) {
  jclass jObstaclePointClass =
      env->FindClass("org/sleipnirgroup/trajopt/ObstaclePoint");
  jfieldID jObstaclePointClassXField =
      env->GetFieldID(jObstaclePointClass, "x", "D");
  jfieldID jObstaclePointClassYField =
      env->GetFieldID(jObstaclePointClass, "y", "D");

  jdouble jObstaclePointX =
      env->GetDoubleField(jObstaclePoint, jObstaclePointClassXField);
  jdouble jObstaclePointY =
      env->GetDoubleField(jObstaclePoint, jObstaclePointClassYField);

  return ObstaclePoint{jObstaclePointX, jObstaclePointY};
}

Obstacle obstacleFromJObstacle(JNIEnv* env, jobject jObstacle) {
  jclass jListClass = env->FindClass("java/util/List");
  jmethodID jListClassSizeMethod = env->GetMethodID(jListClass, "size", "()I");

  jclass jObstacleClass = env->FindClass("org/sleipnirgroup/trajopt/Obstacle");
  jfieldID jObstacleClassSafetyDistanceField =
      env->GetFieldID(jObstacleClass, "safetyDistance", "D");
  jfieldID jObstacleClassPointsField =
      env->GetFieldID(jObstacleClass, "points", "Ljava/util/List;");

  jdouble jObstacleSafetyDistance =
      env->GetDoubleField(jObstacle, jObstacleClassSafetyDistanceField);
  jobject jObstaclePoints =
      env->GetObjectField(jObstacle, jObstacleClassPointsField);
  jint jObstaclePointsSize =
      env->CallIntMethod(jObstaclePoints, jListClassSizeMethod);

  std::vector<ObstaclePoint> obstaclePoints =
      vectorFromJList<ObstaclePoint, obstaclePointFromJObstaclePoint>(
          env,
          jObstaclePoints);  // <-- i love templates

  return Obstacle{jObstacleSafetyDistance, obstaclePoints};
}

InitialGuessPoint initialGuessPointFromJInitialGuessPoint(
    JNIEnv* env, jobject jInitialGuessPoint) {
  jclass jInitialGuessPointClass =
      env->FindClass("org/sleipnirgroup/trajopt/InitialGuessPoint");
  jfieldID jInitialGuessPointClassXField =
      env->GetFieldID(jInitialGuessPointClass, "x", "D");
  jfieldID jInitialGuessPointClassYField =
      env->GetFieldID(jInitialGuessPointClass, "y", "D");
  jfieldID jInitialGuessPointClassHeadingField =
      env->GetFieldID(jInitialGuessPointClass, "heading", "D");

  jdouble jInitialGuessPointX =
      env->GetDoubleField(jInitialGuessPoint, jInitialGuessPointClassXField);
  jdouble jInitialGuessPointY =
      env->GetDoubleField(jInitialGuessPoint, jInitialGuessPointClassYField);
  jdouble jInitialGuessPointHeading = env->GetDoubleField(
      jInitialGuessPoint, jInitialGuessPointClassHeadingField);

  return InitialGuessPoint{jInitialGuessPointX, jInitialGuessPointY,
                           jInitialGuessPointHeading};
}

SwerveModule swerveModuleFromJSwerveModule(JNIEnv* env, jobject jSwerveModule) {
  jclass jSwerveModuleClass =
      env->FindClass("org/sleipnirgroup/trajopt/SwerveModule");
  jfieldID jSwerveModuleClassXField =
      env->GetFieldID(jSwerveModuleClass, "x", "D");
  jfieldID jSwerveModuleClassYField =
      env->GetFieldID(jSwerveModuleClass, "y", "D");
  jfieldID jSwerveModuleClassWheelRadiusField =
      env->GetFieldID(jSwerveModuleClass, "wheelRadius", "D");
  jfieldID jSwerveModuleClassWheelMaxAngularVelocityField =
      env->GetFieldID(jSwerveModuleClass, "wheelMaxAngularVelocity", "D");
  jfieldID jSwerveModuleClassWheelMaxTorqueField =
      env->GetFieldID(jSwerveModuleClass, "wheelMaxTorque", "D");

  jdouble jSwerveModuleX =
      env->GetDoubleField(jSwerveModule, jSwerveModuleClassXField);
  jdouble jSwerveModuleY =
      env->GetDoubleField(jSwerveModule, jSwerveModuleClassYField);
  jdouble jSwerveModuleWheelRadius =
      env->GetDoubleField(jSwerveModule, jSwerveModuleClassWheelRadiusField);
  jdouble jSwerveModuleWheelMaxAngularVelocity = env->GetDoubleField(
      jSwerveModule, jSwerveModuleClassWheelMaxAngularVelocityField);
  jdouble jSwerveModuleWheelMaxTorque =
      env->GetDoubleField(jSwerveModule, jSwerveModuleClassWheelMaxTorqueField);

  return SwerveModule{jSwerveModuleX, jSwerveModuleY, jSwerveModuleWheelRadius,
                      jSwerveModuleWheelMaxAngularVelocity,
                      jSwerveModuleWheelMaxTorque};
}

SwerveDrivetrain swerveDrivetrainFromJSwerveDrivetrain(
    JNIEnv* env, jobject jSwerveDrivetrain) {
  jclass jSwerveDrivetrainClass =
      env->FindClass("org/sleipnirgroup/trajopt/SwerveDrivetrain");
  jfieldID jSwerveDrivetrainClassMassField =
      env->GetFieldID(jSwerveDrivetrainClass, "mass", "D");
  jfieldID jSwerveDrivetrainClassMomentOfInertiaField =
      env->GetFieldID(jSwerveDrivetrainClass, "momentOfInertia", "D");
  jfieldID jSwerveDrivetrainClassModulesField =
      env->GetFieldID(jSwerveDrivetrainClass, "modules", "Ljava/util/List;");
  jfieldID jSwerveDrivetrainClassBumpersField =
      env->GetFieldID(jSwerveDrivetrainClass, "bumpers",
                      "Lorg/sleipnirgroup/trajopt/Obstacle;");

  jdouble jSwerveDrivetrainMass =
      env->GetDoubleField(jSwerveDrivetrain, jSwerveDrivetrainClassMassField);
  jdouble jSwerveDrivetrainMomentOfInertia = env->GetDoubleField(
      jSwerveDrivetrain, jSwerveDrivetrainClassMomentOfInertiaField);
  jobject jSwerveDrivetrainBumpers = env->GetObjectField(
      jSwerveDrivetrain, jSwerveDrivetrainClassBumpersField);
  jobject jSwerveDrivetrainModules = env->GetObjectField(
      jSwerveDrivetrain, jSwerveDrivetrainClassModulesField);

  Obstacle bumpers = obstacleFromJObstacle(env, jSwerveDrivetrainBumpers);

  std::vector<SwerveModule> modules =
      vectorFromJList<SwerveModule, swerveModuleFromJSwerveModule>(
          env, jSwerveDrivetrainModules);

  return SwerveDrivetrain{jSwerveDrivetrainMass,
                          jSwerveDrivetrainMomentOfInertia, modules};
}

std::vector<HolonomicConstraint> poseConstraintsFrom(double x, double y,
                                                     double heading,
                                                     bool xConstrained,
                                                     bool yConstrained,
                                                     bool thetaConstrained) {
  std::vector<HolonomicConstraint> constrs;
  if (thetaConstrained) {
    constrs.emplace_back(HeadingConstraint{heading});
  }
  if (xConstrained && yConstrained) {
    constrs.emplace_back(TranslationConstraint{RectangularSet2d{x, y}});
  } else if (xConstrained) {
    constrs.emplace_back(
        TranslationConstraint{RectangularSet2d{x, IntervalSet1d::R1()}});
  } else if (yConstrained) {
    constrs.emplace_back(
        TranslationConstraint{RectangularSet2d{IntervalSet1d::R1(), y}});
  }
  return constrs;
}

std::vector<HolonomicConstraint> velocityConstraintsFrom(
    double velocityX, double velocityY, double angularVelocity,
    bool vxConstrained, bool vyConstrained, bool vMagnitudeConstrained,
    bool vAngularVelocityConstrained) {
  std::vector<HolonomicConstraint> constrs;
  if (vAngularVelocityConstrained) {
    constrs.emplace_back(AngularVelocityConstraint{angularVelocity});
  }
  using enum CoordinateSystem;
  /*if (!vMagnitudeConstrained && !vxConstrained && !vyConstrained) {
    // pass
  } else if (!vMagnitudeConstrained && !vxConstrained && vyConstrained) {
    // pass
  } else if (!vMagnitudeConstrained && vxConstrained && !vyConstrained) {
    // pass
  } else */
  if (!vMagnitudeConstrained && vxConstrained && vyConstrained) {
    double velocityAngle = std::atan2(velocityY, velocityX);
    constrs.emplace_back(
        HolonomicVelocityConstraint{LinearSet2d{velocityAngle}, kField});
  } else if (vMagnitudeConstrained && !vxConstrained && !vyConstrained) {
    double magnitude = std::hypot(velocityX, velocityY);
    constrs.emplace_back(HolonomicVelocityConstraint{
        EllipticalSet2d::CircularSet2d(magnitude), kField});
  } else if (vMagnitudeConstrained && !vxConstrained && vyConstrained) {
    constrs.emplace_back(HolonomicVelocityConstraint{
        RectangularSet2d{IntervalSet1d::R1(), velocityY}, kField});
  } else if (vMagnitudeConstrained && vxConstrained && !vyConstrained) {
    constrs.emplace_back(HolonomicVelocityConstraint{
        RectangularSet2d{velocityX, IntervalSet1d::R1()}, kField});
  } else if (vMagnitudeConstrained && vxConstrained && vyConstrained) {
    constrs.emplace_back(HolonomicVelocityConstraint{
        RectangularSet2d{velocityX, velocityY}, kField});
  }
  return constrs;
}

std::tuple<std::vector<HolonomicConstraint>, std::vector<InitialGuessPoint>,
           size_t>
holonomicConstraintsFromJHolonomicWaypoint(JNIEnv* env,
                                           jobject jHolonomicWaypoint) {
  jclass jHolonomicWaypointClass =
      env->FindClass("org/sleipnirgroup/trajopt/HolonomicWaypoint");
  jfieldID jHolonomicWaypointClassXField =
      env->GetFieldID(jHolonomicWaypointClass, "x", "D");
  jfieldID jHolonomicWaypointClassYField =
      env->GetFieldID(jHolonomicWaypointClass, "y", "D");
  jfieldID jHolonomicWaypointClassHeadingField =
      env->GetFieldID(jHolonomicWaypointClass, "heading", "D");
  jfieldID jHolonomicWaypointClassVelocityXField =
      env->GetFieldID(jHolonomicWaypointClass, "velocityX", "D");
  jfieldID jHolonomicWaypointClassVelocityYField =
      env->GetFieldID(jHolonomicWaypointClass, "velocityY", "D");
  jfieldID jHolonomicWaypointClassAngularVelocityField =
      env->GetFieldID(jHolonomicWaypointClass, "angularVelocity", "D");
  jfieldID jHolonomicWaypointClassXConstrainedField =
      env->GetFieldID(jHolonomicWaypointClass, "xConstrained", "Z");
  jfieldID jHolonomicWaypointClassYConstrainedField =
      env->GetFieldID(jHolonomicWaypointClass, "yConstrained", "Z");
  jfieldID jHolonomicWaypointClassHeadingConstrainedField =
      env->GetFieldID(jHolonomicWaypointClass, "headingConstrained", "Z");
  jfieldID jHolonomicWaypointClassVelocityXConstrainedField =
      env->GetFieldID(jHolonomicWaypointClass, "velocityXConstrained", "Z");
  jfieldID jHolonomicWaypointClassVelocityYConstrainedField =
      env->GetFieldID(jHolonomicWaypointClass, "velocityYConstrained", "Z");
  jfieldID jHolonomicWaypointClassVelocityMagnitudeConstrainedField =
      env->GetFieldID(jHolonomicWaypointClass, "velocityMagnitudeConstrained",
                      "Z");
  jfieldID jHolonomicWaypointClassAngularVelocityConstrainedField =
      env->GetFieldID(jHolonomicWaypointClass, "angularVelocityConstrained",
                      "Z");
  jfieldID jHolonomicWaypointClassControlIntervalCountField =
      env->GetFieldID(jHolonomicWaypointClass, "controlIntervalCount", "I");
  jfieldID jHolonomicWaypointClassInitialGuessPointsField = env->GetFieldID(
      jHolonomicWaypointClass, "initialGuessPoints", "Ljava/util/List;");

  jdouble jHolonomicWaypointX =
      env->GetDoubleField(jHolonomicWaypoint, jHolonomicWaypointClassXField);
  jdouble jHolonomicWaypointY =
      env->GetDoubleField(jHolonomicWaypoint, jHolonomicWaypointClassYField);
  jdouble jHolonomicWaypointHeading = env->GetDoubleField(
      jHolonomicWaypoint, jHolonomicWaypointClassHeadingField);
  jdouble jHolonomicWaypointVelocityX = env->GetDoubleField(
      jHolonomicWaypoint, jHolonomicWaypointClassVelocityXField);
  jdouble jHolonomicWaypointVelocityY = env->GetDoubleField(
      jHolonomicWaypoint, jHolonomicWaypointClassVelocityYField);
  jdouble jHolonomicWaypointAngularVelocity = env->GetDoubleField(
      jHolonomicWaypoint, jHolonomicWaypointClassAngularVelocityField);
  jboolean jHolonomicWaypointXConstrained = env->GetBooleanField(
      jHolonomicWaypoint, jHolonomicWaypointClassXConstrainedField);
  jboolean jHolonomicWaypointYConstrained = env->GetBooleanField(
      jHolonomicWaypoint, jHolonomicWaypointClassYConstrainedField);
  jboolean jHolonomicWaypointHeadingConstrained = env->GetBooleanField(
      jHolonomicWaypoint, jHolonomicWaypointClassHeadingConstrainedField);
  jboolean jHolonomicWaypointVelocityXConstrained = env->GetBooleanField(
      jHolonomicWaypoint, jHolonomicWaypointClassVelocityXConstrainedField);
  jboolean jHolonomicWaypointVelocityYConstrained = env->GetBooleanField(
      jHolonomicWaypoint, jHolonomicWaypointClassVelocityYConstrainedField);
  jboolean jHolonomicWaypointVelocityMagnitudeConstrained =
      env->GetBooleanField(
          jHolonomicWaypoint,
          jHolonomicWaypointClassVelocityMagnitudeConstrainedField);
  jboolean jHolonomicWaypointAngularVelocityConstrained = env->GetBooleanField(
      jHolonomicWaypoint,
      jHolonomicWaypointClassAngularVelocityConstrainedField);
  jint jHolonomicWaypointControlIntervalCount = env->GetIntField(
      jHolonomicWaypoint, jHolonomicWaypointClassControlIntervalCountField);
  jobject jHolonomicWaypointInitialGuessPoints = env->GetObjectField(
      jHolonomicWaypoint, jHolonomicWaypointClassInitialGuessPointsField);

  std::vector<InitialGuessPoint> initialGuessPoints =
      vectorFromJList<InitialGuessPoint,
                      initialGuessPointFromJInitialGuessPoint>(
          env, jHolonomicWaypointInitialGuessPoints);

  initialGuessPoints.push_back(InitialGuessPoint{
      jHolonomicWaypointX, jHolonomicWaypointY, jHolonomicWaypointHeading});

  auto constraints = poseConstraintsFrom(
      jHolonomicWaypointX, jHolonomicWaypointY, jHolonomicWaypointHeading,
      jHolonomicWaypointXConstrained, jHolonomicWaypointYConstrained,
      jHolonomicWaypointHeadingConstrained);

  auto velocityConstraints = velocityConstraintsFrom(
      jHolonomicWaypointVelocityX, jHolonomicWaypointVelocityY,
      jHolonomicWaypointAngularVelocity, jHolonomicWaypointVelocityXConstrained,
      jHolonomicWaypointVelocityYConstrained,
      jHolonomicWaypointVelocityMagnitudeConstrained,
      jHolonomicWaypointAngularVelocityConstrained);

  constraints.insert(constraints.end(), velocityConstraints.begin(),
                     velocityConstraints.end());

  return {constraints, initialGuessPoints,
          jHolonomicWaypointControlIntervalCount};
}

SwervePathBuilder swervePathFromJHolonomicPath(JNIEnv* env,
                                               jobject jHolonomicPath) {
  jclass jPathClass = env->FindClass("org/sleipnirgroup/trajopt/Path");
  jclass jHolonomicPathClass =
      env->FindClass("org/sleipnirgroup/trajopt/HolonomicPath");
  jfieldID jHolonomicPathClassHolonomicWaypointsField = env->GetFieldID(
      jHolonomicPathClass, "holonomicWaypoints", "Ljava/util/List;");
  jfieldID jPathClassObstaclesField =
      env->GetFieldID(jPathClass, "obstacles", "Ljava/util/List;");

  jobject jHolonomicPathHolonomicWaypoints = env->GetObjectField(
      jHolonomicPath, jHolonomicPathClassHolonomicWaypointsField);
  jobject jPathObstacles =
      env->GetObjectField(jHolonomicPath, jPathClassObstaclesField);

  SwervePathBuilder path;

  auto holonomicWaypoints =
      vectorFromJList<std::tuple<std::vector<HolonomicConstraint>,
                                 std::vector<InitialGuessPoint>, size_t>,
                      &holonomicConstraintsFromJHolonomicWaypoint>(
          env, jHolonomicPathHolonomicWaypoints);

  size_t wptCnt = holonomicWaypoints.size();

  std::vector<size_t> controlIntervalCounts;
  controlIntervalCounts.reserve(wptCnt - 1);

  for (size_t wptIdx = 0; wptIdx < wptCnt; ++wptIdx) {
    std::vector<HolonomicConstraint>& wptConstraints =
        std::get<0>(holonomicWaypoints.at(wptIdx));
    std::vector<InitialGuessPoint>& wptInitialGuessPts =
        std::get<1>(holonomicWaypoints.at(wptIdx));
    size_t wptControlIntervalCount = std::get<2>(holonomicWaypoints.at(wptIdx));
    for (auto& constraint : wptConstraints) {
      path.WptConstraint(wptIdx, constraint);
    }
    path.WptInitialGuessPoint(wptIdx, wptInitialGuessPts.back());
    wptInitialGuessPts.pop_back();
    if (wptIdx > 0) {
      path.SgmtInitialGuessPoints(wptIdx - 1, wptInitialGuessPts);
      controlIntervalCounts.push_back(wptControlIntervalCount);
    }
  }

  path.ControlIntervalCounts(std::move(controlIntervalCounts));

  std::vector<Obstacle> obstacles =
      vectorFromJList<Obstacle, obstacleFromJObstacle>(env, jPathObstacles);

  for (auto& obstacle : obstacles) {
    path.SgmtObstacle(0, wptCnt - 1, obstacle);
  }

  return path;
}

jobject jHolonomicTrajectorySampleFromHolonomicTrajectorySample(
    JNIEnv* env, const HolonomicTrajectorySample& holonomicTrajectorySample) {
  jclass jHolonomicTrajectorySampleClass =
      env->FindClass("org/sleipnirgroup/trajopt/HolonomicTrajectorySample");
  jmethodID jHolonomicTrajectorySampleClassConstructor =
      env->GetMethodID(jHolonomicTrajectorySampleClass, "<init>", "(DDDDDDD)V");

  return env->NewObject(
      jHolonomicTrajectorySampleClass,
      jHolonomicTrajectorySampleClassConstructor,
      holonomicTrajectorySample.timestamp, holonomicTrajectorySample.x,
      holonomicTrajectorySample.y, holonomicTrajectorySample.heading,
      holonomicTrajectorySample.velocityX, holonomicTrajectorySample.velocityY,
      holonomicTrajectorySample.angularVelocity);
}

template <typename Type, jobject (*converter)(JNIEnv* env, const Type& object)>
jobject jListFromVector(JNIEnv* env, const std::vector<Type>& vect) {
  jclass jArrayListClass = env->FindClass("java/util/ArrayList");
  jmethodID jArrayListClassConstructor =
      env->GetMethodID(jArrayListClass, "<init>", "(I)V");
  jmethodID jArrayListClassAddMethod =
      env->GetMethodID(jArrayListClass, "add", "(Ljava/lang/Object;)Z");

  size_t size = vect.size();
  jobject jArrayList =
      env->NewObject(jArrayListClass, jArrayListClassConstructor, size);
  for (size_t index = 0; index < size; index++) {
    env->CallBooleanMethod(jArrayList, jArrayListClassAddMethod,
                           converter(env, vect[index]));
  }

  return jArrayList;
}

jobject jHolonomicTrajectoryFromHolonomicTrajectory(
    JNIEnv* env, const HolonomicTrajectory& holonomicTrajectory) {
  jclass jHolonomicTrajectoryClass =
      env->FindClass("org/sleipnirgroup/trajopt/HolonomicTrajectory");
  jmethodID jHolonomicTrajectoryClassConstructor = env->GetMethodID(
      jHolonomicTrajectoryClass, "<init>", "(Ljava/util/List;)V");

  return env->NewObject(
      jHolonomicTrajectoryClass, jHolonomicTrajectoryClassConstructor,
      jListFromVector<HolonomicTrajectorySample,
                      jHolonomicTrajectorySampleFromHolonomicTrajectorySample>(
          env, holonomicTrajectory.samples));
}

/*
 * Class:     org_sleipnirgroup_trajopt_OptimalTrajectoryGenerator
 * Method:    generateHolonomicTrajectory
 * Signature: (Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_org_sleipnirgroup_trajopt_OptimalTrajectoryGenerator_generateHolonomicTrajectory
  (JNIEnv* env, jobject jObject, jobject jSwerveDrivetrain,
   jobject jHolonomicPath)
{
  auto swerveDrivetrain =
      swerveDrivetrainFromJSwerveDrivetrain(env, jSwerveDrivetrain);
  trajopt::SwervePathBuilder path =
      swervePathFromJHolonomicPath(env, jHolonomicPath);
  path.SetDrivetrain(swerveDrivetrain);

  try {
    auto solution = OptimalTrajectoryGenerator::Generate(path);
    auto trajectory = HolonomicTrajectory(solution);
    return jHolonomicTrajectoryFromHolonomicTrajectory(env, trajectory);
  } catch (const InvalidPathException& e) {
    jclass jInvalidPathExceptionClass =
        env->FindClass("org/sleipnirgroup/trajopt/InvalidPathException");
    env->ThrowNew(jInvalidPathExceptionClass, e.what());
    return nullptr;
  } catch (const TrajectoryGenerationException& e) {
    jclass jTrajectoryGenerationExceptionClass = env->FindClass(
        "org/sleipnirgroup/trajopt/TrajectoryGenerationException");
    env->ThrowNew(jTrajectoryGenerationExceptionClass, e.what());
    return nullptr;
  }
}
