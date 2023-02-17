// Copyright (c) TrajoptLib contributors

#include <algorithm>
#include <memory>

#include "InvalidPathException.h"
#include "OptimalTrajectoryGenerator.h"
#include "TrajectoryGenerationException.h"
#include "constraint/ObstacleConstraint.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "drivetrain/SwerveModule.h"
#include "jni.h"
#include "obstacle/Obstacle.h"
#include "obstacle/ObstaclePoint.h"
#include "org_sleipnirgroup_trajopt_OptimalTrajectoryGenerator.h"
#include "path/HolonomicPath.h"
#include "path/HolonomicWaypoint.h"
#include "path/InitialGuessPoint.h"
#include "trajectory/HolonomicTrajectory.h"

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
      env->FindClass("org/team2363/trajopt/ObstaclePoint");
  jfieldID jObstaclePointClassXField =
      env->GetFieldID(jObstaclePointClass, "x", "D");
  jfieldID jObstaclePointClassYField =
      env->GetFieldID(jObstaclePointClass, "y", "D");

  jdouble jObstaclePointX =
      env->GetDoubleField(jObstaclePoint, jObstaclePointClassXField);
  jdouble jObstaclePointY =
      env->GetDoubleField(jObstaclePoint, jObstaclePointClassYField);

  return ObstaclePoint(jObstaclePointX, jObstaclePointY);
}

Obstacle obstacleFromJObstacle(JNIEnv* env, jobject jObstacle) {
  jclass jListClass = env->FindClass("java/util/List");
  jmethodID jListClassSizeMethod = env->GetMethodID(jListClass, "size", "()I");
  jmethodID jListClassGetMethod =
      env->GetMethodID(jListClass, "get", "(I)Ljava/lang/Object;");

  jclass jObstacleClass = env->FindClass("org/team2363/trajopt/Obstacle");
  jfieldID jObstacleClassSafetyDistanceField =
      env->GetFieldID(jObstacleClass, "safetyDistance", "D");
  jfieldID jObstacleClassApplyToAllSegmentsField =
      env->GetFieldID(jObstacleClass, "applyToAllSegments", "Z");
  jfieldID jObstacleClassPointsField =
      env->GetFieldID(jObstacleClass, "points", "Ljava/util/List;");

  jdouble jObstacleSafetyDistance =
      env->GetDoubleField(jObstacle, jObstacleClassSafetyDistanceField);
  jboolean jObstacleApplyToAllSegments =
      env->GetBooleanField(jObstacle, jObstacleClassApplyToAllSegmentsField);
  jobject jObstaclePoints =
      env->GetObjectField(jObstacle, jObstacleClassPointsField);
  jint jObstaclePointsSize =
      env->CallIntMethod(jObstaclePoints, jListClassSizeMethod);

  std::vector<ObstaclePoint> obstaclePoints =
      vectorFromJList<ObstaclePoint, obstaclePointFromJObstaclePoint>(
          env,
          jObstaclePoints);  // <-- i love templates

      return Obstacle(jObstacleSafetyDistance,
                      obstaclePoints);
}

InitialGuessPoint initialGuessPointFromJInitialGuessPoint(
    JNIEnv* env, jobject jInitialGuessPoint) {
  jclass jInitialGuessPointClass =
      env->FindClass("org/team2363/trajopt/InitialGuessPoint");
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

  return InitialGuessPoint(jInitialGuessPointX, jInitialGuessPointY,
                           jInitialGuessPointHeading);
}

SwerveModule swerveModuleFromJSwerveModule(JNIEnv* env, jobject jSwerveModule) {
  jclass jSwerveModuleClass =
      env->FindClass("org/team2363/trajopt/SwerveModule");
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

  return SwerveModule(jSwerveModuleX, jSwerveModuleY, jSwerveModuleWheelRadius,
                      jSwerveModuleWheelMaxAngularVelocity,
                      jSwerveModuleWheelMaxTorque);
}

SwerveDrivetrain swerveDrivetrainFromJSwerveDrivetrain(
    JNIEnv* env, jobject jSwerveDrivetrain) {
  jclass jListClass = env->FindClass("java/util/List");
  jmethodID jListClassSizeMethod = env->GetMethodID(jListClass, "size", "()I");
  jmethodID jListClassGetMethod =
      env->GetMethodID(jListClass, "get", "(I)Ljava/lang/Object;");

  jclass jSwerveDrivetrainClass =
      env->FindClass("org/team2363/trajopt/SwerveDrivetrain");
  jfieldID jSwerveDrivetrainClassMassField =
      env->GetFieldID(jSwerveDrivetrainClass, "mass", "D");
  jfieldID jSwerveDrivetrainClassMomentOfInertiaField =
      env->GetFieldID(jSwerveDrivetrainClass, "momentOfInertia", "D");
  jfieldID jSwerveDrivetrainClassModulesField =
      env->GetFieldID(jSwerveDrivetrainClass, "modules", "Ljava/util/List;");
  jfieldID jSwerveDrivetrainClassBumpersField = env->GetFieldID(
      jSwerveDrivetrainClass, "bumpers", "Lorg/team2363/trajopt/Obstacle;");

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

  return SwerveDrivetrain(jSwerveDrivetrainMass,
                          jSwerveDrivetrainMomentOfInertia, modules);
}

HolonomicWaypoint holonomicWaypointFromJHolonomicWaypoint(
    JNIEnv* env, jobject jHolonomicWaypoint) {
  jclass jHolonomicWaypointClass =
      env->FindClass("org/team2363/trajopt/HolonomicWaypoint");
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
  jfieldID jHolonomicWaypointClassObstaclesField =
      env->GetFieldID(jHolonomicWaypointClass, "obstacles", "Ljava/util/List;");

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
  jobject jHolonomicWaypointObstacles = env->GetObjectField(
      jHolonomicWaypoint, jHolonomicWaypointClassObstaclesField);

  std::vector<InitialGuessPoint> initialGuessPoints =
      vectorFromJList<InitialGuessPoint,
                      initialGuessPointFromJInitialGuessPoint>(
          env, jHolonomicWaypointInitialGuessPoints);
  std::vector<Obstacle> obstacles =
      vectorFromJList<Obstacle, obstacleFromJObstacle>(
          env, jHolonomicWaypointObstacles);

  std::vector<ObstacleConstraint> obstConsts(obstacles.size());
  for (auto& obst : obstacles) {
    obstConsts.push_back(ObstacleConstraint{obst});
  }

  std::vector<



  return HolonomicWaypoint(
      jHolonomicWaypointX, jHolonomicWaypointY, jHolonomicWaypointHeading,
      jHolonomicWaypointVelocityX, jHolonomicWaypointY,
      jHolonomicWaypointAngularVelocity, jHolonomicWaypointXConstrained,
      jHolonomicWaypointYConstrained, jHolonomicWaypointHeadingConstrained,
      jHolonomicWaypointVelocityXConstrained,
      jHolonomicWaypointVelocityYConstrained,
      jHolonomicWaypointVelocityMagnitudeConstrained,
      jHolonomicWaypointAngularVelocityConstrained,
      jHolonomicWaypointControlIntervalCount, initialGuessPoints, obstacles);
}

HolonomicPath holonomicPathFromJHolonomicPath(JNIEnv* env,
                                              jobject jHolonomicPath) {
  jclass jHolonomicPathClass =
      env->FindClass("org/team2363/trajopt/HolonomicPath");
  jfieldID jHolonomicPathClassHolonomicWaypointsField = env->GetFieldID(
      jHolonomicPathClass, "holonomicWaypoints", "Ljava/util/List;");

  jobject jHolonomicPathHolonomicWaypoints = env->GetObjectField(
      jHolonomicPath, jHolonomicPathClassHolonomicWaypointsField);

  std::vector<HolonomicWaypoint> holonomicWaypoints =
      vectorFromJList<HolonomicWaypoint,
                      holonomicWaypointFromJHolonomicWaypoint>(
          env, jHolonomicPathHolonomicWaypoints);

  return HolonomicPath(holonomicWaypoints);
}

jobject jHolonomicTrajectorySampleFromHolonomicTrajectorySample(
    JNIEnv* env, const HolonomicTrajectorySample& holonomicTrajectorySample) {
  jclass jHolonomicTrajectorySampleClass =
      env->FindClass("org/team2363/trajopt/HolonomicTrajectorySample");
  jmethodID jHolonomicTrajectorySampleClassConstructor =
      env->GetMethodID(jHolonomicTrajectorySampleClass, "<init>", "(DDDDDDD)V");

  return env->NewObject(
      jHolonomicTrajectorySampleClass,
      jHolonomicTrajectorySampleClassConstructor,
      holonomicTrajectorySample.intervalDuration, holonomicTrajectorySample.x,
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

jobject jHolonomicTrajectorySegmentFromHolonomicTrajectorySegment(
    JNIEnv* env, const HolonomicTrajectorySegment& holonomicTrajectorySegment) {
  jclass jHolonomicTrajectorySegmentClass =
      env->FindClass("org/team2363/trajopt/HolonomicTrajectorySegment");
  jmethodID jHolonomicTrajectorySegmentClassConstructor = env->GetMethodID(
      jHolonomicTrajectorySegmentClass, "<init>", "(Ljava/util/List;)V");

  return env->NewObject(
      jHolonomicTrajectorySegmentClass,
      jHolonomicTrajectorySegmentClassConstructor,
      jListFromVector<HolonomicTrajectorySample,
                      jHolonomicTrajectorySampleFromHolonomicTrajectorySample>(
          env, holonomicTrajectorySegment.holonomicSamples));
}

jobject jHolonomicTrajectoryFromHolonomicTrajectory(
    JNIEnv* env, const HolonomicTrajectory& holonomicTrajectory) {
  jclass jHolonomicTrajectoryClass =
      env->FindClass("org/team2363/trajopt/HolonomicTrajectory");
  jmethodID jHolonomicTrajectoryClassConstructor = env->GetMethodID(
      jHolonomicTrajectoryClass, "<init>", "(Ljava/util/List;)V");

  return env->NewObject(
      jHolonomicTrajectoryClass, jHolonomicTrajectoryClassConstructor,
      jListFromVector<
          HolonomicTrajectorySegment,
          jHolonomicTrajectorySegmentFromHolonomicTrajectorySegment>(
          env, holonomicTrajectory.holonomicSegments));
}

/*
 * Class:     org_team2363_trajopt_OptimalTrajectoryGenerator
 * Method:    generateHolonomicTrajectory
 * Signature: (Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL
Java_org_team2363_trajopt_OptimalTrajectoryGenerator_generateHolonomicTrajectory
  (JNIEnv* env, jobject jObject, jobject jSwerveDrivetrain,
   jobject jHolonomicPath)
{
  SwerveDrivetrain swerveDrivetrain =
      swerveDrivetrainFromJSwerveDrivetrain(env, jSwerveDrivetrain);
  HolonomicPath holonomicPath =
      holonomicPathFromJHolonomicPath(env, jHolonomicPath);

  try {
    HolonomicTrajectory holonomicTrajectory =
        OptimalTrajectoryGenerator::Generate(swerveDrivetrain, holonomicPath);
    return jHolonomicTrajectoryFromHolonomicTrajectory(env,
                                                       holonomicTrajectory);
  } catch (const InvalidPathException& e) {
    jclass jInvalidPathExceptionClass =
        env->FindClass("org/team2363/trajopt/InvalidPathException");
    env->ThrowNew(jInvalidPathExceptionClass, e.what());
    return nullptr;
  } catch (const TrajectoryGenerationException& e) {
    jclass jTrajectoryGenerationExceptionClass =
        env->FindClass("org/team2363/trajopt/TrajectoryGenerationException");
    env->ThrowNew(jTrajectoryGenerationExceptionClass, e.what());
    return nullptr;
  }
}
