#include "org_team2363_helixtrajectory_HolonomicTrajectoryGenerator.h"

#include <iostream>
#include <memory>

#include <jni.h>

#include "HolonomicPath.h"
#include "HolonomicDrive.h"
#include "SwerveDrive.h"
#include "HolonomicTrajectoryGenerator.h"
#include "Obstacle.h"
#include "TrajectoryUtil.h"

using namespace helixtrajectory;

std::unique_ptr<HolonomicDrive> holonomicDriveFromJHolonomicDrive(JNIEnv* env, jobject jHolonomicDrive);
Obstacle obstacleFromJObstacle(JNIEnv* env, jobject jObstacle);

JNIEXPORT jobject JNICALL Java_org_team2363_helixtrajectory_HolonomicTrajectoryGenerator_generate
        (JNIEnv* env, jobject jObj) {
    
    jclass jListClass = env->FindClass("java/util/List");
    jmethodID jListClassSizeMethod = env->GetMethodID(jListClass, "size", "()I");
    jmethodID jListClassGetMethod = env->GetMethodID(jListClass, "get", "(I)Ljava/lang/Object;");

    jclass jHolonomicPathClass = env->FindClass("org/team2363/helixtrajectory/HolonomicPath");
    jfieldID jHolonomicPathClassHolonomicWaypointsField = env->GetFieldID(jHolonomicPathClass, "holonomicWaypoints", "Ljava/util/List;");

    jclass jHolonomicWaypointClass = env->FindClass("org/team2363/helixtrajectory/HolonomicWaypoint");
    jfieldID jHolonomicWaypointClassXField = env->GetFieldID(jHolonomicWaypointClass, "x", "D");
    jfieldID jHolonomicWaypointClassYField = env->GetFieldID(jHolonomicWaypointClass, "y", "D");
    jfieldID jHolonomicWaypointClassHeadingField = env->GetFieldID(jHolonomicWaypointClass, "heading", "D");
    jfieldID jHolonomicWaypointClassVelocityXField = env->GetFieldID(jHolonomicWaypointClass, "velocityX", "D");
    jfieldID jHolonomicWaypointClassVelocityYField = env->GetFieldID(jHolonomicWaypointClass, "velocityY", "D");
    jfieldID jHolonomicWaypointClassAngularVelocityField = env->GetFieldID(jHolonomicWaypointClass, "angularVelocity", "D");
    jfieldID jHolonomicWaypointClassXConstrainedField = env->GetFieldID(jHolonomicWaypointClass, "xConstrained", "Z");
    jfieldID jHolonomicWaypointClassYConstrainedField = env->GetFieldID(jHolonomicWaypointClass, "yConstrained", "Z");
    jfieldID jHolonomicWaypointClassHeadingConstrainedField = env->GetFieldID(jHolonomicWaypointClass, "headingConstrained", "Z");
    jfieldID jHolonomicWaypointClassVelocityXConstrainedField = env->GetFieldID(jHolonomicWaypointClass, "velocityXConstrained", "Z");
    jfieldID jHolonomicWaypointClassVelocityYConstrainedField = env->GetFieldID(jHolonomicWaypointClass, "velocityYConstrained", "Z");
    jfieldID jHolonomicWaypointClassVelocityMagnitudeConstrainedField = env->GetFieldID(jHolonomicWaypointClass, "velocityMagnitudeConstrained", "Z");
    jfieldID jHolonomicWaypointClassAngularVelocityConstrainedField = env->GetFieldID(jHolonomicWaypointClass, "angularVelocityConstrained", "Z");
    jfieldID jHolonomicWaypointClassInitialGuessPointsField = env->GetFieldID(jHolonomicWaypointClass, "initialGuessPoints", "Ljava/util/List;");

    jclass jInitialGuessPointClass = env->FindClass("org/team2363/helixtrajectory/InitialGuessPoint");
    jmethodID jInitialGuessPointClassXMethod = env->GetMethodID(jInitialGuessPointClass, "x", "()D");
    jmethodID jInitialGuessPointClassYMethod = env->GetMethodID(jInitialGuessPointClass, "y", "()D");
    jmethodID jInitialGuessPointClassHeadingMethod = env->GetMethodID(jInitialGuessPointClass, "heading", "()D");

    jclass jHolonomicTrajectorySampleClass = env->FindClass("org/team2363/helixtrajectory/HolonomicTrajectorySample");
    jmethodID jHolonomicTrajectorySampleClassConstructor = env->GetMethodID(jHolonomicTrajectorySampleClass, "<init>", "(DDDDDDD)V");
    jclass jHolonomicTrajectoryClass = env->FindClass("org/team2363/helixtrajectory/HolonomicTrajectory");
    jmethodID jHolonomicTrajectoryClassConstructor = env->GetMethodID(jHolonomicTrajectoryClass, "<init>", "(Lorg/team2363/helixtrajectory/HolonomicTrajectorySample;[])V");
    
    jclass jHolonomicTrajectoryGeneratorClass = env->FindClass("org/team2363/helixtrajectory/HolonomicTrajectoryGenerator");
    jfieldID jHolonomicTrajectoryGeneratorClassHolonomicDriveField = env->GetFieldID(jHolonomicTrajectoryGeneratorClass, "holonomicDrive", "Lorg/team2363/helixtrajectory/HolonomicDrive;");
    jfieldID jHolonomicTrajectoryGeneratorClassHolonomicPathField = env->GetFieldID(jHolonomicTrajectoryGeneratorClass, "holonomicPath", "Lorg/team2363/helixtrajectory/HolonomicPath;");
    jfieldID jHolonomicTrajectoryGeneratorClassObstaclesField = env->GetFieldID(jHolonomicTrajectoryGeneratorClass, "obstacles", "Ljava/util/List;");

    jobject jHolonomicDrive = env->GetObjectField(jObj, jHolonomicTrajectoryGeneratorClassHolonomicDriveField);
    jobject jHolonomicPath = env->GetObjectField(jObj, jHolonomicTrajectoryGeneratorClassHolonomicPathField);
    jobject jObstacles = env->GetObjectField(jObj, jHolonomicTrajectoryGeneratorClassObstaclesField);

    jobject jHolonomicPathHolonomicWaypoints = env->GetObjectField(jHolonomicPath, jHolonomicPathClassHolonomicWaypointsField);
    size_t waypointCount = env->CallIntMethod(jHolonomicPathHolonomicWaypoints, jListClassSizeMethod);
    std::vector<HolonomicWaypoint> waypoints;
    waypoints.reserve(waypointCount);
    for (size_t i = 0; i < waypointCount; i++) {
        jobject jHolonomicWaypoint = env->CallObjectMethod(jHolonomicPath, jListClassGetMethod, i);
        double x = env->GetDoubleField(jHolonomicWaypoint, jHolonomicWaypointClassXField);
        double y = env->GetDoubleField(jHolonomicWaypoint, jHolonomicWaypointClassYField);
        double heading = env->GetDoubleField(jHolonomicWaypoint, jHolonomicWaypointClassHeadingField);
        double velocityX = env->GetDoubleField(jHolonomicWaypoint, jHolonomicWaypointClassVelocityXField);
        double velocityY = env->GetDoubleField(jHolonomicWaypoint, jHolonomicWaypointClassVelocityYField);
        double angularVelocity = env->GetDoubleField(jHolonomicWaypoint, jHolonomicWaypointClassAngularVelocityField);
        jboolean jXConstrained = env->GetBooleanField(jHolonomicWaypoint, jHolonomicWaypointClassXConstrainedField);
        jboolean jYConstrained = env->GetBooleanField(jHolonomicWaypoint, jHolonomicWaypointClassYConstrainedField);
        jboolean jHeadingConstrained = env->GetBooleanField(jHolonomicWaypoint, jHolonomicWaypointClassHeadingConstrainedField);
        jboolean jVelocityXConstrained = env->GetBooleanField(jHolonomicWaypoint, jHolonomicWaypointClassVelocityXConstrainedField);
        jboolean jVelocityYConstrained = env->GetBooleanField(jHolonomicWaypoint, jHolonomicWaypointClassVelocityYConstrainedField);
        jboolean jVelocityMagnitudeConstrained = env->GetBooleanField(jHolonomicWaypoint, jHolonomicWaypointClassVelocityMagnitudeConstrainedField);
        jboolean jAngularVelocityConstrained = env->GetBooleanField(jHolonomicWaypoint, jHolonomicWaypointClassAngularVelocityConstrainedField);
        jobject jInitialGuessPoints = env->GetObjectField(jHolonomicWaypoint, jHolonomicWaypointClassInitialGuessPointsField);
        size_t initialGuessPointCount = env->CallIntMethod(jInitialGuessPoints, jListClassSizeMethod);
        std::vector<InitialGuessPoint> initialGuessPoints;
        initialGuessPoints.reserve(initialGuessPointCount);
        for (size_t initialGuessPointIndex = 0; initialGuessPointIndex < initialGuessPointCount; initialGuessPointIndex++) {
            jobject jInitialGuessPoint = env->CallObjectMethod(jInitialGuessPoints, jListClassGetMethod, initialGuessPointIndex);
            jdouble jInitialGuessPointX = env->CallDoubleMethod(jInitialGuessPoint, jInitialGuessPointClassXMethod);
            jdouble jInitialGuessPointY = env->CallDoubleMethod(jInitialGuessPoint, jInitialGuessPointClassYMethod);
            jdouble jInitialGuessPointHeading = env->CallDoubleMethod(jInitialGuessPoint, jInitialGuessPointClassHeadingMethod);
            initialGuessPoints.push_back({jInitialGuessPointX, jInitialGuessPointY, jInitialGuessPointHeading});
        }
        waypoints.push_back({x, y, heading, velocityX, velocityY, angularVelocity,
                static_cast<bool>(jXConstrained),
                static_cast<bool>(jYConstrained),
                static_cast<bool>(jHeadingConstrained),
                static_cast<bool>(jVelocityXConstrained),
                static_cast<bool>(jVelocityYConstrained),
                static_cast<bool>(jVelocityMagnitudeConstrained),
                static_cast<bool>(jAngularVelocityConstrained),
                initialGuessPoints});
    }
    HolonomicPath holonomicPath = waypoints;

    const std::unique_ptr<HolonomicDrive> holonomicDrive = holonomicDriveFromJHolonomicDrive(env, jHolonomicDrive);

    size_t obstacleCount = env->CallIntMethod(jObstacles, jListClassSizeMethod);
    std::vector<Obstacle> obstacles;
    obstacles.reserve(obstacleCount);
    for (size_t obstacleIndex = 0; obstacleIndex < obstacleCount; obstacleIndex++) {
        jobject jObstacle = env->CallObjectMethod(jObstacles, jListClassGetMethod, obstacleIndex);
        obstacles.push_back(obstacleFromJObstacle(env, jObstacle));
    }

    HolonomicTrajectoryGenerator generator(*holonomicDrive, holonomicPath, obstacles);
    std::unique_ptr<HolonomicTrajectory> traj = generator.Generate();
    size_t sampleCount = traj->samples.size();
    jobjectArray jSampleArray = env->NewObjectArray(sampleCount, jHolonomicTrajectorySampleClass, NULL);
    for (size_t sampleIndex = 0; sampleIndex < sampleCount; sampleIndex++) {
        env->SetObjectArrayElement(jSampleArray, sampleIndex, env->NewObject(jHolonomicTrajectorySampleClass, jHolonomicTrajectorySampleClassConstructor,
                traj->samples[sampleIndex].ts,
                traj->samples[sampleIndex].x,
                traj->samples[sampleIndex].y,
                traj->samples[sampleIndex].heading,
                traj->samples[sampleIndex].vx,
                traj->samples[sampleIndex].vy,
                traj->samples[sampleIndex].omega));
    }
    jobject jHolonomicTrajectory = env->NewObject(jHolonomicTrajectoryClass, jHolonomicTrajectoryClassConstructor, jSampleArray);
    return jHolonomicTrajectory;
}

std::unique_ptr<HolonomicDrive> holonomicDriveFromJHolonomicDrive(JNIEnv* env, jobject jHolonomicDrive) {
    jclass jListClass = env->FindClass("java/util/List");
    jmethodID jListClassSizeMethod = env->GetMethodID(jListClass, "size", "()I");
    jmethodID jListClassGetMethod = env->GetMethodID(jListClass, "get", "(I)Ljava/lang/Object;");

    jclass jHolonomicDriveClass = env->FindClass("org/team2363/helixtrajectory/HolonomicDrive");
    jfieldID jHolonomicDriveClassMassField = env->GetFieldID(jHolonomicDriveClass, "mass", "D");
    jfieldID jHolonomicDriveClassMoiField = env->GetFieldID(jHolonomicDriveClass, "moi", "D");
    jfieldID jHolonomicDriveClassBumpersField = env->GetFieldID(jHolonomicDriveClass, "bumpers", "Lorg/team2363/helixtrajectory/Obstacle;");

    jclass jSwerveDriveClass = env->FindClass("org/team2363/helixtrajectory/SwerveDrive");
    jfieldID jSwerveDriveClassModulesField = env->GetFieldID(jSwerveDriveClass, "modules", "Ljava/util/List;");

    jclass jSwerveModuleClass = env->FindClass("org/team2363/helixtrajectory/SwerveModule");
    jmethodID jSwerveModuleClassXMethod = env->GetMethodID(jSwerveModuleClass, "x", "()D");
    jmethodID jSwerveModuleClassYMethod = env->GetMethodID(jSwerveModuleClass, "y", "()D");
    jmethodID jSwerveModuleClassWheelRadiusMethod = env->GetMethodID(jSwerveModuleClass, "wheelRadius", "()D");
    jmethodID jSwerveModuleClassWheelMaxAngularVelocityMethod = env->GetMethodID(jSwerveModuleClass, "wheelMaxAngularVelocity", "()D");
    jmethodID jSwerveModuleClassWheelMaxTorqueMethod = env->GetMethodID(jSwerveModuleClass, "wheelMaxTorque", "()D");

    jdouble jHolonomicDriveMass = env->GetDoubleField(jHolonomicDrive, jHolonomicDriveClassMassField);
    jdouble jHolonomicDriveMoi = env->GetDoubleField(jHolonomicDrive, jHolonomicDriveClassMassField);
    jobject jHolonomicDriveBumpers = env->GetObjectField(jHolonomicDrive, jHolonomicDriveClassBumpersField);

    Obstacle bumpers = obstacleFromJObstacle(env, jHolonomicDriveBumpers);

    if (env->IsInstanceOf(jHolonomicDrive, jSwerveDriveClass)) {
        jobject jSwerveDrive = jHolonomicDrive;
        jobject jSwerveDriveModules = env->GetObjectField(jSwerveDrive, jSwerveDriveClassModulesField);
        size_t swerveModuleCount = env->CallIntMethod(jSwerveDriveModules, jListClassSizeMethod);
        std::vector<SwerveModule> modules;
        modules.reserve(swerveModuleCount);
        for (size_t swerveModuleIndex = 0; swerveModuleIndex < swerveModuleCount; swerveModuleIndex++) {
            jobject jSwerveModule = env->CallObjectMethod(jSwerveDriveModules, jListClassGetMethod, swerveModuleIndex);
            modules.push_back({
                env->CallDoubleMethod(jSwerveModule, jSwerveModuleClassXMethod),
                env->CallDoubleMethod(jSwerveModule, jSwerveModuleClassYMethod),
                env->CallDoubleMethod(jSwerveModule, jSwerveModuleClassWheelRadiusMethod),
                env->CallDoubleMethod(jSwerveModule, jSwerveModuleClassWheelMaxAngularVelocityMethod),
                env->CallDoubleMethod(jSwerveModule, jSwerveModuleClassWheelMaxTorqueMethod)});
        }
        return std::unique_ptr<HolonomicDrive>(new SwerveDrive(jHolonomicDriveMass, jHolonomicDriveMoi, modules, bumpers));
    } else {
        throw "Only swerve is supported currently";
    }
}

Obstacle obstacleFromJObstacle(JNIEnv* env, jobject jObstacle) {
    jclass jListClass = env->FindClass("java/util/List");
    jmethodID jListClassSizeMethod = env->GetMethodID(jListClass, "size", "()I");
    jmethodID jListClassGetMethod = env->GetMethodID(jListClass, "get", "(I)Ljava/lang/Object;");

    jclass jObstacleClass = env->FindClass("org/team2363/helixtrajectory/Obstacle");
    jfieldID jObstacleClassSafetyDistanceField = env->GetFieldID(jObstacleClass, "safetyDistance", "D");
    jfieldID jObstacleClassObstaclePointsField = env->GetFieldID(jObstacleClass, "obstaclePoints", "Ljava/util/List;");
    
    jclass jObstaclePointClass = env->FindClass("org/team2363/helixtrajectory/ObstaclePoint");
    jmethodID jObstaclePointClassXMethod = env->GetMethodID(jObstaclePointClass, "x", "()D");
    jmethodID jObstaclePointClassYMethod = env->GetMethodID(jObstaclePointClass, "y", "()D");

    double obstacleSafetyDistance = env->GetDoubleField(jObstacle, jObstacleClassSafetyDistanceField);
    jobject jObstacleObstaclePoints = env->GetObjectField(jObstacle, jObstacleClassObstaclePointsField);
    size_t obstaclePointCount = env->CallIntMethod(jObstacleObstaclePoints, jListClassSizeMethod);

    std::vector<ObstaclePoint> obstaclePoints;
    obstaclePoints.reserve(obstaclePointCount);
    for (size_t obstaclePointIndex = 0; obstaclePointIndex < obstaclePointCount; obstaclePointIndex++) {
        jobject jObstaclePoint = env->CallObjectMethod(jObstacleObstaclePoints, jListClassGetMethod, obstaclePointIndex);
        double obstaclePointX = env->CallDoubleMethod(jObstaclePoint, jObstaclePointClassXMethod);
        double obstaclePointY = env->CallDoubleMethod(jObstaclePoint, jObstaclePointClassYMethod);
        obstaclePoints.push_back({obstaclePointX, obstaclePointY});
    }

    return Obstacle(obstacleSafetyDistance, obstaclePoints);
}