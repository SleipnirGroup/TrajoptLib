#include <jni.h>
#include "org_team2363_helixtrajectory_HelixTrajectoryInterface.h"

#include <iostream>

#include "Path.h"
#include "TrajectoryGenerator.h"
#include "TrajectoryUtil.h"

JNIEXPORT jdoubleArray JNICALL Java_org_team2363_helixtrajectory_HelixTrajectoryInterface_generateTrajectory
        (JNIEnv *env, jobject jObj, jobject jDrive, jobject jPath, jobjectArray jObstacles) {
    using namespace helixtrajectory;
    
    jclass jPathClass = env->FindClass("org/team2363/helixtrajectory/Path");
    jmethodID jPathClassLengthMethod = env->GetMethodID(jPathClass, "length", "()I");
    jmethodID jPathClassGetMethod = env->GetMethodID(jPathClass, "get", "(I)Lorg/team2363/helixtrajectory/Waypoint;");
    jclass jWaypointClass = env->FindClass("org/team2363/helixtrajectory/Waypoint");
    jfieldID jWaypointClassX = env->GetFieldID(jWaypointClass, "x", "D");
    jfieldID jWaypointClassY = env->GetFieldID(jWaypointClass, "y", "D");
    jfieldID jWaypointClassHeading = env->GetFieldID(jWaypointClass, "heading", "D");
    jfieldID jWaypointClassVX = env->GetFieldID(jWaypointClass, "vx", "D");
    jfieldID jWaypointClassVY = env->GetFieldID(jWaypointClass, "vy", "D");
    jfieldID jWaypointClassOmega = env->GetFieldID(jWaypointClass, "omega", "D");
    jfieldID jWaypointClassXConstrained = env->GetFieldID(jWaypointClass, "xConstrained", "Z");
    jfieldID jWaypointClassYConstrained = env->GetFieldID(jWaypointClass, "yConstrained", "Z");
    jfieldID jWaypointClassHeadingConstrained = env->GetFieldID(jWaypointClass, "headingConstrained", "Z");
    jfieldID jWaypointClassVXConstrained = env->GetFieldID(jWaypointClass, "vxConstrained", "Z");
    jfieldID jWaypointClassVYConstrained = env->GetFieldID(jWaypointClass, "vyConstrained", "Z");
    jfieldID jWaypointClassVMagnitudeConstrained = env->GetFieldID(jWaypointClass, "vMagnitudeConstrained", "Z");
    jfieldID jWaypointClassOmegaConstrained = env->GetFieldID(jWaypointClass, "omegaConstrained", "Z");
    jmethodID jWaypointClassInitialGuessPointsLengthMethod = env->GetMethodID(jWaypointClass, "initialGuessPointsLength", "()I");
    jmethodID jWaypointClassGetInitialGuessPointMethod = env->GetMethodID(jWaypointClass, "getInitialGuessPoint", "(I)Lorg/team2363/helixtrajectory/InitialGuessPoint;");
    jclass jInitialGuessPointClass = env->FindClass("org/team2363/helixtrajectory/InitialGuessPoint");
    jfieldID jInitialGuessPointClassX = env->GetFieldID(jInitialGuessPointClass, "x", "D");
    jfieldID jInitialGuessPointClassY = env->GetFieldID(jInitialGuessPointClass, "y", "D");
    jclass jSwerveDriveClass = env->FindClass("org/team2363/helixtrajectory/SwerveDrive");
    jfieldID jSwerveDriveWheelbaseX = env->GetFieldID(jSwerveDriveClass, "wheelbaseX", "D");
    jfieldID jSwerveDriveWheelbaseY = env->GetFieldID(jSwerveDriveClass, "wheelbaseY", "D");
    jfieldID jSwerveDriveLength = env->GetFieldID(jSwerveDriveClass, "length", "D");
    jfieldID jSwerveDriveWidth = env->GetFieldID(jSwerveDriveClass, "width", "D");
    jfieldID jSwerveDriveMass = env->GetFieldID(jSwerveDriveClass, "mass", "D");
    jfieldID jSwerveDriveMoi = env->GetFieldID(jSwerveDriveClass, "moi", "D");
    jfieldID jSwerveDriveOmegaMax = env->GetFieldID(jSwerveDriveClass, "omegaMax", "D");
    jfieldID jSwerveDriveTauMax = env->GetFieldID(jSwerveDriveClass, "tauMax", "D");
    jfieldID jSwerveDriveWheelRadius = env->GetFieldID(jSwerveDriveClass, "wheelRadius", "D");
    jclass jObstacleClass = env->FindClass("org/team2363/helixtrajectory/Obstacle");
    jfieldID jObstacleClassSafetyDistance = env->GetFieldID(jObstacleClass, "safetyDistance", "D");
    jmethodID jObstacleClassLengthMethod = env->GetMethodID(jObstacleClass, "length", "()I");
    jmethodID jObstacleClassGetMethod = env->GetMethodID(jObstacleClass, "get", "(I)Lorg/team2363/helixtrajectory/ObstaclePoint;");
    jclass jObstaclePointClass = env->FindClass("org/team2363/helixtrajectory/ObstaclePoint");
    jfieldID jObstaclePointClassX = env->GetFieldID(jObstaclePointClass, "x", "D");
    jfieldID jObstaclePointClassY = env->GetFieldID(jObstaclePointClass, "y", "D");

    size_t waypointCount = env->CallIntMethod(jPath, jPathClassLengthMethod);

    std::vector<Waypoint> waypoints;
    waypoints.reserve(waypointCount);
    for (size_t i = 0; i < waypointCount; i++) {
        jobject jWaypoint = env->CallObjectMethod(jPath, jPathClassGetMethod, i);
        jdouble x = env->GetDoubleField(jWaypoint, jWaypointClassX);
        jdouble y = env->GetDoubleField(jWaypoint, jWaypointClassY);
        jdouble heading = env->GetDoubleField(jWaypoint, jWaypointClassHeading);
        jdouble vx = env->GetDoubleField(jWaypoint, jWaypointClassVX);
        jdouble vy = env->GetDoubleField(jWaypoint, jWaypointClassVY);
        jdouble omega = env->GetDoubleField(jWaypoint, jWaypointClassOmega);
        jboolean xConstrained = env->GetBooleanField(jWaypoint, jWaypointClassXConstrained);
        jboolean yConstrained = env->GetBooleanField(jWaypoint, jWaypointClassYConstrained);
        jboolean headingConstrained = env->GetBooleanField(jWaypoint, jWaypointClassHeadingConstrained);
        jboolean vxConstrained = env->GetBooleanField(jWaypoint, jWaypointClassVXConstrained);
        jboolean vyConstrained = env->GetBooleanField(jWaypoint, jWaypointClassVYConstrained);
        jboolean vMagnitudeConstrained = env->GetBooleanField(jWaypoint, jWaypointClassVMagnitudeConstrained);
        jboolean omegaConstrained = env->GetBooleanField(jWaypoint, jWaypointClassOmegaConstrained);
        size_t initialGuessPointCount = env->CallIntMethod(jWaypoint, jWaypointClassInitialGuessPointsLengthMethod);
        std::vector<InitialGuessPoint> initialGuessPoints;
        initialGuessPoints.reserve(initialGuessPointCount);
        for (size_t initialGuessPointIndex = 0; initialGuessPointIndex < initialGuessPointCount; initialGuessPointIndex++) {
            jobject jInitialGuessPoint = env->CallObjectMethod(jWaypoint, jWaypointClassGetInitialGuessPointMethod, initialGuessPointIndex);
            jdouble initialGuessPointX = env->GetDoubleField(jInitialGuessPoint, jInitialGuessPointClassX);
            jdouble initialGuessPointY = env->GetDoubleField(jInitialGuessPoint, jInitialGuessPointClassY);
            initialGuessPoints.push_back({initialGuessPointX, initialGuessPointY});
        }
        waypoints.push_back({x, y, heading, vx, vy, omega,
                static_cast<bool>(xConstrained),
                static_cast<bool>(yConstrained),
                static_cast<bool>(headingConstrained),
                static_cast<bool>(vxConstrained),
                static_cast<bool>(vyConstrained),
                static_cast<bool>(vMagnitudeConstrained),
                static_cast<bool>(omegaConstrained),
                initialGuessPoints});
    }
    Path path = waypoints;

    SwerveDrive drive = {
        env->GetDoubleField(jDrive, jSwerveDriveWheelbaseX),
        env->GetDoubleField(jDrive, jSwerveDriveWheelbaseY),
        env->GetDoubleField(jDrive, jSwerveDriveLength),
        env->GetDoubleField(jDrive, jSwerveDriveWidth),
        env->GetDoubleField(jDrive, jSwerveDriveMass),
        env->GetDoubleField(jDrive, jSwerveDriveMoi),
        env->GetDoubleField(jDrive, jSwerveDriveOmegaMax),
        env->GetDoubleField(jDrive, jSwerveDriveTauMax),
        env->GetDoubleField(jDrive, jSwerveDriveWheelRadius)};

    size_t obstacleCount = env->GetArrayLength(jObstacles);
    std::vector<Obstacle> obstacles;
    obstacles.reserve(obstacleCount);
    for (size_t i = 0; i < obstacleCount; i++) {
        jobject jObstacle = env->GetObjectArrayElement(jObstacles, i);
        double safetyDistance = env->GetDoubleField(jObstacle, jObstacleClassSafetyDistance);
        size_t obstaclePointCount = env->CallIntMethod(jObstacle, jObstacleClassLengthMethod);
        std::vector<ObstaclePoint> obstaclePoints;
        obstaclePoints.reserve(obstaclePointCount);
        for (size_t obstaclePointIndex = 0; obstaclePointIndex < obstaclePointCount; obstaclePointIndex++) {
            jobject jObstaclePoint = env->CallObjectMethod(jObstacle, jObstacleClassGetMethod, obstaclePointIndex);
            jdouble jObstaclePointX = env->GetDoubleField(jObstaclePoint, jObstaclePointClassX);
            jdouble jObstaclePointY = env->GetDoubleField(jObstaclePoint, jObstaclePointClassY);
            obstaclePoints.push_back({jObstaclePointX, jObstaclePointY});
        }
        obstacles.push_back(Obstacle(safetyDistance, obstaclePoints));
    }

    TrajectoryGenerator gen(drive, path, obstacles);
    std::unique_ptr<Trajectory> traj = gen.Generate();
    size_t sampleCount = traj->samples.size();
    size_t sampleArrLength = sampleCount * 7;
    jdouble sampleArr[sampleArrLength];
    for (size_t i = 0; i < sampleCount; i++) {
        sampleArr[7*i    ] = traj->samples[i].ts;
        sampleArr[7*i + 1] = traj->samples[i].x;
        sampleArr[7*i + 2] = traj->samples[i].y;
        sampleArr[7*i + 3] = traj->samples[i].heading;
        sampleArr[7*i + 4] = traj->samples[i].vx;
        sampleArr[7*i + 5] = traj->samples[i].vy;
        sampleArr[7*i + 6] = traj->samples[i].omega;
    }

    jdoubleArray newArr = env->NewDoubleArray(sampleArrLength);
    env->SetDoubleArrayRegion(newArr, 0, sampleArrLength, (const jdouble*) &sampleArr);
    return newArr;
}