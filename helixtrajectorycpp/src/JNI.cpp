#include <jni.h>
#include "org_team2363_helixtrajectory_HelixTrajectoryInterface.h"

#include <iostream>

#include "Path.h"
#include "TrajectoryGenerator.h"
#include "TrajectoryUtil.h"

JNIEXPORT jdoubleArray JNICALL Java_org_team2363_helixtrajectory_HelixTrajectoryInterface_generateTrajectory
        (JNIEnv *env, jobject jObj, jobject jDrive, jobject jPath) {
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

    size_t waypointCount = env->CallIntMethod(jPath, jPathClassLengthMethod);

    std::vector<Waypoint> waypoints;
    waypoints.reserve(waypointCount);
    for (int i = 0; i < waypointCount; i++) {
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
        waypoints.push_back(Waypoint{
                static_cast<double>(x),
                static_cast<double>(y),
                static_cast<double>(heading),
                static_cast<double>(vx),
                static_cast<double>(vy),
                static_cast<double>(omega),
                static_cast<bool>(xConstrained),
                static_cast<bool>(yConstrained),
                static_cast<bool>(headingConstrained),
                static_cast<bool>(vxConstrained),
                static_cast<bool>(vyConstrained),
                static_cast<bool>(vMagnitudeConstrained),
                static_cast<bool>(omegaConstrained)});
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

    TrajectoryGenerator gen(drive);
    std::unique_ptr<Trajectory> traj = gen.Generate(path);
    size_t sampleCount = traj->samples.size();
    size_t sampleArrLength = sampleCount * 7;
    jdouble sampleArr[sampleArrLength];
    for (int i = 0; i < sampleCount; i++) {
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