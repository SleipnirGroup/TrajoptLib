#include <jni.h>
#include "org_team2363_helixtrajectory_HelixTrajectoryInterface.h"

#include <iostream>

#include "Path.h"
#include "TrajectoryGenerator.h"
#include "TrajectoryUtil.h"

JNIEXPORT jdoubleArray JNICALL Java_org_team2363_helixtrajectory_HelixTrajectoryInterface_generateTrajectory
        (JNIEnv *env, jobject obj, jdoubleArray driveArr, jdoubleArray pathArr) {
    using namespace helixtrajectory;
    size_t pathArrLength = env->GetArrayLength(pathArr);
    size_t waypointCount = pathArrLength / 3;
    jboolean isCopy;
    jdouble* driveDoubleArr = env->GetDoubleArrayElements(driveArr, &isCopy);
    jdouble* pathDoubleArr = env->GetDoubleArrayElements(pathArr, &isCopy);

    std::vector<Waypoint> waypoints;
    waypoints.reserve(waypointCount);
    for (int i = 0; i < waypointCount; i++) {
        waypoints.push_back({pathDoubleArr[3*i], pathDoubleArr[3*i+1], pathDoubleArr[3*i+2]});
    }
    Path path = waypoints;

    SwerveDrive drive = {
        driveDoubleArr[0], driveDoubleArr[1], driveDoubleArr[2], driveDoubleArr[3], driveDoubleArr[4],
        driveDoubleArr[5], driveDoubleArr[6], driveDoubleArr[7], driveDoubleArr[8]};

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