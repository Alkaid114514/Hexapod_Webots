#pragma once
#include <webots/InertialUnit.hpp>
#include <webots/Robot.hpp>

#include "Vector3.h"

using namespace webots;
class IMU
{
private:
    InertialUnit* imu;
public:
    IMU(Robot* robot);
    IMU();
    ~IMU();

    float getRoll();
    float getPitch();
    float getYaw();
    Vector3 getRollPitchYaw();
};
