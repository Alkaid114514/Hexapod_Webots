#include "IMU.h"

IMU::IMU(Robot* robot)
{
    imu = robot->getInertialUnit("imu");
    imu->enable((int)robot->getBasicTimeStep());
}

IMU::IMU()
{
}

IMU::~IMU()
{
}

float IMU::getRoll()
{
    return (float)imu->getRollPitchYaw()[0];
}

float IMU::getPitch()
{
    return (float)imu->getRollPitchYaw()[1];
}

float IMU::getYaw()
{
    return (float)imu->getRollPitchYaw()[2];
}

Vector3 IMU::getRollPitchYaw()
{
    auto tmp = imu->getRollPitchYaw();
    return Vector3((float)tmp[0], (float)tmp[1], (float)tmp[2]);
}
