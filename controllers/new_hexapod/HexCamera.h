#pragma once
#include <webots/Camera.hpp>
#include <webots/Robot.hpp>
using namespace webots;
class HexCamera
{
private:
    Camera* camera;
public:
    HexCamera(Robot* robot);
    HexCamera();
    ~HexCamera();
    void enable(int samplingPeriod);
    void disable();
    void setFov(double fov);
    void setFocalDistance(double focalDistance);
};
