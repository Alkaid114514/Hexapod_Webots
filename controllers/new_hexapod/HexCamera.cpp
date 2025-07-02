#include "HexCamera.h"

HexCamera::HexCamera(Robot* robot)
{
    camera = robot->getCamera("camera");
    camera->enable((int)robot->getBasicTimeStep());
}
HexCamera::HexCamera()
{
}
HexCamera::~HexCamera()
{
    camera->disable();
}
