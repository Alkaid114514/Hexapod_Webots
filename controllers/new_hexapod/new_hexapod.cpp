// File:          new_hexapod.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <webots/Robot.hpp>
#include "Hexapod.h"
#define _USE_MATH_DEFINES
#include <cmath>
using namespace webots;

int main(int argc, char** argv)
{
    auto robot = new Hexapod();

    int timeStep = static_cast<int>(robot->getBasicTimeStep());

    /*Vector3 rightAngles = Vector3(-0.0f, -0.0f, -0.0f);
    Vector3 leftAngles = Vector3(0.0f, 0.0f, 0.0f);
    robot->setPose(rightAngles, rightAngles, rightAngles, leftAngles, leftAngles, leftAngles);
    robot->startMove();*/

    robot->omega = 0.1f;
    robot->velocity = Vector3(0.0f, 0.0f, 0.0f);

    // robot->setHeight(0.100459f);
    // robot->setHeight(0.07f);
    // robot->setYaw(0.4f);
    // robot->setRoll(0.5f);
    // robot->setPitch(0.05f);
    // robot->reInit();
    // robot->startMove();


    while (robot->step(timeStep) != -1)
    {
        // robot->FLleg.setBodyTarget(Vector3(0.0f, 0.3f, 0.2f));
        // robot->BRleg.setBodyTarget(Vector3(0.0f, -0.3f, 0.2f));
        // robot->startMove();
        // robot->step(500);
        // robot->FLleg.reInit();
        // robot->BRleg.reInit();
        // robot->startMove();
        // robot->step(500);
        //
        // robot->MRleg.setBodyTarget(Vector3(0.2f, 0.1f, 0.2f));
        // robot->MLleg.setBodyTarget(Vector3(-0.2f, 0.1f, 0.2f));
        // robot->startMove();
        // robot->step(500);
        // robot->MRleg.reInit();
        // robot->MLleg.reInit();
        // robot->startMove();
        // robot->step(500);
        //
        // robot->BLleg.setBodyTarget(Vector3(0.0f, -0.3f, 0.2f));
        // robot->FRleg.setBodyTarget(Vector3(0.0f, 0.3f, 0.2f));
        // robot->startMove();
        // robot->step(500);
        // robot->BLleg.reInit();
        // robot->FRleg.reInit();
        // robot->startMove();
        // robot->step(500);
        // robot->moveTripod();
        // robot->moveWave();
        robot->moveRipple();
    }

    // Enter here exit cleanup code.

    delete robot;
    return 0;
}
