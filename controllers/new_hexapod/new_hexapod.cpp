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

    robot->setHeight(0.100459f);
    // robot->setHeight(0.13f);
    robot->setYaw(0.3f);
    // robot->setRoll((0.2f));
    
    robot->reInit();
    robot->startMove();
    

    // float omegas[3] = {2.0f,2.0f,2.0f};
    // robot->BRleg.setOmega(omegas);
    // robot->MRleg.setOmega(omegas);
    // robot->FRleg.setOmega(omegas);
    // robot->BLleg.setOmega(omegas);
    // robot->MLleg.setOmega(omegas);
    // robot->FLleg.setOmega(omegas);

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
        robot->moveWave();
    }

    // Enter here exit cleanup code.

    delete robot;
    return 0;
}
