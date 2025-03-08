// File:          new_hexapod.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <webots/Robot.hpp>
#include "Hexapod.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <webots/Keyboard.hpp>
using namespace webots;

int main(int argc, char** argv)
{
    auto robot = new Hexapod();

    int timeStep = static_cast<int>(robot->getBasicTimeStep());

    /*Vector3 rightAngles = Vector3(-0.0f, -0.0f, -0.0f);
    Vector3 leftAngles = Vector3(0.0f, 0.0f, 0.0f);
    robot->setPose(rightAngles, rightAngles, rightAngles, leftAngles, leftAngles, leftAngles);
    robot->startMove();*/

    Keyboard* keyboard = robot->getKeyboard();
    keyboard->enable(timeStep);
    
    robot->omega = -0.1f;
    robot->velocity = Vector3(0.0f, 0.05f, 0.0f);

    robot->setHeight(0.100459f);
    // robot->setHeight(0.13f);
    // robot->setYaw(0.3f);
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
        // robot->moveRipple();
        // robot->moveWave();
        
        // Vector3 vector3 = Vector3();
        // bool arr[256] = {false};
        // arr[keyboard->getKey()] = true;
        // arr[keyboard->getKey()] = true;
        // arr[keyboard->getKey()] = true;
        // arr[keyboard->getKey()] = true;
        // arr[keyboard->getKey()] = true;
        // arr[keyboard->getKey()] = true;
        // arr[keyboard->getKey()] = true;
        // if (arr['A'])
        // {
        //     vector3.x -= 0.05f;
        // }
        // if (arr['D'])
        // {
        //     vector3.x += 0.05f;
        // }
        // if (arr['W'])
        // {
        //     vector3.y += 0.05f;
        // }
        // if (arr['S'])
        // {
        //     vector3.y -= 0.05f;
        // }
        // robot->velocity = vector3;
        //
        // float omega = 0.0f;
        // if (arr['Q'])
        // {
        //     omega += 0.1f;
        // }
        // if (arr['E'])
        // {
        //     omega -= 0.1f;
        // }
        // robot->omega = omega;
        robot->moveTripod();
    }

    // Enter here exit cleanup code.

    delete robot;
    return 0;
}
