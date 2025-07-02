// File:          new_hexapod.cpp
// Date:
// Description:
// Author:
// Modifications:
#include <iostream>

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
    
    robot->omega = -0.0f;
    robot->velocity = Vector3(0.0f, 0.0f, 0.0f);

    robot->setHeight(0.100459f);
    // robot->setHeight(0.13f);
    // robot->setYaw(0.3f);
    // robot->setRoll((0.3f));
    // robot->setPitch((0.2f));
    // robot->setBodyPosition(Vector3 (0.05f,0.05f,0.0f));
    // robot->MLleg->setYaw(1.2f);
    // robot->MRleg->setYaw(-1.2f);
    // robot->setBodyPosition(Vector3(0.08f, 0.0f, 0.0f));
    robot->reInit();
    // robot->MLleg->setBodyTarget(Vector3(-0.1f, 0.03f, 0.1f));
    // robot->MRleg->setBodyTarget(Vector3(0.1f, 0.03f, 0.1f));
    robot->startMove();
    

    // float omegas[3] = {2.0f,2.0f,2.0f};
    // robot->BRleg.setOmega(omegas);
    // robot->MRleg.setOmega(omegas);
    // robot->FRleg.setOmega(omegas);
    // robot->BLleg.setOmega(omegas);
    // robot->MLleg.setOmega(omegas);
    // robot->FLleg.setOmega(omegas);
    bool typeKey = false;
    
    while (robot->step(timeStep) != -1)
    {
        // robot->FLleg->setBodyTarget(Vector3(0.0f, 0.3f, 0.2f));
        // robot->BRleg->setBodyTarget(Vector3(0.0f, -0.3f, 0.2f));
        // robot->startMove();
        // robot->step(500);
        // robot->FLleg->reInit();
        // robot->BRleg->reInit();
        // robot->startMove();
        // robot->step(500);
        //
        // robot->MRleg->setBodyTarget(Vector3(0.2f, -0.1f, 0.2f));
        // robot->MLleg->setBodyTarget(Vector3(-0.2f, 0.1f, 0.2f));
        // robot->startMove();
        // robot->step(500);
        // robot->MRleg->setBodyTarget(Vector3(0.2f, 0.1f, 0.2f));
        // robot->MLleg->setBodyTarget(Vector3(-0.2f, -0.1f, 0.2f));
        // robot->startMove();
        // robot->step(500);
        // robot->MRleg->reInit();
        // robot->MLleg->reInit();
        // robot->startMove();
        // robot->step(500);
        //
        // robot->BLleg->setBodyTarget(Vector3(0.0f, -0.3f, 0.2f));
        // robot->FRleg->setBodyTarget(Vector3(0.0f, 0.3f, 0.2f));
        // robot->startMove();
        // robot->step(500);
        // robot->BLleg->reInit();
        // robot->FRleg->reInit();
        // robot->startMove();
        // robot->step(500);

        // robot->setPitch(0.2f);
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        //
        // robot->setPitch(-0.2f);
        //                 robot->reInit(); robot->startMove();
        //                 robot->step(300);
        //
        // robot->setPitch(0.0f);
        //                 robot->reInit(); robot->startMove();
        //         robot->step(300);
        //
        // robot->setRoll(0.2f);
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        //
        // robot->setRoll(-0.2f);
        //         robot->reInit(); robot->startMove();
        //         robot->step(300);
        //
        //
        // robot->setRoll(0.0f);
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        //
        // robot->setYaw(0.4f);
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        // robot->setYaw(-0.4f);
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        // robot->setYaw(0.0f);
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        //
        //
        // robot->setBodyPosition(Vector3(0.02f, 0.0f, 0.0f));
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        // robot->setBodyPosition(Vector3(-0.02f, 0.0f, 0.0f));
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        // robot->setBodyPosition(Vector3(-0.00f, 0.0f, 0.0f));
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        // robot->setBodyPosition(Vector3(0.00f, 0.02f, 0.0f));
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        // robot->setBodyPosition(Vector3(0.00f, -0.02f, 0.0f));
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        // robot->setBodyPosition(Vector3(-0.00f, 0.0f, 0.0f));
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        // robot->setHeight(0.13f);
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        // robot->setHeight(0.07f);
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        // robot->setHeight(0.100459f);
        // robot->reInit(); robot->startMove();
        // robot->step(300);

        // TODO
        // 等效 yaw bug
        // robot->setPitch(0.2f);
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        //
        // robot->setRoll(0.2f);
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        //
        // robot->setPitch(0.0f);
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        //
        // robot->setRoll(0.0f);
        // robot->reInit(); robot->startMove();
        // robot->step(300);
        
        // robot->moveTripod();
        // robot->moveRipple();
        // robot->moveWave();

        // robot->balance();
        // robot->reInit();
        // robot->startMove();
        typeKey = false;
        Vector3 vector3 = Vector3();
        bool arr[256] = {false};
        arr[keyboard->getKey()] = true;
        arr[keyboard->getKey()] = true;
        arr[keyboard->getKey()] = true;
        arr[keyboard->getKey()] = true;
        arr[keyboard->getKey()] = true;
        arr[keyboard->getKey()] = true;
        arr[keyboard->getKey()] = true;
        if (arr['A'])
        {
            vector3.x -= 1.0f;
            typeKey = true;
        }
        if (arr['D'])
        {
            vector3.x += 1.0f;
            typeKey = true;
        }
        if (arr['W'])
        {
            vector3.y += 1.0f;
            typeKey = true;
        }
        if (arr['S'])
        {
            vector3.y -= 1.0f;
            typeKey = true;
        }
        if (typeKey)
        {
            robot->velocity = vector3 / vector3.magnitude() * 0.1f;
        }
        else
        {
            robot->velocity = vector3;
        }
        
        
        float omega = 0.0f;
        if (arr['Q'])
        {
            omega += 0.2f;
        }
        if (arr['E'])
        {
            omega -= 0.2f;
        }
        if (arr['R'])
        {
           robot->reInit();
            robot->startMove();
        }
        robot->omega = omega;
        robot->moveTripod();
        // robot->balance();
        // robot->move4plus2();
        
        // std::cout << "x "  << robot->MRleg.currentStandBodyTarget.x << " y " << robot->MRleg.currentStandBodyTarget.y << std::endl;
        // robot->checkIsOnGround();
        // robot->toGround();
        // robot->reInit();
        // robot->startMove();
    }

    // Enter here exit cleanup code.

    delete robot;
    return 0;
}
