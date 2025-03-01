// File:          new_hexapod.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include "Hexapod.h"
#define _USE_MATH_DEFINES
#include <math.h>
// All the webots classes are defined in the "webots" namespace
using namespace webots;


// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char** argv) {
	// create the Robot instance.
	Hexapod* robot = new Hexapod();

	// get the time step of the current world.
	int timeStep = (int)robot->getBasicTimeStep();

	// You should insert a getDevice-like function in order to get the
	// instance of a device of the robot. Something like:
	//  Motor *motor = robot->getMotor("motorname");
	//  DistanceSensor *ds = robot->getDistanceSensor("dsname");
	//  ds->enable(timeStep);

	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	Vector3 rightAngles = Vector3(-0.0f, -0.0f, -0.0f);
	Vector3 leftAngles = Vector3(0.0f, 0.0f, 0.0f);
	robot->setPose(rightAngles, rightAngles, rightAngles, leftAngles, leftAngles, leftAngles);
	robot->startMove();

	float omega = 0.2f;
	Vector3 velocity = Vector3(0,1,0);

	/*robot->setTargets(
		Vector3(0.3f * 2.0f / 5.0f, -0.1f * sqrt(3), -0.1f),
		Vector3(0.3f * 3.0f / 5.0f, 0.0f, -0.1f),
		Vector3(0.3f * 2.0f / 5.0f, 0.1f * sqrt(3), -0.1f),
		Vector3(-0.3f * 2.0f / 5.0f, -0.1f * sqrt(3), -0.1f),
		Vector3(-0.3f * 3.0f / 5.0f, 0.0f, -0.1f),
		Vector3(-0.3f * 2.0f / 5.0f, 0.1f * sqrt(3), -0.1f)
	);*/


	//robot->startMove();

	/*Vector3 originA = Vector3(0.3f, 0.0f, -0.3f);
	Vector3 r = robot->body2legCoord(originA, robot->ctr2BRroot, robot->ctr2BRrootTheta);
	Vector3 a = robot->leg2bodyCoord(r, robot->ctr2BRroot, robot->ctr2BRrootTheta);
	std::cout << "originA  " << originA.x << " " << originA.y << " " << originA.z << std::endl;
	std::cout << "r        " << r.x << " " << r.y << " " << r.z << std::endl;
	std::cout << "a        " << a.x << " " << a.y << " " << a.z << std::endl;*/


	while (robot->step(timeStep) != -1) {
		// Read the sensors:
		// Enter here functions to read sensor data, like:
		//  double val = ds->getValue();

		// Process sensor data here.

		// Enter here functions to send actuator commands, like:
		//  motor->setPosition(10.0);
	};

	// Enter here exit cleanup code.

	delete robot;
	return 0;
}
