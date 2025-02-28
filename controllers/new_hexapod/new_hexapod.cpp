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

<<<<<<< HEAD
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // 
    // 
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
      float rightAngles[3] = { -0.0f, 0.2f, 0.2f };
	  float leftAngles[3] = { 0.0f, -0.2f, -0.2f };
	  robot->setPose(rightAngles, rightAngles, rightAngles, leftAngles, leftAngles, leftAngles);
    // Process sensor data here.
=======
	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	float rightAngles[3] = { -0.0f, 0.2f, 0.2f };
	float leftAngles[3] = { 0.0f, -0.2f, -0.2f };
	robot->setPose(rightAngles, rightAngles, rightAngles, leftAngles, leftAngles, leftAngles);
>>>>>>> origin/main

	float omega = 0.2;
	Vector3 velocity = Vector3(0,1,0);


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
