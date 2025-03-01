
#include <webots/Motor.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include "Vector3.h"

class Leg
{
private:
	webots::Motor* coxaMotor;
	webots::Motor* femurMotor;
	webots::Motor* tibiaMotor;

	float angles[3] = {0.0f,0.0f,0.0f};

public:
	Leg();
	Leg(webots::Motor* coxa,webots::Motor* femur,webots::Motor* tibia);
	~Leg();
	void setMotor(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia);
	void setOmega(float* omega);
	void setCoxaOmega(float omega);
	void setFemurOmega(float omega);
	void setTibiaOmega(float omega);
	void setRadAngles(Vector3 angles);
	void setCoxaRadAngle(float angle);
	void setFemurRadAngle(float angle);
	void setTibiaRadAngle(float angle);
	void startMotor();
};

