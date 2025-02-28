
#include <webots/Motor.hpp>
#define _USE_MATH_DEFINES
#include <math.h>

class Leg
{
private:
	webots::Motor* coxaMotor;
	webots::Motor* femurMotor;
	webots::Motor* tibiaMotor;

public:
	Leg();
	Leg(webots::Motor* coxa,webots::Motor* femur,webots::Motor* tibia);
	~Leg();
	void setMotor(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia);
	void setOmega(float* omega);
	void setCoxaOmega(float omega);
	void setFemurOmega(float omega);
	void setTibiaOmega(float omega);
	void setRadAngles(float* angles);
	void setCoxaRadAngle(float angle);
	void setFemurRadAngle(float angle);
	void setTibiaRadAngle(float angle);
};

