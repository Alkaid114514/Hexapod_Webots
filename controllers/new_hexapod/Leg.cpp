#include "Leg.h"
Leg::Leg()
{
}
Leg::Leg(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia)
{
	this->coxaMotor = coxa;
	this->femurMotor = femur;
	this->tibiaMotor = tibia;
}

Leg::~Leg()
{
}

void Leg::setMotor(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia)
{
	this->coxaMotor = coxa;
	this->femurMotor = femur;
	this->tibiaMotor = tibia;
}

void Leg::setOmega(float* omega)
{
	this->coxaMotor->setVelocity(omega[0]);
	this->femurMotor->setVelocity(omega[1]);
	this->tibiaMotor->setVelocity(omega[2]);
}

void Leg::setCoxaOmega(float omega)
{
	this->coxaMotor->setVelocity(omega);
}

void Leg::setFemurOmega(float omega)
{
	this->femurMotor->setVelocity(omega);
}

void Leg::setTibiaOmega(float omega)
{
	this->tibiaMotor->setVelocity(omega);
}

void Leg::setRadAngles(float* angles)
{
	this->coxaMotor->setPosition(angles[0]);
	this->femurMotor->setPosition(angles[1]);
	this->tibiaMotor->setPosition(angles[2]);
}

void Leg::setCoxaRadAngle(float angle)
{
	this->coxaMotor->setPosition(angle);
}

void Leg::setFemurRadAngle(float angle)
{
	this->femurMotor->setPosition(angle);
}

void Leg::setTibiaRadAngle(float angle)
{
	this->tibiaMotor->setPosition(angle);
}


