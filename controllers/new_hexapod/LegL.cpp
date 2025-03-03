#include "LegL.h"
#include "Hexapod.h"

LegL::LegL()
{
	this->coxaMotor = nullptr;
	this->femurMotor = nullptr;
	this->tibiaMotor = nullptr;
}
LegL::LegL(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia, Vector3 ctr2root, float ctr2rootTheta)
{
	this->coxaMotor = coxa;
	this->femurMotor = femur;
	this->tibiaMotor = tibia;
	this->ctr2root = ctr2root;
	this->ctr2rootTheta = ctr2rootTheta;
}


LegL::~LegL()
{
}

Vector3 LegL::fk(Vector3 angles)
{
	//angles.z -= (float)M_PI / 3.0f;
	float tmp = (COXA_LEN + FEMUR_LEN * cos(angles.y) + TIBIA_LEN * cos(-angles.z - angles.y));
	auto v = Vector3(
		tmp * cos(angles.x),
		tmp * sin(angles.x),
		-FEMUR_LEN * sin(angles.y) + TIBIA_LEN * sin(-angles.z - angles.y)
	);
	//v.y = -v.y;
	//v.z = -v.z;
	return v;
}

Vector3 LegL::ik(Vector3 vector3)
{
	vector3.z = -vector3.z;
	float theta1 = atan2(vector3.y, vector3.x);
	float R = sqrt(vector3.x * vector3.x + vector3.y * vector3.y);
	float ar = atan2(-vector3.z, (R - COXA_LEN));
	float Lr = sqrt(vector3.z * vector3.z + (R - COXA_LEN) * (R - COXA_LEN));
	float a1 = acos(CLIP((FEMUR_LEN * FEMUR_LEN + Lr * Lr - TIBIA_LEN * TIBIA_LEN) / (2 * Lr * FEMUR_LEN), -1, 1));
	float theta2 = a1 - ar;
	float a2 = acos(CLIP((Lr * Lr + TIBIA_LEN * TIBIA_LEN - FEMUR_LEN * FEMUR_LEN) / (2 * Lr * TIBIA_LEN), -1, 1));
	float theta3 = -(a1 + a2);
	return Vector3(theta1, -theta2, theta3);
}

void LegL::setMotor(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia)
{
	this->coxaMotor = coxa;
	this->femurMotor = femur;
	this->tibiaMotor = tibia;
}

void LegL::setOmega(float* omega)
{
	this->coxaMotor->setVelocity(omega[0]);
	this->femurMotor->setVelocity(omega[1]);
	this->tibiaMotor->setVelocity(omega[2]);
}

void LegL::setCoxaOmega(float omega)
{
	this->coxaMotor->setVelocity(omega);
}

void LegL::setFemurOmega(float omega)
{
	this->femurMotor->setVelocity(omega);
}

void LegL::setTibiaOmega(float omega)
{
	this->tibiaMotor->setVelocity(omega);
}

void LegL::setLegTarget(Vector3 target)
{
	this->angles = this->ik(target);
}


void LegL::setBodyTarget(Vector3 target)
{
	setLegTarget(Hexapod::body2legCoord(target, ctr2root, ctr2rootTheta));
}

void LegL::setPose(Vector3 angles)
{
	angles.z += (float)M_PI / 3.0f;
	this->angles = angles;
}

void LegL::setCoxaPose(float angle)
{
	this->angles.x = angle;
}

void LegL::setFemurPose(float angle)
{
	this->angles.y = angle;
}

void LegL::setTibiaPose(float angle)
{
	angle += (float)M_PI / 3.0f;
	this->angles.z = angle;
}

void LegL::reInit()
{
	setPose(currentStandAngles);
}

void LegL::setHeight(float height)
{
	initStandTarget.z = height;
	this->currentStandAngles = this->ik(initStandTarget);
	this->reInit();
}

void LegL::startMotor()
{
	this->coxaMotor->setPosition(angles.x);
	this->femurMotor->setPosition(angles.y);
	this->tibiaMotor->setPosition(angles.z);
}


