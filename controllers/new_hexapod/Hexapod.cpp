#include "Hexapod.h"

Hexapod::Hexapod() : Robot()
{
	BRleg = Leg(getMotor("M_BR_COXA"), getMotor("M_BR_FEMUR"), getMotor("M_BR_TIBIA"));
	MRleg = Leg(getMotor("M_MR_COXA"), getMotor("M_MR_FEMUR"), getMotor("M_MR_TIBIA"));
	FRleg = Leg(getMotor("M_FR_COXA"), getMotor("M_FR_FEMUR"), getMotor("M_FR_TIBIA"));
	BLleg = Leg(getMotor("M_BL_COXA"), getMotor("M_BL_FEMUR"), getMotor("M_BL_TIBIA"));
	MLleg = Leg(getMotor("M_ML_COXA"), getMotor("M_ML_FEMUR"), getMotor("M_ML_TIBIA"));
	FLleg = Leg(getMotor("M_FL_COXA"), getMotor("M_FL_FEMUR"), getMotor("M_FL_TIBIA"));

	currentStandBR = initStandBR = rfk(Vector3(0.0f, 0.0f, (float)M_PI / 3.0f));
	currentStandMR = initStandMR = rfk(Vector3(0.0f, 0.0f, (float)M_PI / 3.0f));
	currentStandFR = initStandFR = rfk(Vector3(0.0f, -0.0f, (float)M_PI / 3.0f));
	currentStandBL = initStandBL = lfk(Vector3(0.0f, 0.0f, -(float)M_PI / 3.0f));
	currentStandML = initStandML = lfk(Vector3(0.0f, 0.0f, -(float)M_PI / 3.0f));
	currentStandFL = initStandFL = lfk(Vector3(-0.0f, 0.0f, -(float)M_PI / 3.0f));
	/*currentStandBR = initStandBR = rfk(Vector3(0.0f, 0.0f, 0.0f));
	currentStandMR = initStandMR = rfk(Vector3(0.0f, 0.0f, 0.0f));
	currentStandFR = initStandFR = rfk(Vector3(0.0f, -0.0f, 0.0f));
	currentStandBL = initStandBL = lfk(Vector3(0.0f, 0.0f, 0.0f));
	currentStandML = initStandML = lfk(Vector3(0.0f, 0.0f, 0.0f));
	currentStandFL = initStandFL = lfk(Vector3(-0.3f, 0.3f, 0.0f));*/
	std::cout << "initStandFL " << initStandFL.x << " " << initStandFL.y << " " << initStandFL.z << std::endl;
	auto i = lik(initStandFL);
	auto r = this->body2legCoord(this->leg2bodyCoord(initStandFL, ctr2FLroot, ctr2FLrootTheta), ctr2FLroot, ctr2FLrootTheta);
	std::cout << "i " << i.x << " " << i.y << " " << i.z << std::endl;
	std::cout << "r " << r.x << " " << r.y << " " << r.z << std::endl;
	currentHeight = initHeight = initStandFR.z;
	std::cout << "initHeight " << initHeight << std::endl;
}

Hexapod::~Hexapod()
{
}

void Hexapod::setPose(Vector3 backRightAngles, Vector3 middleRightAngles, Vector3 frontRightAngles,
    Vector3 backLeftAngles, Vector3 middleLeftAngles, Vector3 frontLeftAngles) {
	setBRpose(backRightAngles);
	setMRpose(middleRightAngles);
	setFRpose(frontRightAngles);
	setBLpose(backLeftAngles);
	setMLpose(middleLeftAngles);
	setFLpose(frontLeftAngles);
}

void Hexapod::setBRpose(Vector3 angles)
{
	angles.z -= (float)M_PI / 3.0f;
	//angles.x = -angles.x;
	BRleg.setRadAngles(angles);
}

void Hexapod::setMRpose(Vector3 angles)
{
	angles.z -= (float)M_PI / 3.0f;
	//angles.x = -angles.x;
	MRleg.setRadAngles(angles);
}

void Hexapod::setFRpose(Vector3 angles)
{
	angles.z -= (float)M_PI / 3.0f;
	//angles.x = -angles.x;
	FRleg.setRadAngles(angles);
}

void Hexapod::setBLpose(Vector3 angles)
{
	angles.z += (float)M_PI / 3.0f;
	//angles.x = -angles.x;
	BLleg.setRadAngles(angles);
}

void Hexapod::setMLpose(Vector3 angles)
{
	angles.z += (float)M_PI / 3.0f;
	//angles.x = -angles.x;
	MLleg.setRadAngles(angles);
}

void Hexapod::setFLpose(Vector3 angles)
{
	angles.z += (float)M_PI / 3.0f;
	//angles.x = -angles.x;
	FLleg.setRadAngles(angles);
}


Vector3 Hexapod::lik(Vector3 vector3)
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
    return Vector3(theta1,-theta2,theta3);
}


Vector3 Hexapod::rik(Vector3 vector3)
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
	return Vector3(theta1, theta2, -theta3);
}

Vector3 Hexapod::lfk(Vector3 angles)
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

Vector3 Hexapod::rfk(Vector3 angles)
{
	//angles.z -= (float)M_PI / 3.0f;
	float tmp = (COXA_LEN + FEMUR_LEN * cos(angles.y) + TIBIA_LEN * cos(angles.z - angles.y));
	auto v = Vector3(
		tmp * cos(angles.x),
		tmp * sin(angles.x),
		-FEMUR_LEN * sin(angles.y) + TIBIA_LEN * sin(angles.z - angles.y)
	);
	//v.y = -v.y;
	//v.z = -v.z;
	return v;
}

Vector3 Hexapod::leg2bodyCoord(Vector3 relevant, Vector3 bias, float theta)
{
	relevant.z = -relevant.z;
	relevant.y = -relevant.y;
	auto v = Vector3(
		cos(theta) * relevant.x - sin(theta) * relevant.y,
		sin(theta) * relevant.x + cos(theta) * relevant.y,
		relevant.z
	);
	return v + bias;
}



Vector3 Hexapod::body2legCoord(Vector3 absolute, Vector3 bias, float theta)
{
	auto v = Vector3(
		cos(theta) * (absolute.x - bias.x) + sin(theta) * (absolute.y - bias.y),
		-sin(theta) * (absolute.x - bias.x) + cos(theta) * (absolute.y - bias.y),
		absolute.z - bias.z
	);
	v.y = -v.y;
	v.z = -v.z;
	return v;
}

void Hexapod::move(Vector3 velocity, float omega,float timeStep)
{
	if (velocity == Vector3() && omega == 0.0f)
	{
		return;
	}

	Vector3 w = Vector3(0.0f, 0.0f, omega);
	Vector3 r0 = velocity.cross(w) / (omega * omega);


}

Vector3 Hexapod::getNextBodyTarget(Vector3 velocity, float omega, Vector3 r0,Vector3 initial, Vector3 legBias, float timeStep)
{
	initial.z = 0.0f;
	Vector3 pc = r0 + initial + legBias;
	float theta = omega * timeStep;
	Vector3 pt = Vector3(
		cos(theta) * (pc.x) + sin(theta) * (pc.y),
		-sin(theta) * (pc.x) + cos(theta) * (pc.y),
		pc.z
	);
	return pt - r0;
}

void Hexapod::setTargets(Vector3 BRtarget, Vector3 MRtarget, Vector3 FRtarget, Vector3 BLtarget, Vector3 MLtarget, Vector3 FLtarget)
{
	setBLbodyTarget(BLtarget);
	setMLbodyTarget(MLtarget);
	setFLbodyTarget(FLtarget);
	setBRbodyTarget(BRtarget);
	setMRbodyTarget(MRtarget);
	setFRbodyTarget(FRtarget);
}

void Hexapod::setBRbodyTarget(Vector3 target)
{
	this->setBRpose(rik(body2legCoord(target, ctr2BRroot, ctr2BRrootTheta)));
}

void Hexapod::setMRbodyTarget(Vector3 target)
{
	this->setMRpose(rik(body2legCoord(target, ctr2MRroot, ctr2MRrootTheta)));
}

void Hexapod::setFRbodyTarget(Vector3 target)
{
	this->setFRpose(rik(body2legCoord(target, ctr2FRroot, ctr2FRrootTheta)));
}

void Hexapod::setBLbodyTarget(Vector3 target)
{
	this->setBLpose(lik(body2legCoord(target, ctr2BLroot, ctr2BLrootTheta)));
}

void Hexapod::setMLbodyTarget(Vector3 target)
{
	this->setMLpose(lik(body2legCoord(target, ctr2MLroot, ctr2MLrootTheta)));
}

void Hexapod::setFLbodyTarget(Vector3 target)
{
	this->setFLpose(lik(body2legCoord(target, ctr2FLroot, ctr2FLrootTheta)));
}


void Hexapod::setBRlegTarget(Vector3 target)
{
	this->setBRpose(rik(target));
}

void Hexapod::setMRlegTarget(Vector3 target)
{
	this->setMRpose(rik(target));
}

void Hexapod::setFRlegTarget(Vector3 target)
{
	this->setFRpose(rik(target));
}

void Hexapod::setBLlegTarget(Vector3 target)
{
	this->setBLpose(lik(target));
}

void Hexapod::setMLlegTarget(Vector3 target)
{
	this->setMLpose(lik(target));
}

void Hexapod::setFLlegTarget(Vector3 target)
{
	this->setFLpose(lik(target));
}

void Hexapod::setHeight(float height)
{
	currentHeight = height;
	currentStandBR.z = height;
	currentStandMR.z = height;
	currentStandFR.z = height;
	currentStandBL.z = height;
	currentStandML.z = height;
	currentStandFL.z = height;
	std::cout << "currentStandFL.z " << currentStandFL.z << std::endl;
	auto i = lik(currentStandFL);
	std::cout << "ic " << i.x << " " << i.y << " " << i.z << std::endl;
	setBRlegTarget(currentStandBR);
	setMRlegTarget(currentStandMR);
	setFRlegTarget(currentStandFR);
	setBLlegTarget(currentStandBL);
	setMLlegTarget(currentStandML);
	setFLlegTarget(currentStandFL);
}

void Hexapod::startMove()
{
	BRleg.startMotor();
	MRleg.startMotor();
	FRleg.startMotor();
	BLleg.startMotor();
	MLleg.startMotor();
	FLleg.startMotor();
}
