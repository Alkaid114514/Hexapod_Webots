#include "Hexapod.h"

Hexapod::Hexapod() : Robot()
{
	BRleg = Leg(getMotor("M_BR_COXA"), getMotor("M_BR_FEMUR"), getMotor("M_BR_TIBIA"));
	MRleg = Leg(getMotor("M_MR_COXA"), getMotor("M_MR_FEMUR"), getMotor("M_MR_TIBIA"));
	FRleg = Leg(getMotor("M_FR_COXA"), getMotor("M_FR_FEMUR"), getMotor("M_FR_TIBIA"));
	BLleg = Leg(getMotor("M_BL_COXA"), getMotor("M_BL_FEMUR"), getMotor("M_BL_TIBIA"));
	MLleg = Leg(getMotor("M_ML_COXA"), getMotor("M_ML_FEMUR"), getMotor("M_ML_TIBIA"));
	FLleg = Leg(getMotor("M_FL_COXA"), getMotor("M_FL_FEMUR"), getMotor("M_FL_TIBIA"));

	initialBR = fk(Vector3(0.0f, 0.0f, 0.0f));
	initialMR = fk(Vector3(0.0f, 0.0f, 0.0f));
	initialFR = fk(Vector3(0.0f, 0.0f, 0.0f));
	initialBL = fk(Vector3(0.0f, 0.0f, 0.0f));
	initialML = fk(Vector3(0.0f, 0.0f, 0.0f));
	initialFL = fk(Vector3(0.0f, 0.0f, 0.0f));
}

Hexapod::~Hexapod()
{
}

void Hexapod::setPose(Vector3 backRightAngles, Vector3 middleRightAngles, Vector3 frontRightAngles,
    Vector3 backLeftAngles, Vector3 middleLeftAngles, Vector3 frontLeftAngles) {
        //backRightAngles[2] += M_PI / 3.0f;
        BRleg.setRadAngles(backRightAngles);
        //middleRightAngles[2] += M_PI / 3.0f;
        MRleg.setRadAngles(middleRightAngles);
        //frontRightAngles[2] += M_PI / 3.0f;
        FRleg.setRadAngles(frontRightAngles);
        //backLeftAngles[2] -= M_PI / 3.0f;
        BLleg.setRadAngles(backLeftAngles);
        //middleLeftAngles[2] -= M_PI / 3.0f;
        MLleg.setRadAngles(middleLeftAngles);
        //frontLeftAngles[2] -= M_PI / 3.0f;
        FLleg.setRadAngles(frontLeftAngles);
}


Vector3 Hexapod::ik(Vector3 vector3)
{
    float theta1 = atan2(vector3.y, vector3.x);
    float R = sqrt(vector3.x * vector3.x + vector3.y * vector3.y);
    float ar = atan2(-vector3.z, (R - COXA_LEN));
    float Lr = sqrt(vector3.z * vector3.z + (R - COXA_LEN) * (R - COXA_LEN));
    float a1 = acos(CLIP((FEMUR_LEN * FEMUR_LEN + Lr * Lr - TIBIA_LEN * TIBIA_LEN) / (2 * Lr * FEMUR_LEN), -1, 1));
    float theta2 = a1 - ar;
    float a2 = acos(CLIP((Lr * Lr + TIBIA_LEN * TIBIA_LEN - FEMUR_LEN * FEMUR_LEN) / (2 * Lr * TIBIA_LEN), -1, 1));
    float theta3 = -(a1 + a2);
    return Vector3(theta1,theta2,theta3);
}


Vector3 Hexapod::fk(Vector3 angles)
{
	float tmp = (COXA_LEN + FEMUR_LEN * cos(angles.y) + TIBIA_LEN * cos(angles.y + angles.z));
	return Vector3(
		tmp * cos(angles.x), 
		tmp * sin(angles.x), 
		FEMUR_LEN * sin(angles.y) + TIBIA_LEN * sin(angles.y + angles.z)
	);
}

Vector3 Hexapod::leg2bodyCoord(Vector3 relevant, Vector3 bias, float theta)
{
	return Vector3(
		cos(theta) * relevant.x - sin(theta) * relevant.y + bias.x,
		sin(theta) * relevant.x + cos(theta) * relevant.y + bias.y,
		relevant.z + bias.z
	);
}



Vector3 Hexapod::body2legCoord(Vector3 absolute, Vector3 bias, float theta)
{
	return Vector3(
		cos(theta) * (absolute.x - bias.x) + sin(theta) * (absolute.y - bias.y),
		-sin(theta) * (absolute.x - bias.x) + cos(theta) * (absolute.y - bias.y),
		absolute.z - bias.z
	);
}

void Hexapod::move(Vector3 velocity, float omega)
{
	Vector3 v = velocity;
	v = v.normalize();
	Vector3 w = Vector3(0.0f, 0.0f, omega);
    Vector3 r = v.cross(w).normalize() * v.magnitude() / omega;

	Vector3 v1 = v + w.cross(Vector3(0, 0, 1).normalize());
	Vector3 v2 = v - w.cross(Vector3(0, 0, 1).normalize());
	Vector3 v3 = v + w.cross(Vector3(0, 0, 1).normalize());
	Vector3 v4 = v - w.cross(Vector3(0, 0, 1).normalize());
	Vector3 v5 = v + w.cross(Vector3(0, 0, 1).normalize());
	Vector3 v6 = v - w.cross(Vector3(0, 0, 1).normalize());
	/*float* angles1 = ik(v1);
	float* angles2 = ik(v2);
	float* angles3 = ik(v3);
	float* angles4 = ik(v4);
	float* angles5 = ik(v5);
	float* angles6 = ik(v6);*/
	//setPose(angles1, angles2, angles3, angles4, angles5, angles6);
}

void Hexapod::setTargets(Vector3 BRtarget, Vector3 MRtarget, Vector3 FRtarget, Vector3 BLtarget, Vector3 MLtarget, Vector3 FLtarget)
{
	setBLtarget(BLtarget);
	setMLtarget(MLtarget);
	setFLtarget(FLtarget);
	setBRtarget(BRtarget);
	setMRtarget(MRtarget);
	setFRtarget(FRtarget);
}

void Hexapod::setBRtarget(Vector3 target)
{
	this->BRleg.setRadAngles(-ik(body2legCoord(target,ctr2BRroot,ctr2BRrootTheta)));
}

void Hexapod::setMRtarget(Vector3 target)
{
	this->MRleg.setRadAngles(-ik(body2legCoord(target, ctr2MRroot, ctr2MRrootTheta)));
}

void Hexapod::setFRtarget(Vector3 target)
{
	this->FRleg.setRadAngles(-ik(body2legCoord(target, ctr2FRroot, ctr2FRrootTheta)));
}

void Hexapod::setBLtarget(Vector3 target)
{
	this->BLleg.setRadAngles(ik(body2legCoord(target, ctr2BLroot, ctr2BLrootTheta)));
}

void Hexapod::setMLtarget(Vector3 target)
{
	this->MLleg.setRadAngles(ik(body2legCoord(target, ctr2MLroot, ctr2MLrootTheta)));
}

void Hexapod::setFLtarget(Vector3 target)
{
	this->FLleg.setRadAngles(ik(body2legCoord(target, ctr2FLroot, ctr2FLrootTheta)));
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
