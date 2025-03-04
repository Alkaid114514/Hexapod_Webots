#include "Hexapod.h"

Hexapod::Hexapod() : Robot()
{
	BRleg = LegR(getMotor("M_BR_COXA"), getMotor("M_BR_FEMUR"), getMotor("M_BR_TIBIA"), ctr2BRroot, ctr2BRrootTheta);
	MRleg = LegR(getMotor("M_MR_COXA"), getMotor("M_MR_FEMUR"), getMotor("M_MR_TIBIA"), ctr2MRroot, ctr2MRrootTheta);
	FRleg = LegR(getMotor("M_FR_COXA"), getMotor("M_FR_FEMUR"), getMotor("M_FR_TIBIA"), ctr2FRroot, ctr2FRrootTheta);
	BLleg = LegL(getMotor("M_BL_COXA"), getMotor("M_BL_FEMUR"), getMotor("M_BL_TIBIA"), ctr2BLroot, ctr2BLrootTheta);
	MLleg = LegL(getMotor("M_ML_COXA"), getMotor("M_ML_FEMUR"), getMotor("M_ML_TIBIA"), ctr2MLroot, ctr2MLrootTheta);
	FLleg = LegL(getMotor("M_FL_COXA"), getMotor("M_FL_FEMUR"), getMotor("M_FL_TIBIA"), ctr2FLroot, ctr2FLrootTheta);
	
	currentHeight = initHeight = body2legCoord(FLleg.initStandBodyTarget,FLleg.ctr2root,FLleg.ctr2rootTheta).z;
}

Hexapod::~Hexapod()
{
}

void Hexapod::setPose(Vector3 BRangles, Vector3 MRangles, Vector3 FRangles,
	Vector3 BLangles, Vector3 MLangles, Vector3 FLangles) {
	BRleg.setPose(BRangles);
	MRleg.setPose(MRangles);
	FRleg.setPose(FRangles);
	BLleg.setPose(BLangles);
	MLleg.setPose(MLangles);
	FLleg.setPose(FLangles);
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

Vector3 Hexapod::getNextBodyTarget(Vector3 velocity, float omega, Vector3 r0,Vector3 initial, Vector3 legBias,float legBiasTheta, float timeStep)
{
	initial.z = 0.0f;
	Vector3 pc = r0 + leg2bodyCoord(initial,legBias, legBiasTheta);
	float theta = omega * timeStep;
	Vector3 pt = Vector3(
		cos(theta) * (pc.x) + sin(theta) * (pc.y),
		-sin(theta) * (pc.x) + cos(theta) * (pc.y),
		pc.z
	);
	return pt - r0;
}

void Hexapod::setBodyTargets(Vector3 BRtarget, Vector3 MRtarget, Vector3 FRtarget, Vector3 BLtarget, Vector3 MLtarget, Vector3 FLtarget)
{
	BRleg.setLegTarget(body2legCoord(BRtarget, BRleg.ctr2root, BRleg.ctr2rootTheta));
	MRleg.setLegTarget(body2legCoord(MRtarget, MRleg.ctr2root, MRleg.ctr2rootTheta));
	FRleg.setLegTarget(body2legCoord(FRtarget, FRleg.ctr2root, FRleg.ctr2rootTheta));
	BLleg.setLegTarget(body2legCoord(BLtarget, BLleg.ctr2root, BLleg.ctr2rootTheta));
	MLleg.setLegTarget(body2legCoord(MLtarget, MLleg.ctr2root, MLleg.ctr2rootTheta));
	FLleg.setLegTarget(body2legCoord(FLtarget, FLleg.ctr2root, FLleg.ctr2rootTheta));
}

void Hexapod::reInit()
{
	BRleg.reInit();
	MRleg.reInit();
	FRleg.reInit();
	BLleg.reInit();
	MLleg.reInit();
	FLleg.reInit();
}

void Hexapod::setHeight(float height)
{
	currentHeight = height;

	this->BRleg.setHeight(height);
	this->MRleg.setHeight(height);
	this->FRleg.setHeight(height);
	this->BLleg.setHeight(height);
	this->MLleg.setHeight(height);
	this->FLleg.setHeight(height);
	
}

void Hexapod::setYaw(float yaw)
{
	BRleg.setYaw(yaw);
	MRleg.setYaw(yaw);
	FRleg.setYaw(yaw);
	BLleg.setYaw(yaw);
	MLleg.setYaw(yaw);
	FLleg.setYaw(yaw);

}

Vector3 Hexapod::yawBias(Vector3 bias,float theta)
{
	auto v = Vector3(
		cos(theta) * (bias.x) - sin(theta) * (bias.y),
		sin(theta) * (bias.x) + cos(theta) * (bias.y),
		bias.z
	);
	return v;
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
