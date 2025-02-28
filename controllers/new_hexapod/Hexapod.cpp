#include "Hexapod.h"


Hexapod::Hexapod() : Robot()
{
	BackRightLeg = Leg(getMotor("M_BR_COXA"),getMotor("M_BR_FEMUR"),getMotor("M_BR_TIBIA"));
	MiddleRightLeg = Leg(getMotor("M_MR_COXA"), getMotor("M_MR_FEMUR"), getMotor("M_MR_TIBIA"));
	FrontRightLeg = Leg(getMotor("M_FR_COXA"), getMotor("M_FR_FEMUR"), getMotor("M_FR_TIBIA"));
	BackLeftLeg = Leg(getMotor("M_BL_COXA"), getMotor("M_BL_FEMUR"), getMotor("M_BL_TIBIA"));
	MiddleLeftLeg = Leg(getMotor("M_ML_COXA"), getMotor("M_ML_FEMUR"), getMotor("M_ML_TIBIA"));
	FrontLeftLeg = Leg(getMotor("M_FL_COXA"), getMotor("M_FL_FEMUR"), getMotor("M_FL_TIBIA"));
}

Hexapod::~Hexapod()
{
}

void Hexapod::setPose(float* backRightAngles, float* middleRightAngles, float* frontRightAngles,
    float* backLeftAngles, float* middleLeftAngles, float* frontLeftAngles) {
    if (backRightAngles) {
        //backRightAngles[2] += M_PI / 3.0f;
        BackRightLeg.setRadAngles(backRightAngles);
    }
    if (middleRightAngles) {
        //middleRightAngles[2] += M_PI / 3.0f;
        MiddleRightLeg.setRadAngles(middleRightAngles);
    }
    if (frontRightAngles) {
        //frontRightAngles[2] += M_PI / 3.0f;
        FrontRightLeg.setRadAngles(frontRightAngles);
    }
    if (backLeftAngles) {
        //backLeftAngles[2] -= M_PI / 3.0f;
        BackLeftLeg.setRadAngles(backLeftAngles);
    }
    if (middleLeftAngles) {
        //middleLeftAngles[2] -= M_PI / 3.0f;
        MiddleLeftLeg.setRadAngles(middleLeftAngles);
    }
    if (frontLeftAngles) {
        //frontLeftAngles[2] -= M_PI / 3.0f;
        FrontLeftLeg.setRadAngles(frontLeftAngles);
    }
}

//只是反向运动学，输入参数需要进行预处理，vector3应为该条腿根部到目标点的向量
float* Hexapod::ik(Vector3 vector3)
{
    float theta1 = atan2(vector3.y, vector3.x);
    float R = sqrt(vector3.x * vector3.x + vector3.y * vector3.y);
    float ar = atan2(-vector3.z, (R - COXA_LEN));
    float Lr = sqrt(vector3.z * vector3.z + (R - COXA_LEN) * (R - COXA_LEN));
    float a1 = acos(CLIP((FEMUR_LEN * FEMUR_LEN + Lr * Lr - TIBIA_LEN * TIBIA_LEN) / (2 * Lr * FEMUR_LEN), -1, 1));
    float theta2 = a1 - ar;
    float a2 = acos(CLIP((Lr * Lr + TIBIA_LEN * TIBIA_LEN - FEMUR_LEN * FEMUR_LEN) / (2 * Lr * TIBIA_LEN), -1, 1));
    float theta3 = -(a1 + a2);
    float angles[3] = { theta1,theta2,theta3 };
    return angles;
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
	float* angles1 = ik(v1);
	float* angles2 = ik(v2);
	float* angles3 = ik(v3);
	float* angles4 = ik(v4);
	float* angles5 = ik(v5);
	float* angles6 = ik(v6);
	setPose(angles1, angles2, angles3, angles4, angles5, angles6);
}

void Hexapod::setTargets(Vector3 BRtarget, Vector3 MRtarget, Vector3 FRtarget, Vector3 BLtarget, Vector3 MLtarget, Vector3 FLtarget)
{
    Vector3 tmp = BRtarget - center2BackRightRoot;
    Vector3 BRtargetBiased = Vector3(
        cos(center2BackRightRootTheta) * tmp.x + sin(center2BackRightRootTheta) * tmp.y,
		-sin(center2BackRightRootTheta) * tmp.x + cos(center2BackRightRootTheta) * tmp.y,
		tmp.z
        );
	tmp = MRtarget - center2MiddleRightRoot;
	Vector3 MRtargetBiased = Vector3(
		cos(center2MiddleRightRootTheta) * tmp.x + sin(center2MiddleRightRootTheta) * tmp.y,
		-sin(center2MiddleRightRootTheta) * tmp.x + cos(center2MiddleRightRootTheta) * tmp.y,
		tmp.z
	);
	tmp = FRtarget - center2FrontRightRoot;
	Vector3 FRtargetBiased = Vector3(
		cos(center2FrontRightRootTheta) * tmp.x + sin(center2FrontRightRootTheta) * tmp.y,
		-sin(center2FrontRightRootTheta) * tmp.x + cos(center2FrontRightRootTheta) * tmp.y,
		tmp.z
	);
	tmp = BLtarget - center2BackLeftRoot;
	Vector3 BLtargetBiased = Vector3(
		cos(center2BackLeftRootTheta) * tmp.x + sin(center2BackLeftRootTheta) * tmp.y,
		-sin(center2BackLeftRootTheta) * tmp.x + cos(center2BackLeftRootTheta) * tmp.y,
		tmp.z
	);
	tmp = MLtarget - center2MiddleLeftRoot;
	Vector3 MLtargetBiased = Vector3(
		cos(center2MiddleLeftRootTheta) * tmp.x + sin(center2MiddleLeftRootTheta) * tmp.y,
		-sin(center2MiddleLeftRootTheta) * tmp.x + cos(center2MiddleLeftRootTheta) * tmp.y,
		tmp.z
	);
	tmp = FLtarget - center2FrontLeftRoot;
	Vector3 FLtargetBiased = Vector3(
		cos(center2FrontLeftRootTheta) * tmp.x + sin(center2FrontLeftRootTheta) * tmp.y,
		-sin(center2FrontLeftRootTheta) * tmp.x + cos(center2FrontLeftRootTheta) * tmp.y,
		tmp.z
	);
	float* angles1 = ik(BRtargetBiased);
	float* angles2 = ik(MRtargetBiased);
	float* angles3 = ik(FRtargetBiased);
	float* angles4 = ik(BLtargetBiased);
	float* angles5 = ik(MLtargetBiased);
	float* angles6 = ik(FLtargetBiased);
	setPose(angles1, angles2, angles3, angles4, angles5, angles6);
}
