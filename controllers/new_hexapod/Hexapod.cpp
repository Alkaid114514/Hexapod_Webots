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
