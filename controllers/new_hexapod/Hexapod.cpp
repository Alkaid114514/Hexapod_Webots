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
