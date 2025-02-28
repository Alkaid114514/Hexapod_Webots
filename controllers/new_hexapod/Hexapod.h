

#include <webots/Robot.hpp>
#include "Leg.h"
#include "Vector3.h"

#define COXA_LEN (38.0f/1000.0f)
#define FEMUR_LEN (79.2f/1000.0f)
#define TIBIA_LEN (116.0f/1000.0f)

#define CLIP(value, lower, upper) (((value) < (lower)) ? (lower) : ((value) > (upper) ? (upper) : (value)))


class Hexapod : public webots::Robot
{
private:
	Leg BackRightLeg;
	Leg MiddleRightLeg;
	Leg FrontRightLeg;
	Leg BackLeftLeg;
	Leg MiddleLeftLeg;
	Leg FrontLeftLeg;

	const Vector3 center2BackRightRoot = Vector3(0.059, -0.083, 0);
	const Vector3 center2MiddleRightRoot = Vector3(0.08, 0, 0);
	const Vector3 center2FrontRightRoot = Vector3(0.064, 0.082, 0);
	const Vector3 center2BackLeftRoot = Vector3(-0.059, -0.083, 0);
	const Vector3 center2MiddleLeftRoot = Vector3(-0.08, 0, 0);
	const Vector3 center2FrontLeftRoot = Vector3(-0.064, 0.082, 0);

	const float center2BackRightRootTheta = -0.785395;
	const float center2MiddleRightRootTheta = 0;
	const float center2FrontRightRootTheta = 0.785398;
	const float center2BackLeftRootTheta = -2.3562;
	const float center2MiddleLeftRootTheta = 0;
	const float center2FrontLeftRootTheta = 2.3562;
public:
	Hexapod();
	~Hexapod();

	void setPose(float* backRightAngles, float* middleRightAngles, float* frontRightAngles,
		float* backLeftAngles, float* middleLeftAngles, float* frontLeftAngles);
	float* ik(Vector3 vector3);
	void move(Vector3 velocity,float omega);
	void setTargets(Vector3 BRtarget,Vector3 MRtarget,Vector3 FRtarget,Vector3 BLtarget,Vector3 MLtarget,Vector3 FLtarget);
};

