

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
public:
	Hexapod();
	~Hexapod();

	void setPose(float* backRightAngles, float* middleRightAngles, float* frontRightAngles,
		float* backLeftAngles, float* middleLeftAngles, float* frontLeftAngles);
	float* ik(Vector3 vector3);
};

