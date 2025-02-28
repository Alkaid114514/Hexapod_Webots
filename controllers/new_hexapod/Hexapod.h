

#include <webots/Robot.hpp>
#include "Leg.h"

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
};

