

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
	

	const Vector3 center2BackRightRoot = Vector3(0.059f, -0.083f, 0.0f);
	const Vector3 center2MiddleRightRoot = Vector3(0.08f, 0.0f, 0.0f);
	const Vector3 center2FrontRightRoot = Vector3(0.064f, 0.082f, 0.0f);
	const Vector3 center2BackLeftRoot = Vector3(-0.059f, -0.083f, 0.0f);
	const Vector3 center2MiddleLeftRoot = Vector3(-0.08f, 0.0f, 0.0f);
	const Vector3 center2FrontLeftRoot = Vector3(-0.064f, 0.082f, 0.0f);

	const float center2BackRightRootTheta = -0.785395f;
	const float center2MiddleRightRootTheta = 0.0f;
	const float center2FrontRightRootTheta = 0.785398f;
	const float center2BackLeftRootTheta = -2.3562f;
	const float center2MiddleLeftRootTheta = M_PI;
	const float center2FrontLeftRootTheta = 2.3562f;
public:
	
	Hexapod();
	~Hexapod();

	Leg BackRightLeg;
	Leg MiddleRightLeg;
	Leg FrontRightLeg;
	Leg BackLeftLeg;
	Leg MiddleLeftLeg;
	Leg FrontLeftLeg;

	void setPose(Vector3 backRightAngles, Vector3 middleRightAngles, Vector3 frontRightAngles,
		Vector3 backLeftAngles, Vector3 middleLeftAngles, Vector3 frontLeftAngles);

	Vector3 ik(Vector3 vector3);
	void move(Vector3 velocity,float omega);
	void setTargets(Vector3 BRtarget,Vector3 MRtarget,Vector3 FRtarget,Vector3 BLtarget,Vector3 MLtarget,Vector3 FLtarget);
	void setBRtarget(Vector3 target);
	void setMRtarget(Vector3 target);
	void setFRtarget(Vector3 target);
	void setBLtarget(Vector3 target);
	void setMLtarget(Vector3 target);
	void setFLtarget(Vector3 target);
	void startMove();
};

