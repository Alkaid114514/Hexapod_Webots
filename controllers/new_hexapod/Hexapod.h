

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
	Leg BRleg;
	Leg MRleg;
	Leg FRleg;
	Leg BLleg;
	Leg MLleg;
	Leg FLleg;

	
public:
	
	Hexapod();
	~Hexapod();

	enum GaitType
	{
		Tripod,
		Ripple,
		Wave
	};

	/// <summary>
	/// 以机器人身体中心为原点的坐标系为原点，六条腿根部的向量(机器人坐标系)
	/// </summary>
	const Vector3 ctr2BRroot = Vector3(0.059f, -0.083f, 0.0f);
	const Vector3 ctr2MRroot = Vector3(0.08f, 0.0f, 0.0f);
	const Vector3 ctr2FRroot = Vector3(0.064f, 0.082f, 0.0f);
	const Vector3 ctr2BLroot = Vector3(-0.059f, -0.083f, 0.0f);
	const Vector3 ctr2MLroot = Vector3(-0.08f, 0.0f, 0.0f);
	const Vector3 ctr2FLroot = Vector3(-0.064f, 0.082f, 0.0f);

	/// <summary>
	/// 以机器人身体中心为原点的坐标系为原点，六条腿根部的坐标系与机器人身体中心坐标系的夹角
	/// </summary>
	const float ctr2BRrootTheta = -(float)M_PI_4;			//-0.785398f
	const float ctr2MRrootTheta = 0.0f;
	const float ctr2FRrootTheta = (float)M_PI_4;			//0.785398f
	const float ctr2BLrootTheta = -3.0f * (float)M_PI_4;	//-2.3562f
	const float ctr2MLrootTheta = (float)M_PI;		//3.14159f
	const float ctr2FLrootTheta = 3.0f * (float)M_PI_4;	//2.3562f

	/// <summary>
	/// 机器人正常站立时，自身根部到六条腿末端的向量(腿坐标系)
	/// </summary>
	Vector3 initStandBR;
	Vector3 initStandMR;
	Vector3 initStandFR;
	Vector3 initStandBL;
	Vector3 initStandML;
	Vector3 initStandFL;
	float initHeight;

	Vector3 currentStandBR;
	Vector3 currentStandMR;
	Vector3 currentStandFR;
	Vector3 currentStandBL;
	Vector3 currentStandML;
	Vector3 currentStandFL;
	float currentHeight;

	/// <summary>
	/// 同时设置六条腿所有关节的角度，设置完后不会立即运动，需要调用startMove()函数开始运动
	/// </summary>
	/// <param name="BRangles">后右腿的三个关节角度</param>
	/// <param name="MRangles">中右腿的三个关节角度</param>
	/// <param name="FRangles">前右腿的三个关节角度</param>
	/// <param name="BLangles">后左腿的三个关节角度</param>
	/// <param name="MLangles">中左腿的三个关节角度</param>
	/// <param name="FLangles">前左腿的三个关节角度</param>
	void setPose(Vector3 BRangles, Vector3 MRangles, Vector3 FRangles,
		Vector3 BLangles, Vector3 MLangles, Vector3 FLangles);
	void setBRpose(Vector3 angles);
	void setMRpose(Vector3 angles);
	void setFRpose(Vector3 angles);
	void setBLpose(Vector3 angles);
	void setMLpose(Vector3 angles);
	void setFLpose(Vector3 angles);

	/// <summary>
	/// 仅反向运动学，输入参数需要进行预处理，参数vector3应为该条腿根部到目标点的向量(以腿根部为原点的坐标系)
	/// </summary>
	/// <param name="vector3">该条腿根部到目标点的向量(腿坐标系)</param>
	/// <returns>三个关节旋转角</returns>
	Vector3 ik(Vector3 vector3);

	/// <summary>
	/// 仅正向运动学，输入参数为三个关节的角度，返回腿根部到末端的向量(腿坐标系)
	/// </summary>
	/// <param name="angles">三个关节的角度,x为coxa,y为femur,z为tibia</param>
	/// <returns>腿根部到末端的向量(腿坐标系)</returns>
	Vector3 fk(Vector3 angles);

	/// <summary>
	/// 将以腿根部为原点的相对向量(腿坐标系)转换为以机器人身体中心为原点指向目标点的向量(机器人坐标系)
	/// </summary>
	/// <param name="relevant">以腿根部为原点的相对向量(腿坐标系)</param>
	/// <param name="bias">以机器人身体中心为原点指向腿根部的向量(机器人坐标系)</param>
	/// <param name="theta">以机器人身体中心为原点坐标系下，腿根部为原点的坐标系的旋转角</param>
	/// <returns>以机器人身体中心为原点指向目标点的向量(机器人坐标系)</returns>
	Vector3 leg2bodyCoord(Vector3 relevant,Vector3 bias,float theta);

	/// <summary>
	/// 将以机器人身体中心为原点指向目标点的向量(机器人坐标系)转换为以腿根部为原点的相对向量(腿坐标系)
	/// </summary>
	/// <param name="absolute">以机器人身体中心为原点指向目标点的向量(机器人坐标系)</param>
	/// <param name="bias">以机器人身体中心为原点指向腿根部的向量(机器人坐标系)</param>
	/// <param name="theta">以机器人身体中心为原点坐标系下，腿根部为原点的坐标系的旋转角</param>
	/// <returns>以腿根部为原点的相对向量(腿坐标系)</returns>
	Vector3 body2legCoord(Vector3 absolute, Vector3 bias, float theta);

	void move(Vector3 velocity,float omega, float timeStep);

	Vector3 getNextBodyTarget(Vector3 velocity, float omega, Vector3 r0, Vector3 initial,Vector3 legBias,float timeStep);

	//Leg* getLegGroup(GaitType gaitType);

	/// <summary>
	/// 所有的设置目标点的函数并不会使机器人直接运动，需要设置完目标点后调用startMove()函数开始运动
	/// </summary>
	void setTargets(Vector3 BRtarget,Vector3 MRtarget,Vector3 FRtarget,Vector3 BLtarget,Vector3 MLtarget,Vector3 FLtarget);
	void setBRbodyTarget(Vector3 target);
	void setMRbodyTarget(Vector3 target);
	void setFRbodyTarget(Vector3 target);
	void setBLbodyTarget(Vector3 target);
	void setMLbodyTarget(Vector3 target);
	void setFLbodyTarget(Vector3 target);

	void setBRlegTarget(Vector3 target);
	void setMRlegTarget(Vector3 target);
	void setFRlegTarget(Vector3 target);
	void setBLlegTarget(Vector3 target);
	void setMLlegTarget(Vector3 target);
	void setFLlegTarget(Vector3 target);

	void setHeight(float height);

	/// <summary>
	/// 开始按照设置的角度和目标点运动
	/// </summary>
	void startMove();
};

