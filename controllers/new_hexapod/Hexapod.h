

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
	

	
public:
	
	Hexapod();
	~Hexapod();

	Leg BRleg;
	Leg MRleg;
	Leg FRleg;
	Leg BLleg;
	Leg MLleg;
	Leg FLleg;

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
	Vector3 initialBR;
	Vector3 initialMR;
	Vector3 initialFR;
	Vector3 initialBL;
	Vector3 initialML;
	Vector3 initialFL;

	/// <summary>
	/// 同时设置六条腿所有关节的角度
	/// </summary>
	/// <param name="BRangles">后右腿的三个关节角度</param>
	/// <param name="MRangles">中右腿的三个关节角度</param>
	/// <param name="FRangles">前右腿的三个关节角度</param>
	/// <param name="BLangles">后左腿的三个关节角度</param>
	/// <param name="MLangles">中左腿的三个关节角度</param>
	/// <param name="FLangles">前左腿的三个关节角度</param>
	void setPose(Vector3 BRangles, Vector3 MRangles, Vector3 FRangles,
		Vector3 BLangles, Vector3 MLangles, Vector3 FLangles);

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
	Vector3 relevant2absolute(Vector3 relevant,Vector3 bias,float theta);

	/// <summary>
	/// 将以机器人身体中心为原点指向目标点的向量(机器人坐标系)转换为以腿根部为原点的相对向量(腿坐标系)
	/// </summary>
	/// <param name="absolute">以机器人身体中心为原点指向目标点的向量(机器人坐标系)</param>
	/// <param name="bias">以机器人身体中心为原点指向腿根部的向量(机器人坐标系)</param>
	/// <param name="theta">以机器人身体中心为原点坐标系下，腿根部为原点的坐标系的旋转角</param>
	/// <returns>以腿根部为原点的相对向量(腿坐标系)</returns>
	Vector3 absolute2relevant(Vector3 absolute, Vector3 bias, float theta);

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

