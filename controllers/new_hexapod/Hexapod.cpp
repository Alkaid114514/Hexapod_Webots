#include "Hexapod.h"

Hexapod::Hexapod() : Robot()
{
	BRleg = LegR(getMotor("M_BR_COXA"), getMotor("M_BR_FEMUR"), getMotor("M_BR_TIBIA"), ctr2BRroot, ctr2BRrootTheta);
	MRleg = LegR(getMotor("M_MR_COXA"), getMotor("M_MR_FEMUR"), getMotor("M_MR_TIBIA"), ctr2MRroot, ctr2MRrootTheta);
	FRleg = LegR(getMotor("M_FR_COXA"), getMotor("M_FR_FEMUR"), getMotor("M_FR_TIBIA"), ctr2FRroot, ctr2FRrootTheta);
	BLleg = LegL(getMotor("M_BL_COXA"), getMotor("M_BL_FEMUR"), getMotor("M_BL_TIBIA"), ctr2BLroot, ctr2BLrootTheta);
	MLleg = LegL(getMotor("M_ML_COXA"), getMotor("M_ML_FEMUR"), getMotor("M_ML_TIBIA"), ctr2MLroot, ctr2MLrootTheta);
	FLleg = LegL(getMotor("M_FL_COXA"), getMotor("M_FL_FEMUR"), getMotor("M_FL_TIBIA"), ctr2FLroot, ctr2FLrootTheta);

	/*currentStandBR = initStandBR = rfk(Vector3(0.0f, 0.0f, (float)M_PI / 3.0f));
	currentStandMR = initStandMR = rfk(Vector3(0.0f, 0.0f, (float)M_PI / 3.0f));
	currentStandFR = initStandFR = rfk(Vector3(0.0f, -0.0f, (float)M_PI / 3.0f));
	currentStandBL = initStandBL = lfk(Vector3(0.0f, 0.0f, -(float)M_PI / 3.0f));
	currentStandML = initStandML = lfk(Vector3(0.0f, 0.0f, -(float)M_PI / 3.0f));
	currentStandFL = initStandFL = lfk(Vector3(-0.0f, 0.0f, -(float)M_PI / 3.0f));*/
	
	/*currentStandBR = initStandBR = rfk(Vector3(0.0f, 0.0f, 0.0f));
	currentStandMR = initStandMR = rfk(Vector3(0.0f, 0.0f, 0.0f));
	currentStandFR = initStandFR = rfk(Vector3(0.0f, -0.0f, 0.0f));
	currentStandBL = initStandBL = lfk(Vector3(0.0f, 0.0f, 0.0f));
	currentStandML = initStandML = lfk(Vector3(0.0f, 0.0f, 0.0f));
	currentStandFL = initStandFL = lfk(Vector3(-0.3f, 0.3f, 0.0f));*/
	/*std::cout << "initStandFL " << initStandFL.x << " " << initStandFL.y << " " << initStandFL.z << std::endl;
	auto i = lik(initStandFL);
	auto r = this->body2legCoord(this->leg2bodyCoord(initStandFL, ctr2FLroot, ctr2FLrootTheta), ctr2FLroot, ctr2FLrootTheta);
	std::cout << "i " << i.x << " " << i.y << " " << i.z << std::endl;
	std::cout << "r " << r.x << " " << r.y << " " << r.z << std::endl;*/
	currentHeight = initHeight = FLleg.initStandTarget.z;
	//std::cout << "initHeight " << initHeight << std::endl;
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

//void Hexapod::setBRpose(Vector3 angles)
//{
//	angles.z -= (float)M_PI / 3.0f;
//	//angles.x = -angles.x;
//	BRleg.setRadAngles(angles);
//}
//
//void Hexapod::setMRpose(Vector3 angles)
//{
//	angles.z -= (float)M_PI / 3.0f;
//	//angles.x = -angles.x;
//	MRleg.setRadAngles(angles);
//}
//
//void Hexapod::setFRpose(Vector3 angles)
//{
//	angles.z -= (float)M_PI / 3.0f;
//	//angles.x = -angles.x;
//	FRleg.setRadAngles(angles);
//}
//
//void Hexapod::setBLpose(Vector3 angles)
//{
//	angles.z += (float)M_PI / 3.0f;
//	//angles.x = -angles.x;
//	BLleg.setRadAngles(angles);
//}
//
//void Hexapod::setMLpose(Vector3 angles)
//{
//	angles.z += (float)M_PI / 3.0f;
//	//angles.x = -angles.x;
//	MLleg.setRadAngles(angles);
//}
//
//void Hexapod::setFLpose(Vector3 angles)
//{
//	angles.z += (float)M_PI / 3.0f;
//	//angles.x = -angles.x;
//	FLleg.setRadAngles(angles);
//}


//Vector3 Hexapod::lik(Vector3 vector3)
//{
//	vector3.z = -vector3.z;
//    float theta1 = atan2(vector3.y, vector3.x);
//    float R = sqrt(vector3.x * vector3.x + vector3.y * vector3.y);
//    float ar = atan2(-vector3.z, (R - COXA_LEN));
//    float Lr = sqrt(vector3.z * vector3.z + (R - COXA_LEN) * (R - COXA_LEN));
//    float a1 = acos(CLIP((FEMUR_LEN * FEMUR_LEN + Lr * Lr - TIBIA_LEN * TIBIA_LEN) / (2 * Lr * FEMUR_LEN), -1, 1));
//    float theta2 = a1 - ar;
//    float a2 = acos(CLIP((Lr * Lr + TIBIA_LEN * TIBIA_LEN - FEMUR_LEN * FEMUR_LEN) / (2 * Lr * TIBIA_LEN), -1, 1));
//	float theta3 = -(a1 + a2);
//    return Vector3(theta1,-theta2,theta3);
//}
//
//
//Vector3 Hexapod::rik(Vector3 vector3)
//{
//	vector3.z = -vector3.z;
//	float theta1 = atan2(vector3.y, vector3.x);
//	float R = sqrt(vector3.x * vector3.x + vector3.y * vector3.y);
//	float ar = atan2(-vector3.z, (R - COXA_LEN));
//	float Lr = sqrt(vector3.z * vector3.z + (R - COXA_LEN) * (R - COXA_LEN));
//	float a1 = acos(CLIP((FEMUR_LEN * FEMUR_LEN + Lr * Lr - TIBIA_LEN * TIBIA_LEN) / (2 * Lr * FEMUR_LEN), -1, 1));
//	float theta2 = a1 - ar;
//	float a2 = acos(CLIP((Lr * Lr + TIBIA_LEN * TIBIA_LEN - FEMUR_LEN * FEMUR_LEN) / (2 * Lr * TIBIA_LEN), -1, 1));
//	float theta3 = -(a1 + a2);
//	return Vector3(theta1, theta2, -theta3);
//}
//
//Vector3 Hexapod::lfk(Vector3 angles)
//{
//	//angles.z -= (float)M_PI / 3.0f;
//	float tmp = (COXA_LEN + FEMUR_LEN * cos(angles.y) + TIBIA_LEN * cos(-angles.z - angles.y));
//	auto v = Vector3(
//		tmp * cos(angles.x), 
//		tmp * sin(angles.x), 
//		-FEMUR_LEN * sin(angles.y) + TIBIA_LEN * sin(-angles.z - angles.y)
//	);
//	//v.y = -v.y;
//	//v.z = -v.z;
//	return v;
//}
//
//Vector3 Hexapod::rfk(Vector3 angles)
//{
//	//angles.z -= (float)M_PI / 3.0f;
//	float tmp = (COXA_LEN + FEMUR_LEN * cos(angles.y) + TIBIA_LEN * cos(angles.z - angles.y));
//	auto v = Vector3(
//		tmp * cos(angles.x),
//		tmp * sin(angles.x),
//		-FEMUR_LEN * sin(angles.y) + TIBIA_LEN * sin(angles.z - angles.y)
//	);
//	//v.y = -v.y;
//	//v.z = -v.z;
//	return v;
//}

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

//Vector3 Hexapod::calculateFLtarget(Vector3 velocity, float omega, float timeStep, float omega1, float omega2, float omega3){
//	float T = 0.5;//机器人抬腿到放下的时间（这个数是瞎填的，还没调试
//	float A = FEMUR_LEN * sin(omega2) * T;-(omega3 * TIBIA_LEN * sin(omega3 - omega2) * T) / 2 * (omega3 - omega2);
//	float lambda = 2 * omega1 * COXA_LEN * sin(T * omega / 2) * T / (T * omega);//步长
//	float Phase;//相位（用于给腿分组的？）
//	int resolution = 20;//插值插20个点
//	if (velocity == Vector3() && omega == 0.0f)
//	{
//		return;
//	};
//	Vector3 w = Vector3(0.0f, 0.0f, omega);
//	Vector3 r0 = velocity.cross(w) / (omega * omega);
//	//算机器人坐标系下圆心到脚尖的向量
//	Vector3 initTmpFL = r0 + leg2bodyCoord(initStandFL, ctr2FLroot, ctr2FLrootTheta);
//	float alpha = omega * 2 * T;
//	for (int n = 0; n < resolution;++n) {
//		float tmp_alpha = omega * n / resolution;//alpha细分成20份
//		//z轴抬腿的插值如下
//		float  tmpz = initTmpFL.z + A * sin(tmp_alpha);
//		Vector3 tmpFL = Vector3(initTmpFL.x * cos(tmp_alpha) - initTmpFL.y * sin(tmp_alpha),
//			initTmpFL.x * sin(tmp_alpha) + initTmpFL.y * cos(tmp_alpha),
//			tmpz);
//		Vector3 targetFL = body2legCoord(tmpFL - initTmpFL, ctr2FLroot, ctr2FLrootTheta) + initStandFL;
//}

void Hexapod::move(Vector3 velocity, float omega,float timeStep, float omega1, float omega2, float omega3)//omega123指的是舵机的角速度
{	
	float T = 0.5;//机器人抬腿到放下的时间（这个数是瞎填的，还没调试
	float A = FEMUR_LEN * sin(omega2) * T;-(omega3 * TIBIA_LEN * sin(omega3 - omega2) * T) / 2 * (omega3 - omega2);
	float lambda = 2 * omega1 * COXA_LEN * sin(T * omega / 2) * T / (T * omega);//步长
	float Phase;//相位（用于给腿分组的？）
	int resolution = 20;//插值插20个点
	if (velocity == Vector3() && omega == 0.0f)
	{
		return;
	};
	Vector3 w = Vector3(0.0f, 0.0f, omega);
	Vector3 r0 = velocity.cross(w) / (omega * omega);
	//算机器人坐标系下圆心到脚尖的向量
	Vector3 initTmpFL = r0 + leg2bodyCoord(FLleg.initStandTarget, FLleg.ctr2root, FLleg.ctr2rootTheta);
	//Vector3 initTmpML = r0 + leg2bodyCoord(initStandML, ctr2MLroot, ctr2MLrootTheta);
	//Vector3 initTmpBL = r0 + leg2bodyCoord(initStandBL, ctr2BLroot, ctr2BLrootTheta);
	//Vector3 initTmpFR = r0 + leg2bodyCoord(initStandFR, ctr2FRroot, ctr2FRrootTheta);
	//Vector3 initTmpMR = r0 + leg2bodyCoord(initStandMR, ctr2MRroot, ctr2MRrootTheta);
	//Vector3 initTmpBR = r0 + leg2bodyCoord(initStandBR, ctr2BRroot, ctr2BRrootTheta);
	float alpha = omega * 2 * T;
	for (int n = 0; n < resolution;++n) {
		float tmp_alpha = alpha*  n/resolution ;//alpha细分成20份
	
		//z轴抬腿的插值如下
		float  tmpz = initTmpFL.z + A * sin(tmp_alpha);
		//再算内转tmp_alpha角度后的向量
		Vector3 tmpFL = Vector3(initTmpFL.x * cos(tmp_alpha) - initTmpFL.y * sin(tmp_alpha),
			initTmpFL.x * sin(tmp_alpha) + initTmpFL.y * cos(tmp_alpha),
			tmpz);
		/*Vector3 tmpML = Vector3(initTmpML.x * cos(tmp_alpha) - initTmpML.y * sin(tmp_alpha),
			initTmpML.x * sin(tmp_alpha) + initTmpML.y * cos(tmp_alpha),
			tmpz);
		Vector3 tmpBL = Vector3(initTmpBL.x * cos(tmp_alpha) - initTmpBL.y * sin(tmp_alpha),
			initTmpBL.x * sin(tmp_alpha) + initTmpBL.y * cos(tmp_alpha),
			tmpz);
		Vector3 tmpFR = Vector3(initTmpFR.x * cos(tmp_alpha) - initTmpFR.y * sin(tmp_alpha),
			initTmpFR.x * sin(tmp_alpha) + initTmpFR.y * cos(tmp_alpha),
			tmpz);
		Vector3 tmpMR = Vector3(initTmpMR.x * cos(tmp_alpha) - initTmpMR.y * sin(tmp_alpha),
			initTmpMR.x * sin(tmp_alpha) + initTmpMR.y * cos(tmp_alpha),
			tmpz);
		Vector3 tmpBR = Vector3(initTmpBR.x * cos(tmp_alpha) - initTmpBR.y * sin(tmp_alpha),
			initTmpBR.x * sin(tmp_alpha) + initTmpBR.y * cos(tmp_alpha),
			tmpz);*/
		//则需要带入ik的向量如下（已经转换成腿坐标系
	/*	Vector3 targetFL = body2legCoord(tmpFL - initTmpFL, ctr2FLroot, ctr2FLrootTheta) + initStandFL;
		Vector3 targetML = body2legCoord(tmpML - initTmpML, ctr2MLroot, ctr2MLrootTheta) + initStandML;
		Vector3 targetBL = body2legCoord(tmpBL - initTmpBL, ctr2BLroot, ctr2BLrootTheta) + initStandBL;
		Vector3 targetFR = body2legCoord(tmpFR - initTmpFR, ctr2FRroot, ctr2FRrootTheta) + initStandFR;
		Vector3 targetMR = body2legCoord(tmpMR - initTmpMR, ctr2MRroot, ctr2MRrootTheta) + initStandMR;
		Vector3 targetBR = body2legCoord(tmpBR - initTmpBR, ctr2BRroot, ctr2BRrootTheta) + initStandBR;*/

		//移动过程中的脚尖目标点（连起来就是轨迹，在机器人身体坐标系下）
		Vector3 FLtarget = Vector3(tmpFL.x - r0.x, tmpFL.y - r0.y, tmpz);
	/*	Vector3 MLtarget = Vector3(tmpML.x - r0.x, tmpML.y - r0.y, tmpz);
		Vector3 BLtarget = Vector3(tmpBL.x - r0.x, tmpBL.y - r0.y, tmpz);
		Vector3 FRtarget = Vector3(tmpFR.x - r0.x, tmpFR.y - r0.y, tmpz);
		Vector3 MRtarget = Vector3(tmpMR.x - r0.x, tmpMR.y - r0.y, tmpz);
		Vector3 BRtarget = Vector3(tmpBR.x - r0.x, tmpBR.y - r0.y, tmpz);*/

		FLleg.setBodyTarget(FLtarget);
		
	}
		

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

//void Hexapod::setTargets(Vector3 BRtarget, Vector3 MRtarget, Vector3 FRtarget, Vector3 BLtarget, Vector3 MLtarget, Vector3 FLtarget)
//{
//	setBLbodyTarget(BLtarget);
//	setMLbodyTarget(MLtarget);
//	setFLbodyTarget(FLtarget);
//	setBRbodyTarget(BRtarget);
//	setMRbodyTarget(MRtarget);
//	setFRbodyTarget(FRtarget);
//}

void Hexapod::setBodyTargets(Vector3 BRtarget, Vector3 MRtarget, Vector3 FRtarget, Vector3 BLtarget, Vector3 MLtarget, Vector3 FLtarget)
{
	BRleg.setLegTarget(body2legCoord(BRtarget, BRleg.ctr2root, BRleg.ctr2rootTheta));
	MRleg.setLegTarget(body2legCoord(MRtarget, MRleg.ctr2root, MRleg.ctr2rootTheta));
	FRleg.setLegTarget(body2legCoord(FRtarget, FRleg.ctr2root, FRleg.ctr2rootTheta));
	BLleg.setLegTarget(body2legCoord(BLtarget, BLleg.ctr2root, BLleg.ctr2rootTheta));
	MLleg.setLegTarget(body2legCoord(MLtarget, MLleg.ctr2root, MLleg.ctr2rootTheta));
	FLleg.setLegTarget(body2legCoord(FLtarget, FLleg.ctr2root, FLleg.ctr2rootTheta));

}

//void Hexapod::setBRbodyTarget(Vector3 target)
//{
//	this->setBRpose(rik(body2legCoord(target, ctr2BRroot, ctr2BRrootTheta)));
//}
//
//void Hexapod::setMRbodyTarget(Vector3 target)
//{
//	this->setMRpose(rik(body2legCoord(target, ctr2MRroot, ctr2MRrootTheta)));
//}
//
//void Hexapod::setFRbodyTarget(Vector3 target)
//{
//	this->setFRpose(rik(body2legCoord(target, ctr2FRroot, ctr2FRrootTheta)));
//}
//
//void Hexapod::setBLbodyTarget(Vector3 target)
//{
//	this->setBLpose(lik(body2legCoord(target, ctr2BLroot, ctr2BLrootTheta)));
//}
//
//void Hexapod::setMLbodyTarget(Vector3 target)
//{
//	this->setMLpose(lik(body2legCoord(target, ctr2MLroot, ctr2MLrootTheta)));
//}
//
//void Hexapod::setFLbodyTarget(Vector3 target)
//{
//	this->setFLpose(lik(body2legCoord(target, ctr2FLroot, ctr2FLrootTheta)));
//}

void Hexapod::setBodyTarget(Vector3 target,LegL leg)
{
	leg.setLegTarget(body2legCoord(target, leg.ctr2root, leg.ctr2rootTheta));
}

void Hexapod::setBodyTarget(Vector3 target, LegR leg)
{
	leg.setLegTarget(body2legCoord(target, leg.ctr2root, leg.ctr2rootTheta));
}

void Hexapod::setHeight(float height)
{
	currentHeight = height;
	/*currentStandBR.z = height;
	currentStandMR.z = height;
	currentStandFR.z = height;
	currentStandBL.z = height;
	currentStandML.z = height;
	currentStandFL.z = height;*/

	this->BRleg.setHeight(height);
	this->MRleg.setHeight(height);
	this->FRleg.setHeight(height);
	this->BLleg.setHeight(height);
	this->MLleg.setHeight(height);
	this->FLleg.setHeight(height);
	/*std::cout << "currentStandFL.z " << currentStandFL.z << std::endl;
	auto i = lik(currentStandFL);
	std::cout << "ic " << i.x << " " << i.y << " " << i.z << std::endl;*/
	/*this->BRleg.reInit();
	this->MRleg.reInit();
	this->FRleg.reInit();
	this->BLleg.reInit();
	this->MLleg.reInit();
	this->FLleg.reInit();*/

	/*setBRlegTarget(currentStandBR);
	setMRlegTarget(currentStandMR);
	setFRlegTarget(currentStandFR);
	setBLlegTarget(currentStandBL);
	setMLlegTarget(currentStandML);
	setFLlegTarget(currentStandFL);*/
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
