#pragma once
#include <webots/Motor.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include "Vector3.h"

class LegL
{
private:
	webots::Motor* coxaMotor;
	webots::Motor* femurMotor;
	webots::Motor* tibiaMotor;

	Vector3 angles = Vector3();

	
	

public:
	LegL();
	LegL(webots::Motor* coxa,webots::Motor* femur,webots::Motor* tibia, Vector3 ctr2root, float ctr2rootTheta);
	~LegL();

	bool isOnGround = true;
	Vector3 initAngles = Vector3(0.0f, 0.0f, -(float)M_PI / 3.0f);
	Vector3 currentStandAngles = initAngles;
	Vector3 initStandTarget = fk(initAngles);
	Vector3 ctr2root;
	float ctr2rootTheta;

	/// <summary>
	/// �������˶�ѧ���������Ϊ�����ؽڵĽǶȣ������ȸ�����ĩ�˵�����(������ϵ)
	/// </summary>
	/// <param name="angles">�����ؽڵĽǶ�,xΪcoxa,yΪfemur,zΪtibia</param>
	/// <returns>�ȸ�����ĩ�˵�����(������ϵ)</returns>
	Vector3 fk(Vector3 angles);
	/// <summary>
	/// �������˶�ѧ�����������Ҫ����Ԥ��������vector3ӦΪ�����ȸ�����Ŀ��������(���ȸ���Ϊԭ�������ϵ)
	/// </summary>
	/// <param name="vector3">�����ȸ�����Ŀ��������(������ϵ)</param>
	/// <returns>�����ؽ���ת��</returns>
	Vector3 ik(Vector3 vector3);

	void setMotor(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia);
	void setOmega(float* omega);
	void setCoxaOmega(float omega);
	void setFemurOmega(float omega);
	void setTibiaOmega(float omega);
	void setLegTarget(Vector3 target);
	void setBodyTarget(Vector3 target);
	void setPose(Vector3 angles);
	void setCoxaPose(float angle);
	void setFemurPose(float angle);
	void setTibiaPose(float angle);

	void reInit();
	void setHeight(float height);
	void startMotor();

};

