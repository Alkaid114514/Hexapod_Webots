#pragma once
#include <webots/Motor.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include "Vector3.h"


class LegL
{
    webots::Motor* coxaMotor;
    webots::Motor* femurMotor;
    webots::Motor* tibiaMotor;

    Vector3 motorAngles = Vector3();

public:
    LegL();
    LegL(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia, Vector3 ctr2root, float ctr2rootTheta);
    ~LegL();

    bool isOnGround = true;
    Vector3 initAngles = Vector3(0.0f, 0.0f, -static_cast<float>(M_PI) / 3.0f);
    Vector3 currentStandAngles = initAngles;
    Vector3 ctr2root;
    float ctr2rootTheta;
    float currentYaw = 0.0f;
    float currentRoll = 0.0f;
    float currentPitch = 0.0f;
    
    Vector3 initStandBodyTarget;
    Vector3 currentStandBodyTarget;

    Vector3 lastBodyTarget;

    /// <summary>
    /// 仅正向运动学，输入参数为三个关节的角度，返回腿根部到末端的向量(腿坐标系)
    /// </summary>
    /// <param name="angles">三个关节的角度,x为coxa,y为femur,z为tibia</param>
    /// <returns>腿根部到末端的向量(腿坐标系)</returns>
    Vector3 fk(Vector3 angles);
    /// <summary>
    /// 仅反向运动学，输入参数需要进行预处理，参数vector3应为该条腿根部到目标点的向量(以腿根部为原点的坐标系)
    /// </summary>
    /// <param name="vector3">该条腿根部到目标点的向量(腿坐标系)</param>
    /// <returns>三个关节旋转角</returns>
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
    void setYaw(float yaw);
    void setPitch(float pitch);
    void setRoll(float roll);
    void startMotor();
};
