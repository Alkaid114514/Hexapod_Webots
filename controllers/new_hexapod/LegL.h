#pragma once
#include <webots/Motor.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include <webots/TouchSensor.hpp>

#include "Hexapod.h"
#include "Leg.h"
#include "Vector3.h"


class LegL : public Leg
{
public:
    LegL() : Leg()
    {
    }

    LegL(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia, Vector3 ctr2root, float ctr2rootTheta) : Leg(
        coxa, femur, tibia, ctr2root, ctr2rootTheta)
    {
    }

    ~LegL() override = default;


    /// <summary>
    /// 仅正向运动学，输入参数为三个关节的角度，返回腿根部到末端的向量(腿坐标系)
    /// </summary>
    /// <param name="angles">三个关节的角度,x为coxa,y为femur,z为tibia</param>
    /// <returns>腿根部到末端的向量(腿坐标系)</returns>
    Vector3 fk(Vector3 angles) override;
    /// <summary>
    /// 仅反向运动学，输入参数需要进行预处理，参数vector3应为该条腿根部到目标点的向量(以腿根部为原点的坐标系)
    /// </summary>
    /// <param name="vector3">该条腿根部到目标点的向量(腿坐标系)</param>
    /// <returns>三个关节旋转角</returns>
    Vector3 ik(Vector3 vector3) override;

    void setPose(Vector3 angles) override;
    void setTibiaPose(float angle) override;

    // void balance();
};
