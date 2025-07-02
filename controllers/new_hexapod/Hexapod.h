#pragma once
#include <webots/Robot.hpp>
#include "IMU.h"
#include "LegR.h"
#include "LegL.h"
#include "Vector3.h"

#define COXA_LEN (38.0f/1000.0f)
#define FEMUR_LEN (79.2f/1000.0f)
#define TIBIA_LEN (116.0f/1000.0f)

#define WAVE_RATIO (1.0f/5.0f)
#define RIPPLE_RATIO (1.0f/2.0f)
#define FOURPLUSTWO_RATIO (1.0f/3.0f)
#define CLIP(value, lower, upper) (((value) < (lower)) ? (lower) : ((value) > (upper) ? (upper) : (value)))


class Hexapod : public webots::Robot
{
public:
    Hexapod();
    ~Hexapod() override;

    LegR* BRleg;
    LegR* MRleg;
    LegR* FRleg;
    LegL* BLleg;
    LegL* MLleg;
    LegL* FLleg;

    IMU imu;

    enum GaitStatus : std::int8_t
    {
        Tripod,
        Ripple,
        Wave,
        Ready,
        Stop,
        FourPlusTwo
    };

    /// <summary>
    /// 以机器人身体中心为原点的坐标系为原点，六条腿根部的向量(机器人坐标系)
    /// </summary>
    const Vector3 ctr2BRroot = Vector3(0.059f, -0.083f, 0.0f);
    const Vector3 ctr2MRroot = Vector3(0.08f, 0.0, 0.0f);
    const Vector3 ctr2FRroot = Vector3(0.064f, 0.082f, 0.0f);
    const Vector3 ctr2BLroot = Vector3(-0.059f, -0.083f, 0.0f);
    const Vector3 ctr2MLroot = Vector3(-0.08f, 0.0f, 0.0f);
    const Vector3 ctr2FLroot = Vector3(-0.064f, 0.082f, 0.0f);

    /// <summary>
    /// 以机器人身体中心为原点的坐标系为原点，六条腿根部的坐标系与机器人身体中心坐标系的夹角
    /// </summary>
    const float ctr2BRrootTheta = -static_cast<float>(M_PI_4); //-0.785398f
    const float ctr2MRrootTheta = 0.0f;
    const float ctr2FRrootTheta = (float)M_PI_4; //0.785398f
    const float ctr2BLrootTheta = -3.0f * static_cast<float>(M_PI_4); //-2.3562f
    const float ctr2MLrootTheta = (float)M_PI; //3.14159f
    const float ctr2FLrootTheta = 3.0f * static_cast<float>(M_PI_4); //2.3562f

    float initHeight;
    float currentHeight;
    float coxaOmega = 0.1f;
    float femurOmega = 0.1f;
    float tibiaOmega = 0.1f;
    int totalFrame = 32;
    float stepTheta;
    float stepLen = 0.05f;

    int gaitGroupIndex = 0;
    int previousGroupIndex = -1;
    int frame = totalFrame / 2;
    Vector3 velocity = Vector3(0.0f, 0.0f, 0.0f);
    float omega = 0.0f;
    Vector3 lockedVelocity;
    float lockedOmega;
    GaitStatus gaitStatus = Stop;
    Vector3 lockedR;
    float timeStep;

    float currentPitch = 0.0f;
    float currentRoll = 0.0f;

    bool isBanlanced = true;


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

    // void move(Vector3 velocity, float omega, float timeStep);
    void moveTripod();
    void moveWave();
    void moveRipple();
    void move4plus2();
    Vector3 getSwagNextBodyTarget( Vector3 r0, const Vector3& currentStandBodyTarget);
    // Vector3 getSwagNextBodyTarget(Leg* leg, Vector3 r0, const Vector3& currentStandBodyTarget);
    Vector3 getStandNextBodyTarget(Vector3 r0, const Vector3& currentStandBodyTarget, float baseRatio = 1.0f,
                                   float ratio = 0.0f);

    bool isGaitCycleFinish();
    bool isGaitCycleStart();


    /// <summary>
    /// 所有的设置目标点的函数并不会使机器人直接运动，需要设置完目标点后调用startMove()函数开始运动
    /// </summary>
    void setBodyTargets(Vector3 BRtarget, Vector3 MRtarget, Vector3 FRtarget, Vector3 BLtarget, Vector3 MLtarget,
                        Vector3 FLtarget);

    void reInit();
    void reBalance();

    /// <summary>
    /// 设置机器人高度
    /// </summary>
    void setBodyPosition(Vector3 bodyPos);
    void setHeight(float height);
    void setYaw(float yaw);
    void setRoll(float roll);
    void setPitch(float pitch);
    void balance();
    bool checkBalance();
    void toGround();

    /// <summary>
    /// 开始按照设置的角度和目标点运动
    /// </summary>
    void startMove();
    /// <summary>
    /// 将以腿根部为原点的相对向量(腿坐标系)转换为以机器人身体中心为原点指向目标点的向量(机器人坐标系)
    /// </summary>
    /// <param name="relevant">以腿根部为原点的相对向量(腿坐标系)</param>
    /// <param name="bias">以机器人身体中心为原点指向腿根部的向量(机器人坐标系)</param>
    /// <param name="theta">以机器人身体中心为原点坐标系下，腿根部为原点的坐标系的旋转角</param>
    /// <returns>以机器人身体中心为原点指向目标点的向量(机器人坐标系)</returns>
    static Vector3 leg2bodyCoord(Vector3 relevant, Vector3 bias, float theta);

    /// <summary>
    /// 将以机器人身体中心为原点指向目标点的向量(机器人坐标系)转换为以腿根部为原点的相对向量(腿坐标系)
    /// </summary>
    /// <param name="absolute">以机器人身体中心为原点指向目标点的向量(机器人坐标系)</param>
    /// <param name="bias">以机器人身体中心为原点指向腿根部的向量(机器人坐标系)</param>
    /// <param name="theta">以机器人身体中心为原点坐标系下，腿根部为原点的坐标系的旋转角</param>
    /// <returns>以腿根部为原点的相对向量(腿坐标系)</returns>
    static Vector3 body2legCoord(Vector3 absolute, Vector3 bias, float theta);
    bool prepareNextCycle(GaitStatus moveStatus);

    void checkIsOnGround();
};
