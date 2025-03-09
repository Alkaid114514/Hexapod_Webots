#include "Hexapod.h"

Hexapod::Hexapod() : Robot()
{
    imu = IMU(this);
    timeStep = (float)this->getBasicTimeStep();
    
    BRleg = LegR(getMotor("M_BR_COXA"), getMotor("M_BR_FEMUR"), getMotor("M_BR_TIBIA"), ctr2BRroot, ctr2BRrootTheta);
    MRleg = LegR(getMotor("M_MR_COXA"), getMotor("M_MR_FEMUR"), getMotor("M_MR_TIBIA"), ctr2MRroot, ctr2MRrootTheta);
    FRleg = LegR(getMotor("M_FR_COXA"), getMotor("M_FR_FEMUR"), getMotor("M_FR_TIBIA"), ctr2FRroot, ctr2FRrootTheta);
    BLleg = LegL(getMotor("M_BL_COXA"), getMotor("M_BL_FEMUR"), getMotor("M_BL_TIBIA"), ctr2BLroot, ctr2BLrootTheta);
    MLleg = LegL(getMotor("M_ML_COXA"), getMotor("M_ML_FEMUR"), getMotor("M_ML_TIBIA"), ctr2MLroot, ctr2MLrootTheta);
    FLleg = LegL(getMotor("M_FL_COXA"), getMotor("M_FL_FEMUR"), getMotor("M_FL_TIBIA"), ctr2FLroot, ctr2FLrootTheta);

    currentHeight = initHeight = body2legCoord(FLleg.initStandBodyTarget, FLleg.ctr2root, FLleg.ctr2rootTheta).z;
}

Hexapod::~Hexapod()
{
}

void Hexapod::setPose(Vector3 BRangles, Vector3 MRangles, Vector3 FRangles,
                      Vector3 BLangles, Vector3 MLangles, Vector3 FLangles)
{
    BRleg.setPose(BRangles);
    MRleg.setPose(MRangles);
    FRleg.setPose(FRangles);
    BLleg.setPose(BLangles);
    MLleg.setPose(MLangles);
    FLleg.setPose(FLangles);
}


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

// void Hexapod::move(Vector3 velocity, float omega, float timeStep)
// {
//     if (velocity == Vector3() && omega == 0.0)
//     {
//         return;
//     }
//
//     auto w = Vector3(0.0, 0.0, omega);
//     Vector3 r0 = velocity.cross(w) / (omega * omega);
//
//     // switch (gaitType)
//     // {
//     // 	case Tripod:
//     // 		
//     // 		break;
//     // 	case Ripple:
//     // 		break;
//     // 	case Wave:
//     // 		break;
//     // }
//     //
//     // static float t=0.0;
//     // float frequency = 1.0f;
//     // float T = 0.5f;//从抬腿到落下
//     // float A = FEMUR_LEN * sin(omega2) * T-(omega3 * TIBIA_LEN * sin(omega3 - omega2) * T) /( 2 * (omega3 - omega2));
//     // float lambda = 2 * omega1 * COXA_LEN * sin(T * omega / 2) * T / (T * omega);
//     // float Phase;
//     // int resolution = 20;
//     //
//     //
//     //
//     //
//     // t += timeStep;
//     //
//     // Vector3 initTmpFL = r0 + FLleg.currentStandBodyTarget;
//     // // Vector3 initTmpML = r0 + MLleg.currentStandBodyTarget;
//     // // Vector3 initTmpBL = r0 + BLleg.currentStandBodyTarget;
//     // // Vector3 initTmpFR = r0 + FRleg.currentStandBodyTarget;
//     // // Vector3 initTmpMR = r0 + MRleg.currentStandBodyTarget;
//     // // Vector3 initTmpBR = r0 + BRleg.currentStandBodyTarget;
//     // float alpha = omega * T;
//     // for (int n = 1; n < resolution;n++) {
//     // 	float tmp_alpha = alpha*n/resolution;
//     // 	float  tmpz = initTmpFL.z + A * sin(2*M_PI*frequency*t);
//     // 	
//     // 	
//     // 	Vector3 tmpFL = Vector3(initTmpFL.x * cos(tmp_alpha) - initTmpFL.y * sin(tmp_alpha),  // NOLINT(clang-diagnostic-invalid-utf8)
//     // 		initTmpFL.x * sin(tmp_alpha) + initTmpFL.y * cos(tmp_alpha),
//     // 		tmpz);
//     // 	/*Vector3 tmpML = Vector3(initTmpML.x * cos(tmp_alpha) - initTmpML.y * sin(tmp_alpha),
//     // 		initTmpML.x * sin(tmp_alpha) + initTmpML.y * cos(tmp_alpha),
//     // 		tmpz);
//     // 	Vector3 tmpBL = Vector3(initTmpBL.x * cos(tmp_alpha) - initTmpBL.y * sin(tmp_alpha),
//     // 		initTmpBL.x * sin(tmp_alpha) + initTmpBL.y * cos(tmp_alpha),
//     // 		tmpz);
//     // 	Vector3 tmpFR = Vector3(initTmpFR.x * cos(tmp_alpha) - initTmpFR.y * sin(tmp_alpha),
//     // 		initTmpFR.x * sin(tmp_alpha) + initTmpFR.y * cos(tmp_alpha),
//     // 		tmpz);
//     // 	Vector3 tmpMR = Vector3(initTmpMR.x * cos(tmp_alpha) - initTmpMR.y * sin(tmp_alpha),
//     // 		initTmpMR.x * sin(tmp_alpha) + initTmpMR.y * cos(tmp_alpha),
//     // 		tmpz);
//     // 	Vector3 tmpBR = Vector3(initTmpBR.x * cos(tmp_alpha) - initTmpBR.y * sin(tmp_alpha),
//     // 		initTmpBR.x * sin(tmp_alpha) + initTmpBR.y * cos(tmp_alpha),
//     // 		tmpz);*/
//     // 	
//     // /*	Vector3 targetFL = body2legCoord(tmpFL - initTmpFL, ctr2FLroot, ctr2FLrootTheta) + initStandFL;  // NOLINT(clang-diagnostic-invalid-utf8)
//     // 	Vector3 targetML = body2legCoord(tmpML - initTmpML, ctr2MLroot, ctr2MLrootTheta) + initStandML;
//     // 	Vector3 targetBL = body2legCoord(tmpBL - initTmpBL, ctr2BLroot, ctr2BLrootTheta) + initStandBL;
//     // 	Vector3 targetFR = body2legCoord(tmpFR - initTmpFR, ctr2FRroot, ctr2FRrootTheta) + initStandFR;
//     // 	Vector3 targetMR = body2legCoord(tmpMR - initTmpMR, ctr2MRroot, ctr2MRrootTheta) + initStandMR;
//     // 	Vector3 targetBR = body2legCoord(tmpBR - initTmpBR, ctr2BRroot, ctr2BRrootTheta) + initStandBR;*/
//     //
//     // 	//移动过程中的脚尖目标点（连起来就是轨迹，在机器人身体坐标系下）  // NOLINT(clang-diagnostic-invalid-utf8)
//     // 	Vector3 FLtarget = Vector3(tmpFL.x - r0.x, tmpFL.y - r0.y, tmpz);
//     // /*	Vector3 MLtarget = Vector3(tmpML.x - r0.x, tmpML.y - r0.y, tmpz);
//     // 	Vector3 BLtarget = Vector3(tmpBL.x - r0.x, tmpBL.y - r0.y, tmpz);
//     // 	Vector3 FRtarget = Vector3(tmpFR.x - r0.x, tmpFR.y - r0.y, tmpz);
//     // 	Vector3 MRtarget = Vector3(tmpMR.x - r0.x, tmpMR.y - r0.y, tmpz);
//     // 	Vector3 BRtarget = Vector3(tmpBR.x - r0.x, tmpBR.y - r0.y, tmpz);*/
//     //
//     // 	FLleg.setBodyTarget(FLtarget);
//
//
//     // }
// }

void Hexapod::prepareNextCycle(GaitStatus moveStatus)
{
    if (isGaitCycleStart() || gaitStatus == Stop)
    {
        if (velocity == Vector3() && omega == 0.0)
        {
            reInit();
            startMove();
            gaitStatus = Stop;
            return;
        }    
        lockedOmega = omega == 0.0f ? FLT_EPSILON : omega;
        auto w = Vector3(0.0, 0.0, lockedOmega);
        lockedVelocity = velocity;
        bool isVzero = lockedVelocity == Vector3();
        stepLen = (isVzero ? lockedOmega * MRleg.currentStandBodyTarget.x : (lockedOmega / abs(lockedOmega)) * lockedVelocity.magnitude()) * (timeStep / 1000.0f) * (float)totalFrame;
        lockedR = lockedVelocity.cross(w) / (lockedOmega * lockedOmega);
        auto rlen = isVzero ? MRleg.currentStandBodyTarget.x : lockedR.magnitude();
        stepTheta = stepLen / rlen;
        gaitStatus = moveStatus;
    }
}

void Hexapod::moveRipple()
{
    prepareNextCycle(Ripple);
     if (gaitGroupIndex == 0)
    {
        FLleg.setBodyTarget(getSwagNextBodyTarget(lockedR, FLleg.currentStandBodyTarget));
         MRleg.setBodyTarget(getSwagNextBodyTarget(lockedR, MRleg.currentStandBodyTarget));
        FRleg.setBodyTarget(getStandNextBodyTarget(lockedR, FRleg.currentStandBodyTarget,RIPPLE_RATIO,1.0f * RIPPLE_RATIO));
        MLleg.setBodyTarget(getStandNextBodyTarget(lockedR, MLleg.currentStandBodyTarget,RIPPLE_RATIO,2.0f * RIPPLE_RATIO));
        BRleg.setBodyTarget(getStandNextBodyTarget(lockedR,BRleg.currentStandBodyTarget,RIPPLE_RATIO,2.0f * RIPPLE_RATIO));
        BLleg.setBodyTarget(getStandNextBodyTarget(lockedR, BLleg.currentStandBodyTarget,RIPPLE_RATIO,1.0f * RIPPLE_RATIO));

        if (isGaitCycleFinish())
        {
            frame = -1;
            gaitGroupIndex++;
        }
    }
    else if (gaitGroupIndex == 1)
    {
        MLleg.setBodyTarget(getSwagNextBodyTarget(lockedR, MLleg.currentStandBodyTarget));
        BRleg.setBodyTarget(getSwagNextBodyTarget(lockedR, BRleg.currentStandBodyTarget));
        FLleg.setBodyTarget(getStandNextBodyTarget(lockedR, FLleg.currentStandBodyTarget,RIPPLE_RATIO,1.0f * RIPPLE_RATIO));
        FRleg.setBodyTarget(getStandNextBodyTarget(lockedR, FRleg.currentStandBodyTarget,RIPPLE_RATIO,2.0f * RIPPLE_RATIO));
        MRleg.setBodyTarget(getStandNextBodyTarget(lockedR, MRleg.currentStandBodyTarget,RIPPLE_RATIO,1.0f * RIPPLE_RATIO));
        BLleg.setBodyTarget(getStandNextBodyTarget(lockedR, BLleg.currentStandBodyTarget,RIPPLE_RATIO,2.0f * RIPPLE_RATIO));
        

        if (isGaitCycleFinish())
        {
            frame = -1;
            gaitGroupIndex++;
        }
    }
    else if (gaitGroupIndex == 2)
    {
        BLleg.setBodyTarget(getSwagNextBodyTarget(lockedR, BLleg.currentStandBodyTarget));
        FRleg.setBodyTarget(getSwagNextBodyTarget(lockedR, FRleg.currentStandBodyTarget));
        MLleg.setBodyTarget(getStandNextBodyTarget(lockedR, MLleg.currentStandBodyTarget,RIPPLE_RATIO,1.0f * RIPPLE_RATIO));
        MRleg.setBodyTarget(getStandNextBodyTarget(lockedR, MRleg.currentStandBodyTarget,RIPPLE_RATIO,2.0f * RIPPLE_RATIO));
        FLleg.setBodyTarget(getStandNextBodyTarget(lockedR, FLleg.currentStandBodyTarget,RIPPLE_RATIO,2.0f * RIPPLE_RATIO));
        BRleg.setBodyTarget(getStandNextBodyTarget(lockedR, BRleg.currentStandBodyTarget,RIPPLE_RATIO,1.0f * RIPPLE_RATIO));

        if (isGaitCycleFinish())
        {
            frame = -1;
            gaitGroupIndex-=2;
        }
    }
    startMove();
    frame++;
}


void Hexapod::moveWave()
{
    prepareNextCycle(Wave);
    if (gaitGroupIndex == 0)
    {
        FLleg.setBodyTarget(getSwagNextBodyTarget(lockedR, FLleg.currentStandBodyTarget));
        FRleg.setBodyTarget(getStandNextBodyTarget(lockedR, FRleg.currentStandBodyTarget,WAVE_RATIO,5.0f * WAVE_RATIO));
        MLleg.setBodyTarget(getStandNextBodyTarget(lockedR, MLleg.currentStandBodyTarget,WAVE_RATIO,4.0f * WAVE_RATIO));
        MRleg.setBodyTarget(getStandNextBodyTarget(lockedR, MRleg.currentStandBodyTarget,WAVE_RATIO,3.0f * WAVE_RATIO));
        BLleg.setBodyTarget(getStandNextBodyTarget(lockedR, BLleg.currentStandBodyTarget,WAVE_RATIO,2.0f * WAVE_RATIO));
        BRleg.setBodyTarget(getStandNextBodyTarget(lockedR, BRleg.currentStandBodyTarget,WAVE_RATIO,1.0f * WAVE_RATIO));

        if (isGaitCycleFinish())
        {
            frame = -1;
            gaitGroupIndex++;
        }
    }
    else if (gaitGroupIndex == 1)
    {
        FLleg.setBodyTarget(getStandNextBodyTarget(lockedR, FLleg.currentStandBodyTarget,WAVE_RATIO,1.0f * WAVE_RATIO));
        FRleg.setBodyTarget(getSwagNextBodyTarget(lockedR, FRleg.currentStandBodyTarget));
        MLleg.setBodyTarget(getStandNextBodyTarget(lockedR, MLleg.currentStandBodyTarget,WAVE_RATIO,5.0f * WAVE_RATIO));
        MRleg.setBodyTarget(getStandNextBodyTarget(lockedR, MRleg.currentStandBodyTarget,WAVE_RATIO,4.0f * WAVE_RATIO));
        BLleg.setBodyTarget(getStandNextBodyTarget(lockedR, BLleg.currentStandBodyTarget,WAVE_RATIO,3.0f * WAVE_RATIO));
        BRleg.setBodyTarget(getStandNextBodyTarget(lockedR, BRleg.currentStandBodyTarget,WAVE_RATIO,2.0f * WAVE_RATIO));

        if (isGaitCycleFinish())
        {
            frame = -1;
            gaitGroupIndex++;
        }
    }
    else if (gaitGroupIndex == 2)
    {
        FLleg.setBodyTarget(getStandNextBodyTarget(lockedR, FLleg.currentStandBodyTarget,WAVE_RATIO,2.0f * WAVE_RATIO));
        FRleg.setBodyTarget(getStandNextBodyTarget(lockedR, FRleg.currentStandBodyTarget,WAVE_RATIO,1.0f * WAVE_RATIO));
        MLleg.setBodyTarget(getSwagNextBodyTarget(lockedR, MLleg.currentStandBodyTarget));
        MRleg.setBodyTarget(getStandNextBodyTarget(lockedR, MRleg.currentStandBodyTarget,WAVE_RATIO,5.0f * WAVE_RATIO));
        BLleg.setBodyTarget(getStandNextBodyTarget(lockedR, BLleg.currentStandBodyTarget,WAVE_RATIO,4.0f * WAVE_RATIO));
        BRleg.setBodyTarget(getStandNextBodyTarget(lockedR, BRleg.currentStandBodyTarget,WAVE_RATIO,3.0f * WAVE_RATIO));

        if (isGaitCycleFinish())
        {
            frame = -1;
            gaitGroupIndex++;
        }
    }
    else if (gaitGroupIndex == 3)
    {
        FLleg.setBodyTarget(getStandNextBodyTarget(lockedR, FLleg.currentStandBodyTarget,WAVE_RATIO,3.0f * WAVE_RATIO));
        FRleg.setBodyTarget(getStandNextBodyTarget(lockedR, FRleg.currentStandBodyTarget,WAVE_RATIO,2.0f * WAVE_RATIO));
        MLleg.setBodyTarget(getStandNextBodyTarget(lockedR, MLleg.currentStandBodyTarget,WAVE_RATIO,1.0f * WAVE_RATIO));
        MRleg.setBodyTarget(getSwagNextBodyTarget(lockedR, MRleg.currentStandBodyTarget));
        BLleg.setBodyTarget(getStandNextBodyTarget(lockedR, BLleg.currentStandBodyTarget,WAVE_RATIO,5.0f * WAVE_RATIO));
        BRleg.setBodyTarget(getStandNextBodyTarget(lockedR, BRleg.currentStandBodyTarget,WAVE_RATIO,4.0f * WAVE_RATIO));

        if (isGaitCycleFinish())
        {
            frame = -1;
            gaitGroupIndex++;
        }
    }
    else if (gaitGroupIndex == 4)
    {
        FLleg.setBodyTarget(getStandNextBodyTarget(lockedR, FLleg.currentStandBodyTarget,WAVE_RATIO,4.0f * WAVE_RATIO));
        FRleg.setBodyTarget(getStandNextBodyTarget(lockedR, FRleg.currentStandBodyTarget,WAVE_RATIO,3.0f * WAVE_RATIO));
        MLleg.setBodyTarget(getStandNextBodyTarget(lockedR, MLleg.currentStandBodyTarget,WAVE_RATIO,2.0f * WAVE_RATIO));
        MRleg.setBodyTarget(getStandNextBodyTarget(lockedR, MRleg.currentStandBodyTarget,WAVE_RATIO,1.0f * WAVE_RATIO));
        BLleg.setBodyTarget(getSwagNextBodyTarget(lockedR, BLleg.currentStandBodyTarget));
        BRleg.setBodyTarget(getStandNextBodyTarget(lockedR, BRleg.currentStandBodyTarget,WAVE_RATIO,5.0f * WAVE_RATIO));

        if (isGaitCycleFinish())
        {
            frame = -1;
            gaitGroupIndex++;
        }
    }
    else if (gaitGroupIndex == 5)
    {
        FLleg.setBodyTarget(getStandNextBodyTarget(lockedR, FLleg.currentStandBodyTarget,WAVE_RATIO,5.0f * WAVE_RATIO));
        FRleg.setBodyTarget(getStandNextBodyTarget(lockedR, FRleg.currentStandBodyTarget,WAVE_RATIO,4.0f * WAVE_RATIO));
        MLleg.setBodyTarget(getStandNextBodyTarget(lockedR, MLleg.currentStandBodyTarget,WAVE_RATIO,3.0f * WAVE_RATIO));
        MRleg.setBodyTarget(getStandNextBodyTarget(lockedR, MRleg.currentStandBodyTarget,WAVE_RATIO,2.0f * WAVE_RATIO));
        BLleg.setBodyTarget(getStandNextBodyTarget(lockedR, BLleg.currentStandBodyTarget,WAVE_RATIO,1.0f * WAVE_RATIO));
        BRleg.setBodyTarget(getSwagNextBodyTarget(lockedR, BRleg.currentStandBodyTarget));

        if (isGaitCycleFinish())
        {
            frame = -1;
            gaitGroupIndex -= 5; 
        }
    }

    
    startMove();
    frame++;
}

void Hexapod::moveTripod()
{
    prepareNextCycle(Tripod);
    if (gaitGroupIndex == 0)
    {
        MLleg.setBodyTarget(getStandNextBodyTarget( lockedR, MLleg.currentStandBodyTarget));
        FRleg.setBodyTarget(getStandNextBodyTarget( lockedR, FRleg.currentStandBodyTarget));
        BRleg.setBodyTarget(getStandNextBodyTarget(lockedR,  BRleg.currentStandBodyTarget));

        MRleg.setBodyTarget(getSwagNextBodyTarget( lockedR, MRleg.currentStandBodyTarget));
        FLleg.setBodyTarget(getSwagNextBodyTarget(lockedR, FLleg.currentStandBodyTarget));
        BLleg.setBodyTarget(getSwagNextBodyTarget( lockedR, BLleg.currentStandBodyTarget));

        if (isGaitCycleFinish())
        {
            frame = -1;
            gaitGroupIndex++;
        }
    }
    else if (gaitGroupIndex == 1)
    {
        MRleg.setBodyTarget(getStandNextBodyTarget( lockedR, MRleg.currentStandBodyTarget));
        FLleg.setBodyTarget(getStandNextBodyTarget(lockedR, FLleg.currentStandBodyTarget));
        BLleg.setBodyTarget(getStandNextBodyTarget(lockedR,BLleg.currentStandBodyTarget));

        MLleg.setBodyTarget(getSwagNextBodyTarget(lockedR, MLleg.currentStandBodyTarget));
        FRleg.setBodyTarget(getSwagNextBodyTarget(lockedR,FRleg.currentStandBodyTarget));
        BRleg.setBodyTarget(getSwagNextBodyTarget(lockedR,BRleg.currentStandBodyTarget));

        if (isGaitCycleFinish())
        {
            frame = -1;
            gaitGroupIndex--;
        }
    }
    
    startMove();
    frame++;
}


Vector3 Hexapod::getSwagNextBodyTarget(Vector3 r0,Vector3 currentStandBodyTarget)
{
    auto z = -currentHeight;
    currentStandBodyTarget.z = 0.0;
    Vector3 pc = r0 + currentStandBodyTarget;
    float theta = stepTheta * (float)frame / (float)totalFrame - stepTheta / 2.0f;
    auto pt = Vector3(
        cos(theta) * (pc.x) - sin(theta) * (pc.y),
        sin(theta) * (pc.x) + cos(theta) * (pc.y),
        pc.z
    );
    auto t = pt - r0;
    t.z = z + 0.06f * static_cast<float>(sin(((float)frame / (float)totalFrame) * M_PI));
    return t;
}

Vector3 Hexapod::getStandNextBodyTarget(Vector3 r0,Vector3 currentStandBodyTarget,float baseRatio,float ratio)
{
    auto z = -currentHeight;
    currentStandBodyTarget.z = 0.0;
    Vector3 pc = r0 + currentStandBodyTarget;
    float theta =stepTheta * ratio + stepTheta * (baseRatio) * (float)frame / (float)totalFrame  - stepTheta / 2.0f ;
    auto pt = Vector3(
        cos(theta) * (pc.x) + sin(theta) * (pc.y),
        -sin(theta) * (pc.x) + cos(theta) * (pc.y),
        pc.z
    );
    auto t = pt - r0;
    t.z = z;
    return t;
}

bool Hexapod::isGaitCycleFinish()
{
    return frame > totalFrame - 1;
}

bool Hexapod::isGaitCycleStart()
{
    return frame < 1;
}

void Hexapod::setBodyTargets(Vector3 BRtarget, Vector3 MRtarget, Vector3 FRtarget, Vector3 BLtarget, Vector3 MLtarget,
                             Vector3 FLtarget)
{
    BRleg.setLegTarget(body2legCoord(BRtarget, BRleg.ctr2root, BRleg.ctr2rootTheta));
    MRleg.setLegTarget(body2legCoord(MRtarget, MRleg.ctr2root, MRleg.ctr2rootTheta));
    FRleg.setLegTarget(body2legCoord(FRtarget, FRleg.ctr2root, FRleg.ctr2rootTheta));
    BLleg.setLegTarget(body2legCoord(BLtarget, BLleg.ctr2root, BLleg.ctr2rootTheta));
    MLleg.setLegTarget(body2legCoord(MLtarget, MLleg.ctr2root, MLleg.ctr2rootTheta));
    FLleg.setLegTarget(body2legCoord(FLtarget, FLleg.ctr2root, FLleg.ctr2rootTheta));
}

void Hexapod::reInit()
{
    BRleg.reInit();
    MRleg.reInit();
    FRleg.reInit();
    BLleg.reInit();
    MLleg.reInit();
    FLleg.reInit();
}

void Hexapod::setHeight(float height)
{
    currentHeight = height;

    this->BRleg.setHeight(height);
    this->MRleg.setHeight(height);
    this->FRleg.setHeight(height);
    this->BLleg.setHeight(height);
    this->MLleg.setHeight(height);
    this->FLleg.setHeight(height);
}

void Hexapod::setYaw(float yaw)
{
    BRleg.setYaw(yaw);
    MRleg.setYaw(yaw);
    FRleg.setYaw(yaw);
    BLleg.setYaw(yaw);
    MLleg.setYaw(yaw);
    FLleg.setYaw(yaw);
}

void Hexapod::setPitch(float pitch)
{
    BRleg.setPitch(pitch);
    MRleg.setPitch(pitch);
    FRleg.setPitch(pitch);
    BLleg.setPitch(pitch);
    MLleg.setPitch(pitch);
    FLleg.setPitch(pitch);
}

void Hexapod::setRoll(float roll)
{
    BRleg.setRoll(roll);
    MRleg.setRoll(roll);
    FRleg.setRoll(roll);
    BLleg.setRoll(roll);
    MLleg.setRoll(roll);
    FLleg.setRoll(roll);
}

void Hexapod::balance()
{
    float target_pitch = 0.0;  
    float target_roll = 0.0;   
    float current_pitch = imu.getPitch();
    float current_roll = imu.getRoll();
    float c2t_pitch =  current_pitch - target_pitch;
    float c2t_roll = current_roll - target_roll;
    setPitch(c2t_pitch);
    setRoll(c2t_roll);
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
