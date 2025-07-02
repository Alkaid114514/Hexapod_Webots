#include "Leg.h"

#include "Hexapod.h"


Leg::Leg(): ctr2rootTheta(0)
{
    this->coxaMotor = nullptr;
    this->femurMotor = nullptr;
    this->tibiaMotor = nullptr;
}

Leg::Leg(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia, Vector3 ctr2root, float ctr2rootTheta)
{
    this->coxaMotor = coxa;
    this->femurMotor = femur;
    this->tibiaMotor = tibia;
    this->ctr2root = ctr2root;
    this->ctr2rootTheta = ctr2rootTheta;
}


Leg::~Leg()
{
}

void Leg::initialize()
{
    this->initStandBodyTarget = Hexapod::leg2bodyCoord(fk(initAngles), ctr2root, ctr2rootTheta);
    this->currentStandBodyTarget = initStandBodyTarget;
    this->lastBodyTarget = initStandBodyTarget;
}

void Leg::setMotor(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia)
{
    this->coxaMotor = coxa;
    this->femurMotor = femur;
    this->tibiaMotor = tibia;
}

void Leg::setOmega(float* omega)
{
    this->coxaMotor->setVelocity(omega[0]);
    this->femurMotor->setVelocity(omega[1]);
    this->tibiaMotor->setVelocity(omega[2]);
}

void Leg::setCoxaOmega(float omega)
{
    this->coxaMotor->setVelocity(omega);
}

void Leg::setFemurOmega(float omega)
{
    this->femurMotor->setVelocity(omega);
}

void Leg::setTibiaOmega(float omega)
{
    this->tibiaMotor->setVelocity(omega);
}

void Leg::setLegTarget(Vector3 target)
{
    setPose(ik(target));
}


void Leg::setBodyTarget(Vector3 target)
{
    lastBodyTarget = target;
    setLegTarget(Hexapod::body2legCoord(target, ctr2root, ctr2rootTheta));
}

void Leg::setCoxaPose(float angle)
{
    this->motorAngles.x = angle;
}

void Leg::setFemurPose(float angle)
{
    this->motorAngles.y = angle;
}

void Leg::reInit()
{
    setPose(currentStandAngles);
}

// void LegL::balance()
// {
//     setPose(ik(Hexapod::body2legCoord(lastBodyTarget, ctr2root, ctr2rootTheta)));
// }

void Leg::setHeight(float height)
{
    currentStandBodyTarget.z = -height;
    lastBodyTarget = currentStandBodyTarget;
    this->currentStandAngles = ik(Hexapod::body2legCoord(currentStandBodyTarget, ctr2root, ctr2rootTheta));
}

void Leg::setYaw(float yaw)
{
    // ctr2rootTheta += yaw - currentYaw;

    ctr2root = Vector3(
        cos(yaw - currentYaw) * (ctr2root.x) - sin(yaw - currentYaw) * (ctr2root.y),
        sin(yaw - currentYaw) * (ctr2root.x) + cos(yaw - currentYaw) * (ctr2root.y),
        ctr2root.z
    );
    currentYaw = yaw;
    currentStandAngles = ik(Hexapod::body2legCoord(currentStandBodyTarget, ctr2root, ctr2rootTheta));
}

void Leg::setRoll(float roll)
{
    currentStandBodyTarget = Vector3(
        currentStandBodyTarget.x * cos(roll - currentRoll) - currentStandBodyTarget.z * sin(roll - currentRoll),
        currentStandBodyTarget.y,
        currentStandBodyTarget.x * sin(roll - currentRoll) + currentStandBodyTarget.z * cos(roll - currentRoll)
    );

    // Vector3 a = currentStandBodyTarget + Vector3(0.0f,0.0f,height);
    // a = Vector3(
    // a.x * cos(roll - currentRoll) - a.z * sin(roll - currentRoll),
    // a.y,
    // a.x * sin(roll - currentRoll) + a.z * cos(roll - currentRoll));
    // currentStandBodyTarget = a - Vector3(0.0f,0.0f,height);

    // ctr2root = Vector3(ctr2root.x * cos(roll - currentRoll) - ctr2root.z * sin(roll - currentRoll),
    //                      ctr2root.y,
    //                      ctr2root.x * sin(roll - currentRoll) + ctr2root.z * cos(roll - currentRoll)); 
    currentRoll = roll;
    currentStandAngles = ik(Hexapod::body2legCoord(currentStandBodyTarget, ctr2root, ctr2rootTheta));
}

void Leg::setPitch(float pitch)
{
    Vector3 tmp = currentStandBodyTarget;
    currentStandBodyTarget = Vector3(tmp.x,
                                     tmp.y * cos(pitch - currentPitch) + tmp.z * sin(pitch - currentPitch),
                                     -tmp.y * sin(pitch - currentPitch) + tmp.z * cos(pitch - currentPitch));

    // Vector3 a = currentStandBodyTarget + Vector3(0.0f,0.0f,height);
    // a = Vector3(
    // a.x,
    // a.y*cos(pitch-currentPitch)+a.z*sin(pitch-currentPitch),
    // -a.y*sin(pitch-currentPitch)+a.z*cos(pitch-currentPitch)
    // );
    // currentStandBodyTarget = a - Vector3(0.0f,0.0f,height);


    // ctr2root = Vector3(ctr2root.x ,
    //                      ctr2root.y* cos(pitch - currentPitch) + ctr2root.z * sin(pitch - currentPitch),
    //                      -ctr2root.y * sin(pitch - currentPitch) + ctr2root.z * cos(pitch - currentPitch)); 
    currentPitch = pitch;
    currentStandAngles = ik(Hexapod::body2legCoord(currentStandBodyTarget, ctr2root, ctr2rootTheta));
}

void Leg::setBodyPosition(Vector3 bodyPos)
{
    bodyPos.z = 0.0f;
    ctr2root -= (bodyPos - currentPosition);
    currentPosition = bodyPos;
    currentStandAngles = ik(Hexapod::body2legCoord(currentStandBodyTarget, ctr2root, ctr2rootTheta));
}

void Leg::toGround(float height)
{
    isOnGround = (bool)(int)this->touchSensor->getValue() || -(this->lastBodyTarget.z) >= 1.5f * height;
    if (!isOnGround)
    {
        lastBodyTarget.z -= 0.003f;
        // currentStandBodyTarget.z -= 0.003f;
        this->setBodyTarget(lastBodyTarget);
    }
}

void Leg::checkIsOnGroud(float height)
{
    isOnGround = (bool)(int)this->touchSensor->getValue() || -(this->lastBodyTarget.z) >= 1.5f * height;
}

void Leg::moveToGround(float currentHeight)
{
}

void Leg::startMotor()
{
    this->coxaMotor->setPosition(motorAngles.x);
    this->femurMotor->setPosition(motorAngles.y);
    this->tibiaMotor->setPosition(motorAngles.z);
    isOnGround = (bool)(int)this->touchSensor->getValue();
}
