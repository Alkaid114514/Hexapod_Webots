#include "LegL.h"
#include "Hexapod.h"


LegL::LegL(): ctr2rootTheta(0)
{
    this->coxaMotor = nullptr;
    this->femurMotor = nullptr;
    this->tibiaMotor = nullptr;
}

LegL::LegL(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia, Vector3 ctr2root, float ctr2rootTheta)
{
    this->coxaMotor = coxa;
    this->femurMotor = femur;
    this->tibiaMotor = tibia;
    this->ctr2root = ctr2root;
    this->ctr2rootTheta = ctr2rootTheta;
    this->initStandBodyTarget = Hexapod::leg2bodyCoord(fk(initAngles), ctr2root, ctr2rootTheta);
    this->currentStandBodyTarget = initStandBodyTarget;
    this->lastBodyTarget = initStandBodyTarget;
}


LegL::~LegL()
{
}

Vector3 LegL::fk(Vector3 angles)
{
    float tmp = (COXA_LEN + FEMUR_LEN * cos(angles.y) + TIBIA_LEN * cos(-angles.z - angles.y));
    auto v = Vector3(
        tmp * cos(angles.x),
        tmp * sin(angles.x),
        -FEMUR_LEN * sin(angles.y) + TIBIA_LEN * sin(-angles.z - angles.y)
    );
    return v;
}

Vector3 LegL::ik(Vector3 vector3)
{
    vector3.z = -vector3.z;
    float theta1 = atan2(vector3.y, vector3.x);
    float R = sqrt(vector3.x * vector3.x + vector3.y * vector3.y);
    float ar = atan2(-vector3.z, (R - COXA_LEN));
    float Lr = sqrt(vector3.z * vector3.z + (R - COXA_LEN) * (R - COXA_LEN));
    float a1 = acos(CLIP((FEMUR_LEN * FEMUR_LEN + Lr * Lr - TIBIA_LEN * TIBIA_LEN) / (2 * Lr * FEMUR_LEN), -1, 1));
    float theta2 = a1 - ar;
    float a2 = acos(CLIP((Lr * Lr + TIBIA_LEN * TIBIA_LEN - FEMUR_LEN * FEMUR_LEN) / (2 * Lr * TIBIA_LEN), -1, 1));
    float theta3 = -(a1 + a2);
    return Vector3(theta1, -theta2, theta3);
}

void LegL::setMotor(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia)
{
    this->coxaMotor = coxa;
    this->femurMotor = femur;
    this->tibiaMotor = tibia;
}

void LegL::setOmega(float* omega)
{
    this->coxaMotor->setVelocity(omega[0]);
    this->femurMotor->setVelocity(omega[1]);
    this->tibiaMotor->setVelocity(omega[2]);
}

void LegL::setCoxaOmega(float omega)
{
    this->coxaMotor->setVelocity(omega);
}

void LegL::setFemurOmega(float omega)
{
    this->femurMotor->setVelocity(omega);
}

void LegL::setTibiaOmega(float omega)
{
    this->tibiaMotor->setVelocity(omega);
}

void LegL::setLegTarget(Vector3 target)
{
    setPose(ik(target));
}


void LegL::setBodyTarget(Vector3 target)
{
    lastBodyTarget = target;
    setLegTarget(Hexapod::body2legCoord(target, ctr2root, ctr2rootTheta));
}

void LegL::setPose(Vector3 angles)
{
    angles.z += static_cast<float>(M_PI) / 3.0f;
    this->motorAngles = angles;
}

void LegL::setCoxaPose(float angle)
{
    this->motorAngles.x = angle;
}

void LegL::setFemurPose(float angle)
{
    this->motorAngles.y = angle;
}

void LegL::setTibiaPose(float angle)
{
    angle += static_cast<float>(M_PI) / 3.0f;
    this->motorAngles.z = angle;
}

void LegL::reInit()
{
    setPose(currentStandAngles);
}

void LegL::setHeight(float height)
{
    currentStandBodyTarget.z = -height;
    lastBodyTarget = currentStandBodyTarget;
    this->currentStandAngles = ik(Hexapod::body2legCoord(currentStandBodyTarget, ctr2root, ctr2rootTheta));
}

void LegL::setYaw(float yaw)
{
    ctr2rootTheta += yaw - currentYaw;
    currentYaw = yaw;
    ctr2root = Vector3(
        cos(yaw) * (ctr2root.x) - sin(yaw) * (ctr2root.y),
        sin(yaw) * (ctr2root.x) + cos(yaw) * (ctr2root.y),
        ctr2root.z
    );
    currentStandAngles = ik(Hexapod::body2legCoord(currentStandBodyTarget, ctr2root, ctr2rootTheta));
}

void LegL::startMotor()
{
    this->coxaMotor->setPosition(motorAngles.x);
    this->femurMotor->setPosition(motorAngles.y);
    this->tibiaMotor->setPosition(motorAngles.z);
}
