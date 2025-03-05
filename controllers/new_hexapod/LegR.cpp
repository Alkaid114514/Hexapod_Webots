#include "LegR.h"
#include "Hexapod.h"

LegR::LegR()
{
    this->coxaMotor = nullptr;
    this->femurMotor = nullptr;
    this->tibiaMotor = nullptr;
}

LegR::LegR(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia, Vector3 ctr2root, float ctr2rootTheta)
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

LegR::~LegR()
{
}

Vector3 LegR::fk(Vector3 angles)
{
    float tmp = (COXA_LEN + FEMUR_LEN * cos(angles.y) + TIBIA_LEN * cos(angles.z - angles.y));
    auto v = Vector3(
        tmp * cos(angles.x),
        tmp * sin(angles.x),
        -FEMUR_LEN * sin(angles.y) + TIBIA_LEN * sin(angles.z - angles.y)
    );
    return v;
}

Vector3 LegR::ik(Vector3 vector3)
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
    return Vector3(theta1, theta2, -theta3);
}

void LegR::setMotor(webots::Motor* coxa, webots::Motor* femur, webots::Motor* tibia)
{
    this->coxaMotor = coxa;
    this->femurMotor = femur;
    this->tibiaMotor = tibia;
}

void LegR::setOmega(float* omega)
{
    this->coxaMotor->setVelocity(omega[0]);
    this->femurMotor->setVelocity(omega[1]);
    this->tibiaMotor->setVelocity(omega[2]);
}

void LegR::setCoxaOmega(float omega)
{
    this->coxaMotor->setVelocity(omega);
}

void LegR::setFemurOmega(float omega)
{
    this->femurMotor->setVelocity(omega);
}

void LegR::setTibiaOmega(float omega)
{
    this->tibiaMotor->setVelocity(omega);
}

void LegR::setLegTarget(Vector3 target)
{
    setPose(ik(target));
}

void LegR::setBodyTarget(Vector3 target)
{
    lastBodyTarget = target;
    setLegTarget(Hexapod::body2legCoord(target, ctr2root, ctr2rootTheta));
}

void LegR::setPose(Vector3 angles)
{
    angles.z -= static_cast<float>(M_PI) / 3.0f;
    this->angles = angles;
}

void LegR::setCoxaPose(float angle)
{
    this->angles.x = angle;
}

void LegR::setFemurPose(float angle)
{
    this->angles.y = angle;
}

void LegR::setTibiaPose(float angle)
{
    angle -= static_cast<float>(M_PI) / 3.0f;
    this->angles.z = angle;
}

void LegR::reInit()
{
    setPose(currentStandAngles);
}

void LegR::setHeight(float height)
{
    currentStandBodyTarget.z = -height;
    lastBodyTarget = currentStandBodyTarget;
    this->currentStandAngles = ik(Hexapod::body2legCoord(currentStandBodyTarget, ctr2root, ctr2rootTheta));
}

void LegR::setYaw(float yaw)
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

void LegR::startMotor()
{
    this->coxaMotor->setPosition(angles.x);
    this->femurMotor->setPosition(angles.y);
    this->tibiaMotor->setPosition(angles.z);
}
