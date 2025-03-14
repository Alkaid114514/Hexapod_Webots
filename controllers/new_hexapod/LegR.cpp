#include "LegR.h"
#include "Hexapod.h"


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

void LegR::setPose(Vector3 angles)
{
    angles.z -= static_cast<float>(M_PI) / 3.0f;
    this->motorAngles = angles;
}

void LegR::setTibiaPose(float angle)
{
    angle -= static_cast<float>(M_PI) / 3.0f;
    this->motorAngles.z = angle;
}
