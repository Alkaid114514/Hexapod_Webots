#include "Vector3.h"

#include <cmath>


Vector3::Vector3()
{
    this->x = 0;
    this->y = 0;
    this->z = 0;
}

Vector3::Vector3(float x, float y, float z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

Vector3::Vector3(int x, int y, int z)
{
    this->x = static_cast<float>(x);
    this->y = static_cast<float>(y);
    this->z = static_cast<float>(z);
}

Vector3::~Vector3()
{
}

void Vector3::setVector(float x, float y, float z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

Vector3 Vector3::operator+(Vector3 vector)
{
    return Vector3(this->x + vector.x, this->y + vector.y, this->z + vector.z);
}

Vector3 Vector3::operator-(Vector3 vector)
{
    return Vector3(this->x - vector.x, this->y - vector.y, this->z - vector.z);
}

Vector3 Vector3::operator-()
{
    return Vector3(-(this->x), -(this->y), -(this->z));
}

Vector3 Vector3::operator*(float scalar)
{
    return Vector3(this->x * scalar, this->y * scalar, this->z * scalar);
}

Vector3 Vector3::operator/(float scalar)
{
    return Vector3(this->x / scalar, this->y / scalar, this->z / scalar);
}

Vector3 Vector3::operator+=(Vector3 vector)
{
    this->x += vector.x;
    this->y += vector.y;
    this->z += vector.z;
    return *this;
}

Vector3 Vector3::operator-=(Vector3 vector)
{
    this->x -= vector.x;
    this->y -= vector.y;
    this->z -= vector.z;
    return *this;
}

Vector3 Vector3::operator*=(float scalar)
{
    this->x *= scalar;
    this->y *= scalar;
    this->z *= scalar;
    return *this;
}

Vector3 Vector3::operator/=(float scalar)
{
    this->x /= scalar;
    this->y /= scalar;
    this->z /= scalar;
    return *this;
}

bool Vector3::operator==(Vector3 vector)
{
    return this->x == vector.x && this->y == vector.y && this->z == vector.z;
}

bool Vector3::operator!=(Vector3 vector)
{
    return this->x != vector.x || this->y != vector.y || this->z != vector.z;
}

float Vector3::magnitude()
{
    return (float)sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
}

float Vector3::squareMagnitude()
{
    return this->x * this->x + this->y * this->y + this->z * this->z;
}

Vector3 Vector3::normalize()
{
    float mag = this->magnitude();
    return Vector3(this->x / mag, this->y / mag, this->z / mag);
}

Vector3 Vector3::cross(Vector3 vectorR)
{
    return Vector3(this->y * vectorR.z - this->z * vectorR.y, this->z * vectorR.x - this->x * vectorR.z,
                   this->x * vectorR.y - this->y * vectorR.x);
}

float Vector3::dot(Vector3 vector)
{
    return this->x * vector.x + this->y * vector.y + this->z * vector.z;
}
