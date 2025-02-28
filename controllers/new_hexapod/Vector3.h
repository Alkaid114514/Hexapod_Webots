#pragma once

#include <math.h>

class Vector3
{
public:
	float x;
	float y;
	float z;
	Vector3();
	Vector3(float x, float y, float z);
	Vector3(int x, int y, int z);
	~Vector3();
	void setVector(float x, float y, float z);
	Vector3 operator+(Vector3 vector);
	Vector3 operator-(Vector3 vector);
	Vector3 operator-();
	Vector3 operator*(float scalar);
	Vector3 operator/(float scalar);
	Vector3 operator+=(Vector3 vector);
	Vector3 operator-=(Vector3 vector);
	Vector3 operator*=(float scalar);
	Vector3 operator/=(float scalar);
	bool operator==(Vector3 vector);
	bool operator!=(Vector3 vector);
	float magnitude();
	Vector3 normalize();
	Vector3 cross(Vector3 vectorR);
	float dot(Vector3 vector);
};

