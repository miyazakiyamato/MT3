#pragma once
#include "Vector3.h"
class MyMtVector3{
public:
	static Vector3 Add(const Vector3& v1, const Vector3& v2);
	static Vector3 Subtract(const Vector3& v1, const Vector3& v2); 
	static Vector3 Multiply(float scalar, const Vector3& v); 
	static float Dot(const Vector3& v1, const Vector3& v2);
	static float Length(const Vector3& v);
	static Vector3 Normalize(const Vector3& v);
};

