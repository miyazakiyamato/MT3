#include "MyMtVector3.h"
#include <math.h>
Vector3 MyMtVector3::Add(const Vector3& v1, const Vector3& v2) {
	Vector3 v3;
	v3.x = v1.x + v2.x;
	v3.y = v1.y + v2.y;
	v3.z = v1.z + v2.z;
	return v3;
}

Vector3 MyMtVector3::Subtract(const Vector3& v1, const Vector3& v2) {
	Vector3 v3;
	v3.x = v1.x - v2.x;
	v3.y = v1.y - v2.y;
	v3.z = v1.z - v2.z;
	return v3;
}

Vector3 MyMtVector3::Multiply(float scalar, const Vector3& v) {
	Vector3 v2;
	v2.x = v.x * scalar;
	v2.y = v.y * scalar;
	v2.z = v.z * scalar;
	return v2;
}
float MyMtVector3::Dot(const Vector3& v1, const Vector3& v2) {
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

float MyMtVector3::Length(const Vector3& v) {
	return sqrtf(Dot(v, v));
}

Vector3 MyMtVector3::Normalize(const Vector3& v) {
	Vector3 v2;
	v2.x = v.x / Length(v);
	v2.y = v.y / Length(v);
	v2.z = v.z / Length(v);
	return v2;
}
