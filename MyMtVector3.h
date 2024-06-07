#pragma once
#include "Vector3.h"
struct Line {
	Vector3 origin;
	Vector3 diff;
};
struct Ray {
	Vector3 origin;
	Vector3 diff;
};
struct Segment {
	Vector3 origin;
	Vector3 diff;
};
struct Triangle
{
	Vector3 Vertices[3];
};

class MyMtVector3{
public:
	static Vector3 Add(const Vector3& v1, const Vector3& v2);
	static Vector3 Subtract(const Vector3& v1, const Vector3& v2); 
	static Vector3 Multiply(float scalar, const Vector3& v);
	static Vector3 Divide(float scalar, const Vector3& v);
	static float Dot(const Vector3& v1, const Vector3& v2);
	static float Length(const Vector3& v);
	static Vector3 Normalize(const Vector3& v);
	static Vector3 Cross(const Vector3& v1, const Vector3& v2);
	static Vector3 Project(const Vector3& v1, const Vector3& v2);
	static Vector3 Min(const Vector3& v, const Vector3& min);
	static Vector3 Max(const Vector3& v, const Vector3& max);
	static Vector3 ClampMin(const Vector3& v, const Vector3& min);
	static Vector3 ClampMax(const Vector3& v, const Vector3& max);
	static Vector3 Clamp(const Vector3& v,const Vector3& min,const Vector3& max);
	static Vector3 ClosestPoint(const Vector3& point, const Line& line);
	static Vector3 ClosestPoint(const Vector3& point, const Ray& ray);
	static Vector3 ClosestPoint(const Vector3& point, const Segment& segment);
};

