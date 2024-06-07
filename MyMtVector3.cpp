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
Vector3 MyMtVector3::Divide(float scalar, const Vector3& v)
{
	Vector3 v2;
	v2.x = v.x / scalar;
	v2.y = v.y / scalar;
	v2.z = v.z / scalar;
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

Vector3 MyMtVector3::Cross(const Vector3& v1, const Vector3& v2) {
	Vector3 ab{ v1.y * v2.z - v1.z * v2.y,v1.z * v2.x - v1.x * v2.z,v1.x * v2.y - v1.y * v2.x };
	return ab;
}

Vector3 MyMtVector3::Project(const Vector3& v1, const Vector3& v2)
{
	Vector3 v3;
	v3 = Multiply(Dot(v1,Normalize(v2)),Normalize(v2));
	return v3;
}

Vector3 MyMtVector3::Min(const Vector3& v, const Vector3& min)
{
	Vector3 v2 = v;
	v2.x = v2.x < min.x ? v2.x : min.x;
	v2.y = v2.y < min.y ? v2.y : min.y;
	v2.z = v2.z < min.z ? v2.z : min.z;
	return v2;
}

Vector3 MyMtVector3::Max(const Vector3& v, const Vector3& max)
{
	Vector3 v2 = v;
	v2.x = v2.x > max.x ? v2.x :max.x;
	v2.y = v2.y > max.y ? v2.y : max.y;
	v2.z = v2.z > max.z ? v2.z : max.z;
	return v2;
}
Vector3 MyMtVector3::ClampMin(const Vector3& v, const Vector3& min)
{
	Vector3 v2 = v;
	v2.x = v2.x < min.x ? min.x : v2.x;
	v2.y = v2.y < min.y ? min.y : v2.y;
	v2.z = v2.z < min.z ? min.z : v2.z;
	return v2;
}

Vector3 MyMtVector3::ClampMax(const Vector3& v, const Vector3& max)
{
	Vector3 v2 = v;
	v2.x = v2.x > max.x ? max.x : v2.x;
	v2.y = v2.y > max.y ? max.y : v2.y;
	v2.z = v2.z > max.z ? max.z : v2.z;
	return v2;
}
Vector3 MyMtVector3::Clamp(const Vector3& v, const Vector3& min, const Vector3& max)
{
	Vector3 v2 = v;
	v2 = ClampMin(v2, min);
	v2 = ClampMax(v2, max);
	return v2;
}

Vector3 MyMtVector3::ClosestPoint(const Vector3& point, const Line& line)
{
	Vector3 v3;
	v3 = Add(line.origin, Project(Subtract(point, line.origin), line.diff));
	return v3;
}

Vector3 MyMtVector3::ClosestPoint(const Vector3& point, const Ray& ray)
{
	Vector3 v3;
	v3 = Add(ray.origin, Project(Subtract(point, ray.origin), ray.diff));
	v3 = ClampMin(v3, ray.origin);
	return v3;
}

Vector3 MyMtVector3::ClosestPoint(const Vector3& point, const Segment& segment)
{
	Vector3 v3;
	v3 = Add(segment.origin, Project(Subtract(point, segment.origin), segment.diff));
	v3 = Clamp(v3, segment.origin, Add(segment.origin, segment.diff));
	return v3;
}
