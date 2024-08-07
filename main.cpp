#define _USE_MATH_DEFINES
#include <Novice.h>
#include <cstdint>
#include <cmath>
#include "MyMtVector3.h"
#include "MyMtMatrix.h"
#include "imgui.h"
#include <algorithm>

const char kWindowTitle[] = "LD2B_07_ミヤザキ_ヤマト";


Vector3 operator+(const Vector3& v1, const Vector3& v2) { return MyMtVector3::Add(v1, v2); }
Vector3 operator-(const Vector3& v1, const Vector3& v2) { return MyMtVector3::Subtract(v1, v2); }
Vector3 operator*(float v1, const Vector3& v2) { return MyMtVector3::Multiply(v1, v2); }
Vector3 operator*(const Vector3& v1, float v2) { return v2 * v1; }
Vector3 operator/(float v1, const Vector3& v2) { return MyMtVector3::Divide(v1, v2); }
Vector3 operator/(const Vector3& v1, float v2) { return v2 / v1; }
Vector3 operator-(const Vector3& v) { return { -v.x,-v.y,-v.z }; }
Matrix4x4 operator+(const Matrix4x4& m1, const Matrix4x4& m2) { return MyMtMatrix::Add(m1, m2); }
Matrix4x4 operator-(const Matrix4x4& m1, const Matrix4x4& m2) { return MyMtMatrix::Subtract(m1, m2); }
Matrix4x4 operator*(const Matrix4x4& m1, const Matrix4x4& m2) { return MyMtMatrix::Multiply(m1, m2); }

struct Sphere
{
	Vector3 center;
	float radius;
};
struct Plane {
	Vector3 normal;
	float distance;
};
struct AABB
{
	Vector3 min;
	Vector3 max;
};
struct OBB
{
	Vector3 center;
	Vector3 orientations[3];
	Vector3 size;
};
struct Spring {
	//アンカー。固定された端の位置
	Vector3 anchor;
	float naturalLength;//自然長
	float stiffness;//剛性。ばね係数
	float dampingCoefficient;//減衰係数
};
struct Ball
{
	Vector3 position;//位置
	Vector3 velocity;//速度
	Vector3 acceleration;//加速度
	float mass;//質量
	float radius;//半径
	unsigned int color;//色
};
struct Pendulum
{
	Vector3 anchor;//アンカーポイント固定された位置
	float length;//紐の長さ
	float angle;//現在の角度
	float angularVelocity;//角速度
	float angularAcceleration;//角加速度
};
struct ConicalPendulum
{
	Vector3 anchor;//アンカーポイント固定された位置
	float length;//紐の長さ
	float halApexAngle;//円錐の頂点の半分
	float angle;//現在の角度
	float angularVelocity;//角速度
};
struct Capsule
{
	Vector3 origin;
	Vector3 diff;
	float radius;
};

//クロス積
Vector3 Cross(const Vector3& v1, const Vector3& v2) {
	Vector3 ab{ v1.y * v2.z - v1.z * v2.y,v1.z * v2.x - v1.x * v2.z,v1.x * v2.y - v1.y * v2.x };
	return ab;
}
Vector3 Perpendicular(const Vector3& vector) {
	if (vector.x != 0.0f || vector.y != 0.0f) {
		return { -vector.y,vector.x,0.0f };
	}
	return { 0.0f,-vector.z,vector.y };
}
OBB MakeOBBRotate(const OBB& obb, const Vector3& rotate) {
	OBB obb2 = obb;
	Matrix4x4 obbRotateMatrix = MyMtMatrix::Multiply(MyMtMatrix::Multiply(MyMtMatrix::MakeRotateXMatrix(rotate.x), MyMtMatrix::MakeRotateYMatrix(rotate.y)), MyMtMatrix::MakeRotateZMatrix(rotate.z));
	obb2.orientations[0] = {
		obbRotateMatrix.m[0][0],
		obbRotateMatrix.m[0][1],
		obbRotateMatrix.m[0][2] };
	obb2.orientations[1] = {
		obbRotateMatrix.m[1][0],
		obbRotateMatrix.m[1][1],
		obbRotateMatrix.m[1][2] };
	obb2.orientations[2] = {
		obbRotateMatrix.m[2][0],
		obbRotateMatrix.m[2][1],
		obbRotateMatrix.m[2][2] };
	return obb2;
}
Vector3 Reflect(const Vector3& input, const Vector3& normal) {
	return input - (2.0f * MyMtVector3::Dot(input , normal)) * normal;
}

bool IsCollision(const Sphere& s1, const Sphere& s2) {
	float distance = MyMtVector3::Length(MyMtVector3::Subtract(s2.center, s1.center));
	return (distance <= (s1.radius + s2.radius) ? true : false);
}
bool IsCollision(const Sphere& sphere, const Plane& plane) {
	float k = MyMtVector3::Dot(plane.normal, sphere.center) - plane.distance;
	k = k < 0 ? -k : k;
	if (k <= sphere.radius) {
		return true;
	}
	return false;
}
bool IsCollision(const Line& line, const Plane& plane) {
	float dot = MyMtVector3::Dot(plane.normal, line.diff);
	if (dot == 0.0f) {
		return false;
	}
	float t = (plane.distance - MyMtVector3::Dot(line.origin, plane.normal)) / dot;
	if (t >= -1.0f && t <= 2.0f) {
		return true;
	}
	return false;
}
bool IsCollision(const Ray& ray, const Plane& plane) {
	float dot = MyMtVector3::Dot(plane.normal, ray.diff);
	if (dot == 0.0f) {
		return false;
	}
	float t = (plane.distance - MyMtVector3::Dot(ray.origin, plane.normal)) / dot;
	if (t >= 0.0f && t <= 2.0f) {
		return true;
	}
	return false;
}
bool IsCollision(const Segment& segment, const Plane& plane) {
	float dot = MyMtVector3::Dot(plane.normal, segment.diff);
	if (dot == 0.0f) {
		return false;
	}
	float t = (plane.distance - MyMtVector3::Dot(segment.origin, plane.normal)) / dot;
	if (t >= 0.0f && t <= 1.0f) {
		return true;
	}
	return false;
}
bool IsCollision(const Capsule& capsule, const Plane& plane) {
	float dot = MyMtVector3::Dot(plane.normal, capsule.diff);
	if (dot == 0.0f) {
		return false;
	}
	float t = (plane.distance - MyMtVector3::Dot(capsule.origin, plane.normal)) / dot;
	if (t >= -capsule.radius && t <= 1.0f + capsule.radius) {
		return true;
	}
	return false;
}
bool IsCollision(const Triangle& triangle, const Segment& segment) {
	// ベクトルv1,v2を求める
	Vector3 v01 = MyMtVector3::Subtract(triangle.Vertices[1], triangle.Vertices[0]);
	Vector3 v12 = MyMtVector3::Subtract(triangle.Vertices[2], triangle.Vertices[1]);
	Plane plane;
	// 法線nを算出
	plane.normal = MyMtVector3::Normalize(Cross(v01, v12));
	// 距離を求める
	plane.distance = MyMtVector3::Dot(triangle.Vertices[0], plane.normal);

	if (IsCollision(segment, plane)) {
		float dot = MyMtVector3::Dot(plane.normal, segment.diff);
		float t = (plane.distance - MyMtVector3::Dot(segment.origin, plane.normal)) / dot;
		Vector3 segmentP = MyMtVector3::Add(segment.origin, MyMtVector3::Multiply(t, segment.diff));

		Vector3 cross01 = Cross(MyMtVector3::Subtract(triangle.Vertices[1], triangle.Vertices[0]), MyMtVector3::Subtract(segmentP, triangle.Vertices[1]));
		Vector3 cross12 = Cross(MyMtVector3::Subtract(triangle.Vertices[2], triangle.Vertices[1]), MyMtVector3::Subtract(segmentP, triangle.Vertices[2]));
		Vector3 cross20 = Cross(MyMtVector3::Subtract(triangle.Vertices[0], triangle.Vertices[2]), MyMtVector3::Subtract(segmentP, triangle.Vertices[0]));

		if (MyMtVector3::Dot(cross01, plane.normal) >= 0.0f &&
			MyMtVector3::Dot(cross12, plane.normal) >= 0.0f &&
			MyMtVector3::Dot(cross20, plane.normal) >= 0.0f) {
			return true;
		}
	}
	return false;
}
bool IsCollision(const AABB& aabb1, const AABB& aabb2) {
	if (aabb1.min.x <= aabb2.max.x && aabb1.max.x >= aabb2.min.x &&
		aabb1.min.y <= aabb2.max.y && aabb1.max.y >= aabb2.min.y &&
		aabb1.min.z <= aabb2.max.z && aabb1.max.z >= aabb2.min.z) {
		return true;
	}
	return false;
}
bool IsCollision(const AABB& aabb, const Sphere& sphere) {
	Vector3 closestPoint = MyMtVector3::Clamp(sphere.center, aabb.min, aabb.max);
	float distance = MyMtVector3::Length(MyMtVector3::Subtract(closestPoint, sphere.center));
	if (distance <= sphere.radius) {
		return true;
	}
	return false;
}
bool IsCollision(const AABB& aabb, const Segment& segment) {
	Vector3 tMin = {
		(aabb.min.x - segment.origin.x) / segment.diff.x,
		(aabb.min.y - segment.origin.y) / segment.diff.y,
		(aabb.min.z - segment.origin.z) / segment.diff.z };
	Vector3 tMax = {
		(aabb.max.x - segment.origin.x) / segment.diff.x,
		(aabb.max.y - segment.origin.y) / segment.diff.y,
		(aabb.max.z - segment.origin.z) / segment.diff.z };
	Vector3 tNear = MyMtVector3::Min(tMin, tMax);
	Vector3 tFar = MyMtVector3::Max(tMin, tMax);
	float tmin = max(max(tNear.x, tNear.y), tNear.z);
	float tmax = min(min(tFar.x, tFar.y), tFar.z);
	if (tmin <= tmax) {
		if ((tmin >= 0 && tmin <= 1) || (tmax >= 0 && tmax <= 1) || tmin < 0 && tmax > 1) {
			return true;
		}
	}
	return false;
}
bool IsCollision(const AABB& aabb, const Ray& ray) {
	Vector3 tMin = {
		(aabb.min.x - ray.origin.x) / ray.diff.x,
		(aabb.min.y - ray.origin.y) / ray.diff.y,
		(aabb.min.z - ray.origin.z) / ray.diff.z };
	Vector3 tMax = {
		(aabb.max.x - ray.origin.x) / ray.diff.x,
		(aabb.max.y - ray.origin.y) / ray.diff.y,
		(aabb.max.z - ray.origin.z) / ray.diff.z };
	Vector3 tNear = MyMtVector3::Min(tMin, tMax);
	Vector3 tFar = MyMtVector3::Max(tMin, tMax);
	float tmin = max(max(tNear.x, tNear.y), tNear.z);
	float tmax = min(min(tFar.x, tFar.y), tFar.z);
	if (tmin <= tmax) {
		if ((tmin >= 0) || (tmax >= 0)) {
			return true;
		}
	}
	return false;
}
bool IsCollision(const AABB& aabb, const Line& line) {
	Vector3 tMin = {
		(aabb.min.x - line.origin.x) / line.diff.x,
		(aabb.min.y - line.origin.y) / line.diff.y,
		(aabb.min.z - line.origin.z) / line.diff.z };
	Vector3 tMax = {
		(aabb.max.x - line.origin.x) / line.diff.x,
		(aabb.max.y - line.origin.y) / line.diff.y,
		(aabb.max.z - line.origin.z) / line.diff.z };
	Vector3 tNear = MyMtVector3::Min(tMin, tMax);
	Vector3 tFar = MyMtVector3::Max(tMin, tMax);
	float tmin = max(max(tNear.x, tNear.y), tNear.z);
	float tmax = min(min(tFar.x, tFar.y), tFar.z);
	if (tmin <= tmax) {
		return true;
	}
	return false;
}
bool IsCollision(const OBB& obb, const Sphere& sphere) {
	Matrix4x4 obbWorldMatrix{
		obb.orientations[0].x,obb.orientations[1].x,obb.orientations[2].x,0,
		obb.orientations[0].y,obb.orientations[1].y,obb.orientations[2].y,0,
		obb.orientations[0].z,obb.orientations[1].z,obb.orientations[2].z,0,
		obb.center.x,obb.center.y,obb.center.z,1
	};
	Vector3 centerInObbLocalSpace = MyMtMatrix::Transform(sphere.center, MyMtMatrix::Inverse(obbWorldMatrix));
	AABB aabbOBBLocal{ .min = MyMtVector3::Multiply(-1.0,obb.size),.max = obb.size };
	Sphere sphereOBBLocal{ centerInObbLocalSpace,sphere.radius };
	if (IsCollision(aabbOBBLocal, sphereOBBLocal)) {
		return true;
	}
	return false;
}
bool IsCollision(const Segment& segment, const OBB& obb) {
	Matrix4x4 obbWorldMatrix{
		obb.orientations[0].x,obb.orientations[1].x,obb.orientations[2].x,0,
		obb.orientations[0].y,obb.orientations[1].y,obb.orientations[2].y,0,
		obb.orientations[0].z,obb.orientations[1].z,obb.orientations[2].z,0,
		obb.center.x,obb.center.y,obb.center.z,1
	};
	Vector3 localOrigin = MyMtMatrix::Transform(segment.origin, MyMtMatrix::Inverse(obbWorldMatrix));
	Vector3 localEnd = MyMtMatrix::Transform(MyMtVector3::Add(segment.origin, segment.diff), MyMtMatrix::Inverse(obbWorldMatrix));

	AABB localAABB{ .min = MyMtVector3::Multiply(-1.0,obb.size),.max = obb.size };

	Segment localSegment;

	localSegment.origin = localOrigin;
	localSegment.diff = MyMtVector3::Subtract(localEnd, localOrigin);

	if (IsCollision(localAABB, localSegment)) {
		return true;
	}
	return false;
}
bool IsCollision(const Line& line, const OBB& obb) {
	Matrix4x4 obbWorldMatrix{
		obb.orientations[0].x,obb.orientations[1].x,obb.orientations[2].x,0,
		obb.orientations[0].y,obb.orientations[1].y,obb.orientations[2].y,0,
		obb.orientations[0].z,obb.orientations[1].z,obb.orientations[2].z,0,
		obb.center.x,obb.center.y,obb.center.z,1
	};
	Vector3 localOrigin = MyMtMatrix::Transform(line.origin, MyMtMatrix::Inverse(obbWorldMatrix));
	Vector3 localEnd = MyMtMatrix::Transform(MyMtVector3::Add(line.origin, line.diff), MyMtMatrix::Inverse(obbWorldMatrix));

	AABB localAABB{ .min = MyMtVector3::Multiply(-1.0,obb.size),.max = obb.size };

	Line localLine;

	localLine.origin = localOrigin;
	localLine.diff = MyMtVector3::Subtract(localEnd, localOrigin);

	return IsCollision(localAABB, localLine);
}
// 分離軸に投影された軸成分から投影線分長を算出
float LenSegOnSeparateAxis(const Vector3& Sep, const Vector3& e1, const Vector3& e2, Vector3 e3 = { -1,-1,-1 })
{
	// 3つの内積の絶対値の和で投影線分長を計算
	float r1 = fabsf(MyMtVector3::Dot(Sep, e1));
	float r2 = fabsf(MyMtVector3::Dot(Sep, e2));
	float r3;
	if (e3.x == -1 && e3.y == -1 && e3.z == -1) {
		r3 = 0;
	}
	else {
		r3 = fabsf(MyMtVector3::Dot(Sep, e3));
	}
	return r1 + r2 + r3;
}
bool IsCollision(const OBB& obb1, const OBB& obb2) {
	// 各方向ベクトルの確保
	Vector3 NAe[3], NBe[3], Ae[3], Be[3];
	float size1[3] = { obb1.size.x,obb1.size.y,obb1.size.z };
	float size2[3] = { obb2.size.x,obb2.size.y,obb2.size.z };
	for (size_t i = 0; i < 3; i++)
	{
		NAe[i] = obb1.orientations[i], Ae[i] = MyMtVector3::Multiply(size1[i], NAe[i]);
		NBe[i] = obb2.orientations[i], Be[i] = MyMtVector3::Multiply(size2[i], NBe[i]);
	}
	Vector3 Interval = MyMtVector3::Subtract(obb1.center, obb2.center);
	float rA, rB, L;
	for (size_t i = 0; i < 3; i++)
	{
		// 分離軸 : Ae
		rA = MyMtVector3::Length(Ae[i]);
		rB = LenSegOnSeparateAxis(NAe[i], Be[0], Be[1], Be[2]);
		L = fabs(MyMtVector3::Dot(Interval, NAe[i]));
		if (L > rA + rB)return false;
	}
	for (size_t i = 0; i < 3; i++)
	{
		// 分離軸 : Be
		rA = LenSegOnSeparateAxis(NBe[i], Ae[0], Ae[1], Ae[2]);
		rB = MyMtVector3::Length(Be[i]);
		L = fabs(MyMtVector3::Dot(Interval, NBe[i]));
		if (L > rA + rB)return false;
	}

	// 分離軸 : A1B3
	Vector3 Cross1;
	Cross1 = Cross(NAe[0], NBe[0]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[1], Ae[2]);
	rB = LenSegOnSeparateAxis(Cross1, Be[1], Be[2]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A1B2
	Cross1 = Cross(NAe[0], NBe[1]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[1], Ae[2]);
	rB = LenSegOnSeparateAxis(Cross1, Be[0], Be[2]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A1B3
	Cross1 = Cross(NAe[0], NBe[2]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[1], Ae[2]);
	rB = LenSegOnSeparateAxis(Cross1, Be[0], Be[1]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A2B1
	Cross1 = Cross(NAe[1], NBe[0]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[0], Ae[2]);
	rB = LenSegOnSeparateAxis(Cross1, Be[1], Be[2]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A2B2
	Cross1 = Cross(NAe[1], NBe[1]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[0], Ae[2]);
	rB = LenSegOnSeparateAxis(Cross1, Be[0], Be[2]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A2B3
	Cross1 = Cross(NAe[1], NBe[2]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[0], Ae[2]);
	rB = LenSegOnSeparateAxis(Cross1, Be[0], Be[1]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A3B1
	Cross1 = Cross(NAe[2], NBe[0]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[0], Ae[1]);
	rB = LenSegOnSeparateAxis(Cross1, Be[1], Be[2]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A3B2
	Cross1 = Cross(NAe[2], NBe[1]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[0], Ae[1]);
	rB = LenSegOnSeparateAxis(Cross1, Be[0], Be[2]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A3B3
	Cross1 = Cross(NAe[2], NBe[2]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[0], Ae[1]);
	rB = LenSegOnSeparateAxis(Cross1, Be[0], Be[1]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	return true;
}
static const int kRowHeight = 20;
static const int KColumnWidth = 60;
void VectorScreenPrintf(int x, int y, const Vector3& vector, const char* label) {
	Novice::ScreenPrintf(x, y, "%.02f", vector.x);
	Novice::ScreenPrintf(x + KColumnWidth, y, "%.02f", vector.y);
	Novice::ScreenPrintf(x + KColumnWidth * 2, y, "%.02f", vector.z);
	Novice::ScreenPrintf(x + KColumnWidth * 3, y, "%s", label);
}
void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix, const char* label) {
	Novice::ScreenPrintf(x, y, "%s", label);
	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			Novice::ScreenPrintf(x + column * KColumnWidth, y + row * kRowHeight + kRowHeight, "%6.02f", matrix.m[row][column]);
		}
	}
}
void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	const uint32_t kSubdivision = 20;
	const float kLonEvery = float(M_PI) * 2 / kSubdivision;
	const float kLatEvery = float(M_PI) / kSubdivision;
	//
	for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex) {
		float lat = -float(M_PI) / 2.0f + kLatEvery * latIndex;

		for (uint32_t lonIndex = 0; lonIndex < kSubdivision; ++lonIndex) {
			float lon = lonIndex * kLonEvery;

			Vector3 a, b, c;
			a = MyMtVector3::Add(sphere.center, MyMtVector3::Multiply(sphere.radius, { cosf(lat) * cosf(lon),sinf(lat),cosf(lat) * sinf(lon) }));
			b = MyMtVector3::Add(sphere.center, MyMtVector3::Multiply(sphere.radius, { cosf(lat + kLatEvery) * cosf(lon),sinf(lat + kLatEvery),cosf(lat + kLatEvery) * sinf(lon) }));
			c = MyMtVector3::Add(sphere.center, MyMtVector3::Multiply(sphere.radius, { cosf(lat) * cosf(lon + kLonEvery),sinf(lat),cosf(lat) * sinf(lon + kLonEvery) }));


			Vector3 ndcVertex[3]{ MyMtMatrix::Transform(a, viewProjectionMatrix),MyMtMatrix::Transform(b, viewProjectionMatrix),MyMtMatrix::Transform(c, viewProjectionMatrix) };
			Vector3 screenVertices[3]{ MyMtMatrix::Transform(ndcVertex[0], viewportMatrix),MyMtMatrix::Transform(ndcVertex[1], viewportMatrix),MyMtMatrix::Transform(ndcVertex[2], viewportMatrix) };

			Novice::DrawLine((int)screenVertices[0].x, (int)screenVertices[0].y, (int)screenVertices[1].x, (int)screenVertices[1].y, color);
			Novice::DrawLine((int)screenVertices[0].x, (int)screenVertices[0].y, (int)screenVertices[2].x, (int)screenVertices[2].y, color);
		}
	}
}
void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
	const float kGridHalfwidth = 2.0f;
	const uint32_t kSubdivision = 10;
	const float kGridEvery = (kGridHalfwidth * 2.0f) / float(kSubdivision);
	//
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
		Vector3 ndcVertex = MyMtMatrix::Transform({ kGridEvery * xIndex - kGridEvery * kSubdivision / 2,0.0f, -kGridEvery * (kSubdivision / 2) }, viewProjectionMatrix);
		Vector3 screenVertices = MyMtMatrix::Transform(ndcVertex, viewportMatrix);
		ndcVertex = MyMtMatrix::Transform({ kGridEvery * xIndex - kGridEvery * kSubdivision / 2,0.0f, kGridEvery * kSubdivision / 2 }, viewProjectionMatrix);
		Vector3 screenVertices2 = MyMtMatrix::Transform(ndcVertex, viewportMatrix);
		if (xIndex == 5) {
			Novice::DrawLine(int(screenVertices.x), int(screenVertices.y), int(screenVertices2.x), int(screenVertices2.y), 0x000000ff);
		}
		else {
			Novice::DrawLine(int(screenVertices.x), int(screenVertices.y), int(screenVertices2.x), int(screenVertices2.y), 0xaaaaaaff);
		}
	}
	//
	for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
		Vector3 ndcVertex = MyMtMatrix::Transform({ -kGridEvery * (kSubdivision / 2),0.0f,kGridEvery * zIndex - kGridEvery * kSubdivision / 2 }, viewProjectionMatrix);
		Vector3 screenVertices = MyMtMatrix::Transform(ndcVertex, viewportMatrix);
		ndcVertex = MyMtMatrix::Transform({ kGridEvery * kSubdivision / 2,0.0f,kGridEvery * zIndex - kGridEvery * kSubdivision / 2 }, viewProjectionMatrix);
		Vector3 screenVertices2 = MyMtMatrix::Transform(ndcVertex, viewportMatrix);
		if (zIndex == 5) {
			Novice::DrawLine(int(screenVertices.x), int(screenVertices.y), int(screenVertices2.x), int(screenVertices2.y), 0x000000ff);
		}
		else {
			Novice::DrawLine(int(screenVertices.x), int(screenVertices.y), int(screenVertices2.x), int(screenVertices2.y), 0xaaaaaaff);
		}

	}
}
void DrawPlane(const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Vector3 center = MyMtVector3::Multiply(plane.distance, plane.normal);
	Vector3 perpendiculars[4];
	perpendiculars[0] = MyMtVector3::Normalize(Perpendicular(plane.normal));
	perpendiculars[1] = { -perpendiculars[0].x,-perpendiculars[0].y,-perpendiculars[0].z };
	perpendiculars[2] = Cross(plane.normal, perpendiculars[0]);
	perpendiculars[3] = { -perpendiculars[2].x,-perpendiculars[2].y,-perpendiculars[2].z };
	//
	Vector3 points[4];
	for (int32_t index = 0; index < 4; ++index) {
		Vector3 extend = MyMtVector3::Multiply(2.0f, perpendiculars[index]);
		Vector3 point = MyMtVector3::Add(center, extend);
		points[index] = MyMtMatrix::Transform(MyMtMatrix::Transform(point, viewProjectionMatrix), viewportMatrix);
	}
	Novice::DrawLine(int(points[0].x), int(points[0].y), int(points[2].x), int(points[2].y), color);
	Novice::DrawLine(int(points[0].x), int(points[0].y), int(points[3].x), int(points[3].y), color);
	Novice::DrawLine(int(points[1].x), int(points[1].y), int(points[2].x), int(points[2].y), color);
	Novice::DrawLine(int(points[1].x), int(points[1].y), int(points[3].x), int(points[3].y), color);

}
void DrawTriangle(const Triangle& triangle, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Vector3 points[3];
	for (int32_t index = 0; index < 3; ++index) {

		points[index] = MyMtMatrix::Transform(MyMtMatrix::Transform(triangle.Vertices[index], viewProjectionMatrix), viewportMatrix);
	}
	Novice::DrawTriangle(int(points[0].x), int(points[0].y), int(points[1].x), int(points[1].y), int(points[2].x), int(points[2].y), color, kFillModeWireFrame);
}
void DrawAABB(const AABB& aabb, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Vector3 perpendiculars[8]{};
	perpendiculars[0] = aabb.min;
	perpendiculars[1] = { aabb.max.x,aabb.min.y,aabb.min.z };
	perpendiculars[2] = { aabb.min.x,aabb.max.y,aabb.min.z };
	perpendiculars[3] = { aabb.max.x,aabb.max.y,aabb.min.z };

	perpendiculars[4] = { aabb.min.x,aabb.min.y,aabb.max.z };
	perpendiculars[5] = { aabb.max.x,aabb.min.y,aabb.max.z };
	perpendiculars[6] = { aabb.min.x,aabb.max.y,aabb.max.z };
	perpendiculars[7] = aabb.max;

	Vector3 points[8];
	for (int32_t index = 0; index < 8; ++index) {
		points[index] = MyMtMatrix::Transform(MyMtMatrix::Transform(perpendiculars[index], viewProjectionMatrix), viewportMatrix);
	}
	Novice::DrawLine((int)points[0].x, (int)points[0].y, (int)points[1].x, (int)points[1].y, color);
	Novice::DrawLine((int)points[0].x, (int)points[0].y, (int)points[2].x, (int)points[2].y, color);
	Novice::DrawLine((int)points[3].x, (int)points[3].y, (int)points[2].x, (int)points[2].y, color);
	Novice::DrawLine((int)points[3].x, (int)points[3].y, (int)points[1].x, (int)points[1].y, color);

	Novice::DrawLine((int)points[4].x, (int)points[4].y, (int)points[5].x, (int)points[5].y, color);
	Novice::DrawLine((int)points[4].x, (int)points[4].y, (int)points[6].x, (int)points[6].y, color);
	Novice::DrawLine((int)points[7].x, (int)points[7].y, (int)points[6].x, (int)points[6].y, color);
	Novice::DrawLine((int)points[7].x, (int)points[7].y, (int)points[5].x, (int)points[5].y, color);

	Novice::DrawLine((int)points[0].x, (int)points[0].y, (int)points[4].x, (int)points[4].y, color);
	Novice::DrawLine((int)points[1].x, (int)points[1].y, (int)points[5].x, (int)points[5].y, color);
	Novice::DrawLine((int)points[2].x, (int)points[2].y, (int)points[6].x, (int)points[6].y, color);
	Novice::DrawLine((int)points[3].x, (int)points[3].y, (int)points[7].x, (int)points[7].y, color);
}
void DrawOBB(const OBB& obb, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	Matrix4x4 obbWorldMatrix{
		obb.orientations[0].x,obb.orientations[1].x,obb.orientations[2].x,0,
		obb.orientations[0].y,obb.orientations[1].y,obb.orientations[2].y,0,
		obb.orientations[0].z,obb.orientations[1].z,obb.orientations[2].z,0,
		obb.center.x,obb.center.y,obb.center.z,1
	};
	Vector3 perpendiculars[8]{};
	perpendiculars[0] = { -obb.size.x,-obb.size.y,-obb.size.z };
	perpendiculars[1] = { obb.size.x,-obb.size.y,-obb.size.z };
	perpendiculars[2] = { -obb.size.x,obb.size.y,-obb.size.z };
	perpendiculars[3] = { obb.size.x,obb.size.y,-obb.size.z };

	perpendiculars[4] = { -obb.size.x,-obb.size.y,obb.size.z };
	perpendiculars[5] = { obb.size.x,-obb.size.y,obb.size.z };
	perpendiculars[6] = { -obb.size.x,obb.size.y,obb.size.z };
	perpendiculars[7] = { obb.size.x,obb.size.y,obb.size.z };

	Vector3 points[8];
	for (int32_t index = 0; index < 8; ++index) {
		points[index] = MyMtMatrix::Transform(MyMtMatrix::Transform(perpendiculars[index], MyMtMatrix::Multiply(obbWorldMatrix, viewProjectionMatrix)), viewportMatrix);
	}
	Novice::DrawLine((int)points[0].x, (int)points[0].y, (int)points[1].x, (int)points[1].y, color);
	Novice::DrawLine((int)points[0].x, (int)points[0].y, (int)points[2].x, (int)points[2].y, color);
	Novice::DrawLine((int)points[3].x, (int)points[3].y, (int)points[2].x, (int)points[2].y, color);
	Novice::DrawLine((int)points[3].x, (int)points[3].y, (int)points[1].x, (int)points[1].y, color);

	Novice::DrawLine((int)points[4].x, (int)points[4].y, (int)points[5].x, (int)points[5].y, color);
	Novice::DrawLine((int)points[4].x, (int)points[4].y, (int)points[6].x, (int)points[6].y, color);
	Novice::DrawLine((int)points[7].x, (int)points[7].y, (int)points[6].x, (int)points[6].y, color);
	Novice::DrawLine((int)points[7].x, (int)points[7].y, (int)points[5].x, (int)points[5].y, color);

	Novice::DrawLine((int)points[0].x, (int)points[0].y, (int)points[4].x, (int)points[4].y, color);
	Novice::DrawLine((int)points[1].x, (int)points[1].y, (int)points[5].x, (int)points[5].y, color);
	Novice::DrawLine((int)points[2].x, (int)points[2].y, (int)points[6].x, (int)points[6].y, color);
	Novice::DrawLine((int)points[3].x, (int)points[3].y, (int)points[7].x, (int)points[7].y, color);
}
void DrawBezier(const Vector3& controlPoint0, const Vector3& controlPoint1, const Vector3& controlPoint2, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	const size_t segmentCount = 100;
	for (size_t i = 0; i < segmentCount; i++) {
		float t = 1.0f / segmentCount * i;
		float t2 = t + 1.0f / segmentCount;
		Vector3 p2 = MyMtVector3::Lerp(controlPoint0, controlPoint1, t),
			p3 = MyMtVector3::Lerp(controlPoint1, controlPoint2, t);
		Vector3 pos[2];
		pos[0] = MyMtVector3::CatmullRomInterpolation(controlPoint0, p2, p3, controlPoint2, t);
		p2 = MyMtVector3::Lerp(controlPoint0, controlPoint1, t2);
		p3 = MyMtVector3::Lerp(controlPoint1, controlPoint2, t2);
		pos[1] = MyMtVector3::CatmullRomInterpolation(controlPoint0, p2, p3, controlPoint2, t2);
		Vector3 points[2];
		for (int32_t index = 0; index < 2; ++index) {
			points[index] = MyMtMatrix::Transform(MyMtMatrix::Transform(pos[index], viewProjectionMatrix), viewportMatrix);
		}
		Novice::DrawLine((int)points[0].x, (int)points[0].y, (int)points[1].x, (int)points[1].y, color);
	}
}
void DrawCotmullRom(const Vector3& controlPoint0, const Vector3& controlPoint1, const Vector3& controlPoint2, const Vector3& controlPoint3, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color) {
	const size_t segmentCount = 100;
	for (size_t j = 0; j < 3; j++) {
		Vector3 controlPoint[4] = { controlPoint0, controlPoint1, controlPoint2, controlPoint3 };
		if (j == 0) {
			controlPoint[1] = controlPoint0;
			controlPoint[2] = controlPoint1;
			controlPoint[3] = controlPoint2;
		}
		if (j == 2) {
			controlPoint[0] = controlPoint1;
			controlPoint[1] = controlPoint2;
			controlPoint[2] = controlPoint3;
		}
		for (size_t i = 0; i < segmentCount; i++) {
			float t = 1.0f / segmentCount * i;
			float t2 = t + 1.0f / segmentCount;
			Vector3 pos[2];
			pos[0] = MyMtVector3::CatmullRomInterpolation(controlPoint[0], controlPoint[1], controlPoint[2], controlPoint[3], t);
			pos[1] = MyMtVector3::CatmullRomInterpolation(controlPoint[0], controlPoint[1], controlPoint[2], controlPoint[3], t2);
			Vector3 points[2];
			for (int32_t index = 0; index < 2; ++index) {
				points[index] = MyMtMatrix::Transform(MyMtMatrix::Transform(pos[index], viewProjectionMatrix), viewportMatrix);
			}
			Novice::DrawLine((int)points[0].x, (int)points[0].y, (int)points[1].x, (int)points[1].y, color);
		}
	}
}
// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };
	//
	int kWindowWidth = 1280;
	int kWindowHeight = 720;

	Vector3 cameraTranslate{ 0.0f,1.9f,-6.49f };
	Vector3 cameraRotate{ 0.26f,0.0f,0.0f };

	Matrix4x4 worldMatrix{};
	Matrix4x4 cameraMatrix{};

	Matrix4x4 viewMatrix{};
	Matrix4x4 projectionMatrix{};
	Matrix4x4 ViewProjectionMatrix{};
	Matrix4x4 viewportMatrix{};

	Matrix4x4 screenMatrix{
		1,0,0,0,
		0,-1,0,0,
		0,0,1,0,
		0,0,0,1
	};

	int mouseX = 0;
	int mouseY = 0;
	Vector3 mouse{};
	Vector3 preMouse{};
	Vector3 wheelVelocity{};

	float kCameraSpeed = 0.03f;
	Vector3 cameraVelocity{};

	/*Sphere sphere1{
		{0.0f,0.0f,0.0f},
		0.05f
	};*/
	//uint32_t sphereColor1 = 0xffffffff;
	/*Segment segment{
		.origin{-0.8f,-0.3f,0.0f},
		.diff{0.5f,0.5f,0.5f},
	};
	Segment screneSegment{};
	uint32_t segmentColor = 0xffffffff;*/
	/*Plane plane{
		{0.0f,1.0f,0.0f},
		1.0f
	};*/
	/*Triangle triangle;
	triangle.Vertices[0] = { -1.0f, 0, 0 };
	triangle.Vertices[1] = { 0, 1.0f, 0 };
	triangle.Vertices[2] = { 1.0f, 0, 0 };
	unsigned int triangleColor = 0xffffffff;*/
	/*AABB aabb1{
		.min{-0.5f,-0.5f,-0.5f},
		.max{0.5f,0.5f,0.5f},
	};
	unsigned int aabb1Color = 0xffffffff;*/
	/*Vector3 obbRotate{ 0.0f,0.0f,0.0f };
	Matrix4x4 obbRotateMatrix{};
	OBB obb{
		.center{0.0f,0.0f,0.0f},
		.orientations = {{1.0f,0.0f,0.0f},
						 {0.0f,1.0f,0.0f},
						 {0.0f,0.0f,1.0f}},
		.size{0.5f,0.5f,0.5f}
	};
	uint32_t obbColor = 0xffffffff;
	Vector3 obbRotate2{ -0.5f,-2.49f,0.15f };
	OBB obb2{
		.center{0.9f,0.66f,0.78f},
		.orientations = {{1.0f,0.0f,0.0f},
						 {0.0f,1.0f,0.0f},
						 {0.0f,0.0f,1.0f}},
		.size{0.5f,0.37f,0.5f}
	};*/
	/*Vector3 controlPoints[4] = {
		{-0.8f,0.58f,1.0f},
		{1.76f,1.0f,-0.3f},
		{0.94f,-0.7f,2.3f},
		{-0.53f,-0.26f,-0.15f,}
	};*/
	/*Sphere sphere[3]{};
	Vector3 translates[3] = {
		{0.2f,1.0f,0.0f},
		{0.4f,0.0f,0.0f},
		{0.3f,0.0f,0.0f},
	};
	Vector3 rotates[3] = {
		{0.0f,0.0f,-6.8f},
		{0.0f,0.0f,-1.4f},
		{0.0f,0.0f,0.0f},
	};
	Vector3 scales[3] = {
		{1.0f,1.0f,1.0f},
		{1.0f,1.0f,1.0f},
		{1.0f,1.0f,1.0f},
	};
	Matrix4x4 sphereWorldMatrix[3];
	Matrix4x4 sphereWorldViewProjectionMatrix[3]{};*/

	/*Spring spring{};
	spring.anchor = { 0.0f,1.0f,0.0f };
	spring.naturalLength = 0.7f;
	spring.stiffness = 100.f;
	spring.dampingCoefficient = 2.0f;*/

	Ball ball{};
	ball.position = { 0.8f,1.2f,0.3f };
	ball.acceleration = { 0.0f,-9.8f,0.0f };
	ball.mass = 2.0f;
	ball.radius = 0.05f;
	ball.color = WHITE;
	
	Plane plane{};
	plane.normal = MyMtVector3::Normalize({ -0.2f,0.9f,-0.3f });
	plane.distance = 0.0f;

	Capsule capsule{};
	capsule.radius = ball.radius;

	float deltaTime = 1.0f / 60.0f;

	/*const Vector3 kGravity{ 0.0f,-9.8f,0.0f };
	Vector3 diff;
	float length;*/
	
	//float angularVelocity = 3.14f;
	//float angle = 0.0f;
	//float radius = 0.8f;
	//Vector3 position{ 0.0f,1.0f,0.0f };
	//Vector3 center{};
	//
	//Vector3 acceleration{};
	//Vector3 velocity{};
	
	/*Pendulum pendulum;
	pendulum.anchor = { 0.0f,1.0f,0.0f };
	pendulum.length = 0.8f;
	pendulum.angle = 0.7f;
	pendulum.angularVelocity = 0.0f;
	pendulum.angularAcceleration = 0.0f;*/

	/*ConicalPendulum conicalPendulum;
	conicalPendulum.anchor = { 0.0f,1.0f,0.0f };
	conicalPendulum.length = 0.8f;
	conicalPendulum.halApexAngle = 0.7f;
	conicalPendulum.angle = 0.0f;
	conicalPendulum.angularVelocity = 0.0f;*/

	bool isStart = false;
	ImVec2 button = {100,20};
	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///
		//
		Novice::GetMousePosition(&mouseX, &mouseY);
		mouse = { float(mouseX),float(-mouseY),0 };
		wheelVelocity = MyMtMatrix::Transform({ 0,0,float(Novice::GetWheel()) / 400.0f }, MyMtMatrix::MakeAffineMatrix({ 1.0f,1.0f,1.0f }, cameraRotate, { 0,0,0 }));
		cameraTranslate = MyMtVector3::Add(cameraTranslate, wheelVelocity);

		cameraVelocity = { 0,0,0 };
		if (keys[DIK_A]) {
			cameraVelocity.x -= kCameraSpeed;
		}
		if (keys[DIK_D]) {
			cameraVelocity.x += kCameraSpeed;
		}
		if (keys[DIK_W]) {
			cameraVelocity.y += kCameraSpeed;
		}
		if (keys[DIK_S]) {
			cameraVelocity.y -= kCameraSpeed;
		}
		cameraVelocity = MyMtMatrix::Transform(cameraVelocity, MyMtMatrix::MakeAffineMatrix({ 1.0f,1.0f,1.0f }, cameraRotate, { 0,0,0 }));
		cameraTranslate = MyMtVector3::Add(cameraTranslate, cameraVelocity);

		if (Novice::IsPressMouse(0) && keys[DIK_LSHIFT]) {
			cameraRotate = MyMtVector3::Add(cameraRotate, MyMtVector3::Divide(1000.0f, { MyMtVector3::Subtract(preMouse, mouse).y,-MyMtVector3::Subtract(preMouse, mouse).x,0 }));
			cameraTranslate = MyMtVector3::Add(cameraTranslate, MyMtMatrix::Transform(MyMtVector3::Divide(120.0f, MyMtVector3::Subtract(preMouse, mouse)), MyMtMatrix::MakeAffineMatrix({ 1.0f,1.0f,1.0f }, cameraRotate, { 0,0,0 })));
			//cameraRotate = MyMtVector3::Add(cameraRotate, MyMtVector3::Divide(1000.0f, { MyMtVector3::Subtract(preMouse, mouse).y,-MyMtVector3::Subtract(preMouse, mouse).x,0}));
		}
		if (keys[DIK_SPACE]) {
			cameraRotate = MyMtVector3::Add(cameraRotate, MyMtVector3::Divide(1000.0f, { MyMtVector3::Subtract(preMouse, mouse).y,-MyMtVector3::Subtract(preMouse, mouse).x,0 }));
		}
		cameraMatrix = MyMtMatrix::MakeAffineMatrix({ 1.0f,1.0f,1.0f }, cameraRotate, cameraTranslate);
		viewMatrix = MyMtMatrix::Inverse(cameraMatrix);
		projectionMatrix = MyMtMatrix::MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		ViewProjectionMatrix = MyMtMatrix::Multiply(viewMatrix, projectionMatrix);
		viewportMatrix = MyMtMatrix::MakeViewportMatrix(0, 0, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);
		
		if (isStart) {
			/*diff = ball.position - spring.anchor;
			length = MyMtVector3::Length(diff);
			if (length != 0.0f) {
				Vector3 direction = MyMtVector3::Normalize(diff);
				Vector3 restPosition = spring.anchor + direction * spring.naturalLength;
				Vector3 displacement = length * (ball.position - restPosition);
				Vector3 restoringForce = -spring.stiffness * displacement;
				Vector3 force = restoringForce;
				Vector3 dampingForce = -spring.dampingCoefficient * ball.velocity;
				force = restoringForce + dampingForce;
				ball.cceleration = force / ball.mass;
			}
			ball.velocity = ball.velocity + ball.cceleration * deltaTime + kGravity * deltaTime;
			ball.position = ball.position + ball.velocity * deltaTime;*/
			/*float womega = angularVelocity * deltaTime;
			angle += womega;
			velocity = Vector3(-radius * womega * std::sinf(angle), radius * womega * std::cosf(angle), 0);
			acceleration = -powf(womega, 2) * (position - center);
			position = position + velocity + acceleration;*/

			/*conicalPendulum.angularVelocity = std::sqrtf(9.8f / (conicalPendulum.length * std::cosf(conicalPendulum.halApexAngle)));
			conicalPendulum.angle += conicalPendulum.angularVelocity * deltaTime;*/
			ball.velocity = ball.velocity + ball.acceleration * deltaTime;
			capsule.origin = ball.position;
			capsule.diff = ball.velocity * deltaTime;
			ball.position = ball.position + ball.velocity * deltaTime;
		}

		/*if (IsCollision(Sphere{ ball.position,ball.radius }, plane)) {
			Vector3 reflected = Reflect(ball.velocity, plane.normal);
			Vector3 projectToNormal = MyMtVector3::Project(reflected, plane.normal);
			Vector3 movingDirection = reflected - projectToNormal;
			ball.velocity = projectToNormal * 0.6f + movingDirection;
		}*/
		if (IsCollision(capsule, plane)) {
			ball.position = capsule.origin;
			Vector3 reflected = Reflect(ball.velocity, plane.normal);
			Vector3 projectToNormal = MyMtVector3::Project(reflected, plane.normal);
			Vector3 movingDirection = reflected - projectToNormal;
			ball.velocity = projectToNormal * 0.6f + movingDirection;
		}

		/*sphere1.center = ball.position;
		sphere1.radius = ball.radius;*/
		
		//sphere1.center = position + center;
		/*float radius = std::sinf(conicalPendulum.halApexAngle) * conicalPendulum.length;
		float height = std::cosf(conicalPendulum.halApexAngle) * conicalPendulum.length;
		position.x = conicalPendulum.anchor.x + std::cosf(conicalPendulum.angle) * radius;
		position.y = conicalPendulum.anchor.y - height;
		position.z = conicalPendulum.anchor.z - std::sinf(conicalPendulum.angle) * radius;
		sphere1.center = position;*/

		//for (size_t i = 0; i < 3; i++) {
		//	sphere[i] = { {0.0f,0.0f,0.0f},0.1f };
		//	if (i == 0) {
		//		sphereWorldMatrix[i] = MyMtMatrix::MakeAffineMatrix(scales[i], rotates[i], translates[i]);
		//	}
		//	else {
		//		sphereWorldMatrix[i] = MyMtMatrix::Multiply(MyMtMatrix::MakeAffineMatrix(scales[i], rotates[i], translates[i]), sphereWorldMatrix[i - 1]);
		//	}
		//	sphereWorldViewProjectionMatrix[i] = MyMtMatrix::Multiply(sphereWorldMatrix[i], ViewProjectionMatrix);
		//}

		//obb = MakeOBBRotate(obb, obbRotate);
		//obb2 = MakeOBBRotate(obb2, obbRotate2);
		/*sphereColor1 = IsCollision(sphere1, plane) == true ? 0xff0000ff : 0xffffffff;
		segmentColor = IsCollision(segment, plane) == true ? 0xff0000ff : 0xffffffff;*/
		/*segmentColor = IsCollision(triangle, segment) == true ? 0xff0000ff : 0xffffffff;*/
		//aabb1Color = IsCollision(aabb1,sphere1) == true ? 0xff0000ff : 0xffffffff;
		//aabb1Color = IsCollision(aabb1, segment) == true ? 0xff0000ff : 0xffffffff;
		//obbColor = IsCollision(obb,sphere1) == true ? 0xff0000ff : 0xffffffff;
		/*obbColor = IsCollision(obb,obb2) == true ? 0xff0000ff : 0xffffffff;*/

		/*screneSegment.origin = MyMtMatrix::Transform(MyMtMatrix::Transform(segment.origin,ViewProjectionMatrix), viewportMatrix);
		screneSegment.diff = MyMtMatrix::Transform(MyMtMatrix::Transform(MyMtVector3::Add(segment.origin,segment.diff),ViewProjectionMatrix), viewportMatrix);*/

		ImGui::Begin("Window");
		if (ImGui::Button("Start", button)) {
			isStart = true;
		}
		ImGui::DragFloat3("CameraTranslate", &cameraTranslate.x, 0.01f);
		ImGui::DragFloat3("CameraRotate", &cameraRotate.x, 0.01f);

		/*ImGui::DragFloat3("SphereCenter1", &sphere1.center.x, 0.01f);
		ImGui::DragFloat("SphereRadius1", &sphere1.radius, 0.01f);*/

		ImGui::DragFloat3("Plane.Normal", &plane.normal.x, 0.01f);
		ImGui::DragFloat("Plane.Distance", &plane.distance, 0.01f);
		plane.normal = MyMtVector3::Normalize(plane.normal);

		/*ImGui::DragFloat3("Triangle.Vertices0", &triangle.Vertices[0].x, 0.01f);
		ImGui::DragFloat3("Triangle.Vertices1", &triangle.Vertices[1].x, 0.01f);
		ImGui::DragFloat3("Triangle.Vertices2", &triangle.Vertices[2].x, 0.01f);*/

		/*ImGui::DragFloat3("Segment.Origin", &segment.origin.x, 0.01f);
		ImGui::DragFloat3("Segment.Diff", &segment.diff.x, 0.01f);*/

		/*ImGui::DragFloat3("AABB1.MIN", &aabb1.min.x, 0.01f);
		ImGui::DragFloat3("AABB1.MAX", &aabb1.max.x, 0.01f);*/
		/*aabb1.min.x = (std::min)(aabb1.min.x, aabb1.max.x);
		aabb1.max.x = (std::max)(aabb1.min.x, aabb1.max.x);
		aabb1.min.y = (std::min)(aabb1.min.y, aabb1.max.y);
		aabb1.max.y = (std::max)(aabb1.min.y, aabb1.max.y);
		aabb1.min.z = (std::min)(aabb1.min.z, aabb1.max.z);
		aabb1.max.z = (std::max)(aabb1.min.z, aabb1.max.z);*/

		/*ImGui::DragFloat("OBB.Rotate.x", &obbRotate.x, 0.01f);
		ImGui::DragFloat("OBB.Rotate.y", &obbRotate.y, 0.01f);
		ImGui::DragFloat("OBB.Rotate.z", &obbRotate.z, 0.01f);

		ImGui::DragFloat3("OBB.Center", &obb.center.x, 0.01f);
		ImGui::DragFloat3("OBB.Orientations[0]", &obb.orientations[0].x, 0.01f);
		ImGui::DragFloat3("OBB.Orientations[1]", &obb.orientations[1].x, 0.01f);
		ImGui::DragFloat3("OBB.Orientations[2]", &obb.orientations[2].x, 0.01f);
		ImGui::DragFloat3("OBB.Size", &obb.size.x, 0.01f);

		ImGui::DragFloat("OBB2.Rotate.x", &obbRotate2.x, 0.01f);
		ImGui::DragFloat("OBB2.Rotate.y", &obbRotate2.y, 0.01f);
		ImGui::DragFloat("OBB2.Rotate.z", &obbRotate2.z, 0.01f);

		ImGui::DragFloat3("OBB2.Center", &obb2.center.x, 0.01f);
		ImGui::DragFloat3("OBB2.Orientations[0]", &obb2.orientations[0].x, 0.01f);
		ImGui::DragFloat3("OBB2.Orientations[1]", &obb2.orientations[1].x, 0.01f);
		ImGui::DragFloat3("OBB2.Orientations[2]", &obb2.orientations[2].x, 0.01f);
		ImGui::DragFloat3("OBB2.Size", &obb2.size.x, 0.01f);*/
		/*ImGui::DragFloat3("controlPoints[0]", &controlPoints[0].x, 0.01f);
		ImGui::DragFloat3("controlPoints[1]", &controlPoints[1].x, 0.01f);
		ImGui::DragFloat3("controlPoints[2]", &controlPoints[2].x, 0.01f);*/
		/*ImGui::DragFloat3("translates[0]", &translates[0].x, 0.01f);
		ImGui::DragFloat3("translates[1]", &translates[1].x, 0.01f);
		ImGui::DragFloat3("translates[2]", &translates[2].x, 0.01f);
		ImGui::DragFloat3("rotates[0]", &rotates[0].x, 0.01f);
		ImGui::DragFloat3("rotates[1]", &rotates[1].x, 0.01f);
		ImGui::DragFloat3("rotates[2]", &rotates[2].x, 0.01f);
		ImGui::DragFloat3("scales[0]", &scales[0].x, 0.01f);
		ImGui::DragFloat3("scales[1]", &scales[1].x, 0.01f);
		ImGui::DragFloat3("scales[2]", &scales[2].x, 0.01f);*/
		/*ImGui::DragFloat3("spring.anchor", &spring.anchor.x, 0.01f);
		ImGui::DragFloat("spring.naturalLength", &spring.naturalLength, 0.01f);
		ImGui::DragFloat("spring.stiffness", &spring.stiffness, 0.01f);*/
		ImGui::DragFloat3("ball.position", &ball.position.x, 0.01f);
		ImGui::DragFloat3("ball.velocity", &ball.velocity.x, 0.01f);
		ImGui::DragFloat3("ball.acceleration", &ball.acceleration.x, 0.01f);
		ImGui::DragFloat("ball.mass", &ball.mass, 0.01f);
		
		//ImGui::DragFloat3("position", &position.x, 0.01f);
		/*ImGui::DragFloat3("velocity", &velocity.x, 0.01f);
		ImGui::DragFloat3("acceleration", &acceleration.x, 0.01f);*/
		
		ImGui::End();
		/*preMouse = mouse;*/
		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		DrawGrid(ViewProjectionMatrix, viewportMatrix);
		/*for (size_t i = 0; i < 3; i++) {
			if (i != 0) {
				Vector3 screneSegment = MyMtMatrix::Transform(MyMtMatrix::Transform(sphere[i - 1].center, sphereWorldViewProjectionMatrix[i - 1]), viewportMatrix);
				Vector3 screneSegment2 = MyMtMatrix::Transform(MyMtMatrix::Transform(sphere[i].center, sphereWorldViewProjectionMatrix[i]), viewportMatrix);
				Novice::DrawLine((int)screneSegment.x, (int)screneSegment.y, (int)screneSegment2.x, (int)screneSegment2.y, 0xffffffff);
			}
			DrawSphere(sphere[i], sphereWorldViewProjectionMatrix[i], viewportMatrix, 0x000000ff);
		}*/
		//DrawSphere(sphere1, ViewProjectionMatrix, viewportMatrix, 0xffffffff);
		/*Vector3 screneSegment = MyMtMatrix::Transform(MyMtMatrix::Transform(spring.anchor, ViewProjectionMatrix), viewportMatrix);
		Vector3 screneSegment2 = MyMtMatrix::Transform(MyMtMatrix::Transform(sphere1.center, ViewProjectionMatrix), viewportMatrix);*/
		//Novice::DrawLine((int)screneSegment.x, (int)screneSegment.y, (int)screneSegment2.x, (int)screneSegment2.y, 0xffffffff);
		/*Vector3 screneSegment = MyMtMatrix::Transform(MyMtMatrix::Transform(conicalPendulum.anchor, ViewProjectionMatrix), viewportMatrix);
		Vector3 screneSegment2 = MyMtMatrix::Transform(MyMtMatrix::Transform(sphere1.center, ViewProjectionMatrix), viewportMatrix);*/
		//Novice::DrawLine((int)screneSegment.x, (int)screneSegment.y, (int)screneSegment2.x, (int)screneSegment2.y, 0xffffffff);
		DrawSphere(Sphere(ball.position,ball.radius), ViewProjectionMatrix, viewportMatrix, ball.color);
		//Novice::DrawLine((int)screneSegment.origin.x, (int)screneSegment.origin.y, (int)screneSegment.diff.x, (int)screneSegment.diff.y, segmentColor);
		DrawPlane(plane, ViewProjectionMatrix, viewportMatrix,0xffffffff);
		//DrawTriangle(triangle, worldViewProjectionMatrix, viewportMatrix, triangleColor);
		//DrawAABB(aabb1, worldViewProjectionMatrix, viewportMatrix, aabb1Color);
		/*DrawOBB(obb, ViewProjectionMatrix, viewportMatrix, obbColor);
		DrawOBB(obb2, ViewProjectionMatrix, viewportMatrix, 0xffffffff);*/
		//DrawCotmullRom(controlPoints[0], controlPoints[1], controlPoints[2],controlPoints[3], ViewProjectionMatrix, viewportMatrix, 0x0000ffff);
		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
