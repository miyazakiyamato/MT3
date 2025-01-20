#define _USE_MATH_DEFINES
#include <Novice.h>
#include <cstdint>
#include <cmath>
#include "MyMtVector3.h"
#include "MyMtMatrix.h"
#include "Quaternion.h"
#include "imgui.h"
#include <algorithm>

const char kWindowTitle[] = "LD2B_07_ミヤザキ_ヤマト";

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

float Probability(uint32_t sizeS, uint32_t sizeA) {
	return (float)sizeA / (float)sizeS;
}

uint32_t Fractional(uint32_t n) {
	if (n > 0) {
		return n * Fractional(n - 1);
	}
	return 1;
}

uint32_t Permutation(uint32_t n, uint32_t k) {
	return Fractional(n) / Fractional(n - k);
}

float Combination(uint32_t n, uint32_t k) {
	return float(Permutation(n,k) / Fractional(k));
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
	plane.normal = MyMtVector3::Normalize(MyMtVector3::Cross(v01, v12));
	// 距離を求める
	plane.distance = MyMtVector3::Dot(triangle.Vertices[0], plane.normal);

	if (IsCollision(segment, plane)) {
		float dot = MyMtVector3::Dot(plane.normal, segment.diff);
		float t = (plane.distance - MyMtVector3::Dot(segment.origin, plane.normal)) / dot;
		Vector3 segmentP = MyMtVector3::Add(segment.origin, MyMtVector3::Multiply(t, segment.diff));

		Vector3 cross01 = MyMtVector3::Cross(MyMtVector3::Subtract(triangle.Vertices[1], triangle.Vertices[0]), MyMtVector3::Subtract(segmentP, triangle.Vertices[1]));
		Vector3 cross12 = MyMtVector3::Cross(MyMtVector3::Subtract(triangle.Vertices[2], triangle.Vertices[1]), MyMtVector3::Subtract(segmentP, triangle.Vertices[2]));
		Vector3 cross20 = MyMtVector3::Cross(MyMtVector3::Subtract(triangle.Vertices[0], triangle.Vertices[2]), MyMtVector3::Subtract(segmentP, triangle.Vertices[0]));

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
	Cross1 = MyMtVector3::Cross(NAe[0], NBe[0]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[1], Ae[2]);
	rB = LenSegOnSeparateAxis(Cross1, Be[1], Be[2]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A1B2
	Cross1 = MyMtVector3::Cross(NAe[0], NBe[1]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[1], Ae[2]);
	rB = LenSegOnSeparateAxis(Cross1, Be[0], Be[2]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A1B3
	Cross1 = MyMtVector3::Cross(NAe[0], NBe[2]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[1], Ae[2]);
	rB = LenSegOnSeparateAxis(Cross1, Be[0], Be[1]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A2B1
	Cross1 = MyMtVector3::Cross(NAe[1], NBe[0]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[0], Ae[2]);
	rB = LenSegOnSeparateAxis(Cross1, Be[1], Be[2]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A2B2
	Cross1 = MyMtVector3::Cross(NAe[1], NBe[1]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[0], Ae[2]);
	rB = LenSegOnSeparateAxis(Cross1, Be[0], Be[2]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A2B3
	Cross1 = MyMtVector3::Cross(NAe[1], NBe[2]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[0], Ae[2]);
	rB = LenSegOnSeparateAxis(Cross1, Be[0], Be[1]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A3B1
	Cross1 = MyMtVector3::Cross(NAe[2], NBe[0]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[0], Ae[1]);
	rB = LenSegOnSeparateAxis(Cross1, Be[1], Be[2]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A3B2
	Cross1 = MyMtVector3::Cross(NAe[2], NBe[1]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[0], Ae[1]);
	rB = LenSegOnSeparateAxis(Cross1, Be[0], Be[2]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	// 分離軸 : A3B3
	Cross1 = MyMtVector3::Cross(NAe[2], NBe[2]);
	rA = LenSegOnSeparateAxis(Cross1, Ae[0], Ae[1]);
	rB = LenSegOnSeparateAxis(Cross1, Be[0], Be[1]);
	L = fabs(MyMtVector3::Dot(Interval, Cross1));
	if (L > rA + rB)return false;

	return true;
}
//Draw
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
			Novice::ScreenPrintf(x + column * KColumnWidth, y + row * kRowHeight + kRowHeight, "%6.03f", matrix.m[row][column]);
		}
	}
}
void QuaternionScreenPrintf(int x, int y, const Quaternion& quaternion, const char* label) {
	// x, y 座標に quaternion の各成分を表示
	Novice::ScreenPrintf(x, y, "%.02f", quaternion.x);
	Novice::ScreenPrintf(x + KColumnWidth, y, "%.02f", quaternion.y);
	Novice::ScreenPrintf(x + KColumnWidth * 2, y, "%.02f", quaternion.z);
	Novice::ScreenPrintf(x + KColumnWidth * 3, y, "%.02f", quaternion.w);
	Novice::ScreenPrintf(x + KColumnWidth * 4, y, ": %s", label);
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
	perpendiculars[2] = MyMtVector3::Cross(plane.normal, perpendiculars[0]);
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
	
	Quaternion rotation = Quaternion::MakeRotateAxisAngleQuaternion(MyMtVector3::Normalize(Vector3{ 1.0f,0.4f,-0.2f }), 0.45f);
	Vector3 pointY = { 2.1f, -0.9f, 1.3f };
	Matrix4x4 rotateMatrix = Quaternion::MakeRotateMatrix(rotation);
	Vector3 rotateByQuaternion = Quaternion::RotateVector(pointY, rotation);
	Vector3 rotateByMatrix = MyMtMatrix::Transform(pointY, rotateMatrix);

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
		
		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		Novice::ScreenPrintf(0, 0, "%f", Permutation(5, 3));
		/*QuaternionScreenPrintf(0, 0, rotation, "rotation");
		MatrixScreenPrintf(0, kRowHeight, rotateMatrix, "rotateMatrix");
		VectorScreenPrintf(0, kRowHeight * 6, rotateByQuaternion, " : rotateByQuaternion");
		VectorScreenPrintf(0, kRowHeight * 7, rotateByMatrix, " : rotateByMatrix");*/
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
