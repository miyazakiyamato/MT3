#include "Quaternion.h"
#include <math.h>

Matrix4x4 Quaternion::MakeRotateAxisAngle(const Vector3& axis, float angle)
{
	// 回転軸を正規化
	Vector3 axisNormal = MyMtVector3::Normalize(axis);

	//何度も使う計算
	float cosTheta = cosf(angle);
	float sinTheta = -sinf(angle);
	Vector3 axisCos = axisNormal * (1.0f - cosTheta);

	Matrix4x4 rotateMatrix = {
		cosTheta + axisNormal.x * axisCos.x, axisNormal.x * axisCos.y + -axisNormal.z * sinTheta, axisNormal.x * axisCos.z + axisNormal.y * sinTheta, 0.0f,
		axisNormal.x * axisCos.y + axisNormal.z * sinTheta,cosTheta + axisNormal.y * axisCos.y, axisNormal.y * axisCos.z + -axisNormal.x * sinTheta, 0.0f,
		axisNormal.x * axisCos.z + -axisNormal.y * sinTheta, axisNormal.y * axisCos.z + axisNormal.x * sinTheta,cosTheta + axisNormal.z * axisCos.z, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	};

	return rotateMatrix;
}
Matrix4x4 Quaternion::DirectionToDirection(const Vector3& from, const Vector3& to)
{
	// fromとtoの正規化
	Vector3 fromNormal = MyMtVector3::Normalize(from);
	Vector3 toNormal = MyMtVector3::Normalize(to);

	// 回転軸（クロス積）
	Vector3 axis = MyMtVector3::Cross(fromNormal, toNormal);

	// 回転角度（内積からコサインを計算してアークコサインで角度に変換）
	float angle = acosf(MyMtVector3::Dot(fromNormal, toNormal));
	if (axis.x == 0.0f && axis.y == 0.0f && axis.z == 0.0f) {
		if (toNormal.x != 0 || toNormal.y != 0) {
			axis = { toNormal.y,-toNormal.x,0 };
		}
		else if (toNormal.x != 0 || toNormal.z != 0) {
			axis = { toNormal.z,0,-toNormal.x };
		}
	}
	// 回転軸を正規化
	axis = MyMtVector3::Normalize(axis);

	// 回転行列を計算
	Matrix4x4 rotationMatrix = MakeRotateAxisAngle(axis, angle);

	return rotationMatrix;
}

Quaternion Quaternion::Identity()
{
	return Quaternion(0.0f,0.0f,0.0f,1.0f);
}

Quaternion Quaternion::Conjugate(const Quaternion& quaternion)
{
	return Quaternion(-quaternion.x, -quaternion.y, -quaternion.z, quaternion.w);
}

Quaternion Quaternion::Inverse(const Quaternion& quaternion)
{
	float norm = Norm(quaternion);
	if (norm == 0.0f) {
		// 単位Quaternionを返す
		return Identity();
	}
	Quaternion conjugate = Conjugate(quaternion);
	float norm2 = norm * norm;
	return conjugate / norm2;
}

Quaternion Quaternion::Normalize(const Quaternion& quaternion)
{
	float norm = Norm(quaternion);
	if (norm == 0.0f) {
		// 単位Quaternionを返す
		return Identity();
	}
	return (Quaternion)quaternion / norm;
}

Quaternion Quaternion::Multiply(const Quaternion& lhs, const Quaternion& rhs)
{
	Quaternion result;
	result.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
	result.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
	result.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
	result.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
	return result;
}

float Quaternion::Norm(const Quaternion& quaternion)
{
	return sqrtf(quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z + quaternion.w * quaternion.w);
}

Quaternion Quaternion::operator/(const float& f)
{
	Quaternion result;
	result.x = (*this).x / f;
	result.y = (*this).y / f;
	result.z = (*this).z / f;
	result.w = (*this).w / f;
	return result;
}
