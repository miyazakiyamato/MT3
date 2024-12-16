#pragma once
#include "MyMtVector3.h"
#include "Matrix4x4.h"

class MyMtMatrix
{
public:
	static Matrix4x4 Add(const Matrix4x4& m1, const Matrix4x4& m2); 
	static Matrix4x4 Subtract(const Matrix4x4& m1, const Matrix4x4& m2);
	static Matrix4x4 Multiply(const Matrix4x4& m1, const Matrix4x4& m2);
	static Matrix4x4 Inverse(const Matrix4x4& m); 
	static Matrix4x4 Transpose(const Matrix4x4& m); 
	static Matrix4x4 MakeIdentity4x4(); 

	static Matrix4x4 MakeTranslateMatrix(const Vector3& translate);
	static Matrix4x4 MakeScaleMatrix(const Vector3& scale);
	static Vector3 Transform(const Vector3& Vector, const Matrix4x4& matrix);
	static Matrix4x4 MakeRotateXMatrix(float radius);
	static Matrix4x4 MakeRotateYMatrix(float radius);
	static Matrix4x4 MakeRotateZMatrix(float radius);

	static Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3& translate);

	static Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip);
	static Matrix4x4 MakeOrthographicMatrix(float left, float top, float right, float bottom, float nearClip, float farClip);
	static Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth);

};
inline Vector3 operator+(const Vector3& v1, const Vector3& v2) { return MyMtVector3::Add(v1, v2); }
inline Vector3 operator-(const Vector3& v1, const Vector3& v2) { return MyMtVector3::Subtract(v1, v2); }
inline Vector3 operator*(float v1, const Vector3& v2) { return MyMtVector3::Multiply(v1, v2); }
inline Vector3 operator*(const Vector3& v1, float v2) { return v2 * v1; }
inline Vector3 operator/(float v1, const Vector3& v2) { return MyMtVector3::Divide(v1, v2); }
inline Vector3 operator/(const Vector3& v1, float v2) { return v2 / v1; }
inline Vector3 operator-(const Vector3& v) { return { -v.x,-v.y,-v.z }; }
inline Matrix4x4 operator+(const Matrix4x4& m1, const Matrix4x4& m2) { return MyMtMatrix::Add(m1, m2); }
inline Matrix4x4 operator-(const Matrix4x4& m1, const Matrix4x4& m2) { return MyMtMatrix::Subtract(m1, m2); }
inline Matrix4x4 operator*(const Matrix4x4& m1, const Matrix4x4& m2) { return MyMtMatrix::Multiply(m1, m2); }