#include <Novice.h>
#include <cassert>
#include <cmath>

const char kWindowTitle[] = "LD2B_07_ミヤザキ_ヤマト";

struct Vector3 {
	float x;
	float y;
	float z;
};
struct Matrix4x4
{
	float m[4][4];
};

Matrix4x4 Multiply(const Matrix4x4& m1, const Matrix4x4& m2) {
	Matrix4x4 m3;
	m3.m[0][0] = m1.m[0][0] * m2.m[0][0] + m1.m[0][1] * m2.m[1][0] + m1.m[0][2] * m2.m[2][0] + m1.m[0][3] * m2.m[3][0]; m3.m[0][1] = m1.m[0][0] * m2.m[0][1] + m1.m[0][1] * m2.m[1][1] + m1.m[0][2] * m2.m[2][1] + m1.m[0][3] * m2.m[3][1]; m3.m[0][2] = m1.m[0][0] * m2.m[0][2] + m1.m[0][1] * m2.m[1][2] + m1.m[0][2] * m2.m[2][2] + m1.m[0][3] * m2.m[3][2]; m3.m[0][3] = m1.m[0][0] * m2.m[0][3] + m1.m[0][1] * m2.m[1][3] + m1.m[0][2] * m2.m[2][3] + m1.m[0][3] * m2.m[3][3];
	m3.m[1][0] = m1.m[1][0] * m2.m[0][0] + m1.m[1][1] * m2.m[1][0] + m1.m[1][2] * m2.m[2][0] + m1.m[1][3] * m2.m[3][0]; m3.m[1][1] = m1.m[1][0] * m2.m[0][1] + m1.m[1][1] * m2.m[1][1] + m1.m[1][2] * m2.m[2][1] + m1.m[1][3] * m2.m[3][1]; m3.m[1][2] = m1.m[1][0] * m2.m[0][2] + m1.m[1][1] * m2.m[1][2] + m1.m[1][2] * m2.m[2][2] + m1.m[1][3] * m2.m[3][2]; m3.m[1][3] = m1.m[1][0] * m2.m[0][3] + m1.m[1][1] * m2.m[1][3] + m1.m[1][2] * m2.m[2][3] + m1.m[1][3] * m2.m[3][3];
	m3.m[2][0] = m1.m[2][0] * m2.m[0][0] + m1.m[2][1] * m2.m[1][0] + m1.m[2][2] * m2.m[2][0] + m1.m[2][3] * m2.m[3][0]; m3.m[2][1] = m1.m[2][0] * m2.m[0][1] + m1.m[2][1] * m2.m[1][1] + m1.m[2][2] * m2.m[2][1] + m1.m[2][3] * m2.m[3][1]; m3.m[2][2] = m1.m[2][0] * m2.m[0][2] + m1.m[2][1] * m2.m[1][2] + m1.m[2][2] * m2.m[2][2] + m1.m[2][3] * m2.m[3][2]; m3.m[2][3] = m1.m[2][0] * m2.m[0][3] + m1.m[2][1] * m2.m[1][3] + m1.m[2][2] * m2.m[2][3] + m1.m[2][3] * m2.m[3][3];
	m3.m[3][0] = m1.m[3][0] * m2.m[0][0] + m1.m[3][1] * m2.m[1][0] + m1.m[3][2] * m2.m[2][0] + m1.m[3][3] * m2.m[3][0]; m3.m[3][1] = m1.m[3][0] * m2.m[0][1] + m1.m[3][1] * m2.m[1][1] + m1.m[3][2] * m2.m[2][1] + m1.m[3][3] * m2.m[3][1]; m3.m[3][2] = m1.m[3][0] * m2.m[0][2] + m1.m[3][1] * m2.m[1][2] + m1.m[3][2] * m2.m[2][2] + m1.m[3][3] * m2.m[3][2]; m3.m[3][3] = m1.m[3][0] * m2.m[0][3] + m1.m[3][1] * m2.m[1][3] + m1.m[3][2] * m2.m[2][3] + m1.m[3][3] * m2.m[3][3];
	return m3;
};

Matrix4x4 MakeTranslateMatrix(const Vector3& translate) {
	Matrix4x4 m;
	m.m[0][0] = 1; m.m[0][1] = 0; m.m[0][2] = 0; m.m[0][3] = 0;
	m.m[1][0] = 0; m.m[1][1] = 1; m.m[1][2] = 0; m.m[1][3] = 0;
	m.m[2][0] = 0; m.m[2][1] = 0; m.m[2][2] = 1; m.m[2][3] = 0;
	m.m[3][0] = translate.x; m.m[3][1] = translate.y; m.m[3][2] = translate.z; m.m[3][3] = 1;
	return m;
}
Matrix4x4 MakeScaleMatrix(const Vector3& scale) {
	Matrix4x4 m;
	m.m[0][0] = scale.x; m.m[0][1] = 0; m.m[0][2] = 0; m.m[0][3] = 0;
	m.m[1][0] = 0; m.m[1][1] = scale.y; m.m[1][2] = 0; m.m[1][3] = 0;
	m.m[2][0] = 0; m.m[2][1] = 0; m.m[2][2] = scale.z; m.m[2][3] = 0;
	m.m[3][0] = 0; m.m[3][1] = 0; m.m[3][2] = 0; m.m[3][3] = 1;
	return m;
}
Vector3 Transform(const Vector3& Vector, const Matrix4x4& matrix) {
	Vector3 v;
	v.x = Vector.x * matrix.m[0][0] + Vector.y * matrix.m[1][0] + Vector.z * matrix.m[2][0] + matrix.m[3][0];
	v.y = Vector.x * matrix.m[0][1] + Vector.y * matrix.m[1][1] + Vector.z * matrix.m[2][1] + matrix.m[3][1];
	v.z = Vector.x * matrix.m[0][2] + Vector.y * matrix.m[1][2] + Vector.z * matrix.m[2][2] + matrix.m[3][2];
	float w = Vector.x * matrix.m[0][3] + Vector.y * matrix.m[1][3] + Vector.z * matrix.m[2][3] + matrix.m[3][3];

	assert(w != 0.0f);
	v.x /= w;
	v.y /= w;
	v.z /= w;

	return v;
}

Matrix4x4 MakeRotateXMatrix(float radius) {
	Matrix4x4 m;
	m.m[0][0] = 1; m.m[0][1] = 0; m.m[0][2] = 0; m.m[0][3] = 0;
	m.m[1][0] = 0; m.m[1][1] = std::cosf(radius); m.m[1][2] = std::sinf(radius); m.m[1][3] = 0;
	m.m[2][0] = 0; m.m[2][1] = -std::sinf(radius); m.m[2][2] = std::cosf(radius);  m.m[2][3] = 0;
	m.m[3][0] = 0; m.m[3][1] = 0; m.m[3][2] = 0; m.m[3][3] = 1;
	return m;
}
Matrix4x4 MakeRotateYMatrix(float radius) {
	Matrix4x4 m;
	m.m[0][0] = std::cosf(radius); m.m[0][1] = 0; m.m[0][2] = -std::sinf(radius); m.m[0][3] = 0;
	m.m[1][0] = 0; m.m[1][1] = 1; m.m[1][2] = 0; m.m[1][3] = 0;
	m.m[2][0] = std::sinf(radius); m.m[2][1] = 0; m.m[2][2] = std::cosf(radius); m.m[2][3] = 0;
	m.m[3][0] = 0; m.m[3][1] = 0; m.m[3][2] = 0; m.m[3][3] = 1;
	return m;
}
Matrix4x4 MakeRotateZMatrix(float radius) {
	Matrix4x4 m;
	m.m[0][0] = std::cosf(radius); m.m[0][1] = std::sinf(radius); m.m[0][2] = 0; m.m[0][3] = 0;
	m.m[1][0] = -std::sinf(radius); m.m[1][1] = std::cosf(radius); m.m[1][2] = 0; m.m[1][3] = 0;
	m.m[2][0] = 0; m.m[2][1] = 0; m.m[2][2] = 1; m.m[2][3] = 0;
	m.m[3][0] = 0; m.m[3][1] = 0; m.m[3][2] = 0; m.m[3][3] = 1;
	return m;
}
Matrix4x4 MakeAffineMatrix(const Vector3& scale, const Vector3& rotate, const Vector3& translate ) {
	Matrix4x4 m;
	Matrix4x4 rotateMatrix = Multiply(MakeRotateXMatrix(rotate.x), Multiply(MakeRotateYMatrix(rotate.y), MakeRotateZMatrix(rotate.z)));
		m = Multiply(MakeScaleMatrix(scale), Multiply(rotateMatrix, MakeTranslateMatrix(translate)));
		return m;
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
// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	Vector3 scale{ 1.2f,0.79f,-2.1f };
	Vector3 rotate{ 0.4f,1.43f,-0.8f };
	Vector3 translate{ 2.7f,-4.15f,1.57f };
	Matrix4x4 worldMatrix = MakeAffineMatrix(scale,rotate,translate);

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

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		MatrixScreenPrintf(0, 0, worldMatrix, "worldMatrix");
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
