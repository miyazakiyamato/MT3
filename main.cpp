#include <Novice.h>
#include <cstdint>
#include "MyMtVector3.h"
#include "MyMtMatrix.h"


const char kWindowTitle[] = "LD2B_07_ミヤザキ_ヤマト";

//クロス積
Vector3 Cross(const Vector3& v1, const Vector3& v2) {
	Vector3 ab{ v1.y * v2.z - v1.z * v2.y,v1.z * v2.x - v1.x * v2.z,v1.x * v2.y - v1.y * v2.x };
	return ab;
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
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	Vector3 v1{ 1.2f,-3.9f,2.5f };
	Vector3 v2{ 2.8f,0.4f,-1.3f };
	Vector3 cross = Cross(v1, v2);
	//
	Vector3 rotate{};
	Vector3 translate{};
	int kWindowWidth = 1280;
	int kWindowHeight = 720;
	Vector3 cameraPosition{ 0,0,-100 };
	Matrix4x4 worldMatrix{};
	Matrix4x4 cameraMatrix{};
	Matrix4x4 viewMatrix{};
	Matrix4x4 projectionMatrix{};
	Matrix4x4 worldViewProjectionMatrix{};
	Matrix4x4 viewportMatrix{};
	Vector3 screenVertices[3];
	Vector3 kLocalVertices[3]{ {0,10,0},{10,-10,0},{-10,-10,0} };
	//
	float speed = 3;
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
		if (keys[DIK_A]) {
			translate.x -= speed / 10;
		}
		if (keys[DIK_D]) {
			translate.x += speed / 10;
		}
		if (keys[DIK_W]) {
			translate.z += speed;
		}
		if (keys[DIK_S]) {
			translate.z -= speed;
		}
		rotate.y += speed / 50;
		//
		worldMatrix = MyMtMatrix::MakeAffineMatrix({ 1.0f,1.0f,1.0f }, rotate, translate);
		cameraMatrix = MyMtMatrix::MakeAffineMatrix({ 1.0f,1.0f,1.0f }, { 0.0f,0.0f,0.0f }, cameraPosition);
		viewMatrix = MyMtMatrix::Inverse(cameraMatrix);
		projectionMatrix = MyMtMatrix::MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		worldViewProjectionMatrix = MyMtMatrix::Multiply(worldMatrix, MyMtMatrix::Multiply(viewMatrix, projectionMatrix));
		viewportMatrix = MyMtMatrix::MakeViewportMatrix(0, 0, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);
		for (uint32_t i = 0; i < 3; ++i) {
			Vector3 ndcVertex = MyMtMatrix::Transform(kLocalVertices[i], worldViewProjectionMatrix);
			screenVertices[i] = MyMtMatrix::Transform(ndcVertex, viewportMatrix);
		}

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		VectorScreenPrintf(0, 0, cross, "Cross");
		Novice::DrawTriangle(
			int(screenVertices[0].x), int(screenVertices[0].y), int(screenVertices[1].x), int(screenVertices[1].y),
			int(screenVertices[2].x), int(screenVertices[2].y), RED, kFillModeSolid
		);
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
