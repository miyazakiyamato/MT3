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
struct Sphere
{
	Vector3 center;
	float radius;
};
//void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color);
void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix) {
	const float kGridHalfwidth = 2.0f;
	const uint32_t kSubdivision = 10;
	const float kGridEvery = (kGridHalfwidth * 2.0f) / float(kSubdivision);
	//
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex) {
		Vector3 ndcVertex = MyMtMatrix::Transform({ kGridEvery * xIndex - kGridEvery * kSubdivision / 2,0.0f, -kGridEvery * (kSubdivision / 2)}, viewProjectionMatrix);
		Vector3 screenVertices = MyMtMatrix::Transform(ndcVertex, viewportMatrix);
		ndcVertex = MyMtMatrix::Transform({ kGridEvery * xIndex - kGridEvery * kSubdivision / 2,0.0f, kGridEvery * kSubdivision / 2 }, viewProjectionMatrix);
		Vector3 screenVertices2 = MyMtMatrix::Transform(ndcVertex, viewportMatrix);
		Novice::DrawLine(int(screenVertices.x), int(screenVertices.y), int(screenVertices2.x), int(screenVertices2.y), 0xaaaaaaff);
	}
	//
	for (uint32_t zIndex = 0; zIndex <= kSubdivision; ++zIndex) {
		Vector3 ndcVertex = MyMtMatrix::Transform({-kGridEvery * (kSubdivision / 2),0.0f,kGridEvery * zIndex - kGridEvery * kSubdivision / 2}, viewProjectionMatrix);
		Vector3 screenVertices = MyMtMatrix::Transform(ndcVertex, viewportMatrix);
		ndcVertex = MyMtMatrix::Transform({kGridEvery * kSubdivision / 2,0.0f,kGridEvery * zIndex - kGridEvery * kSubdivision / 2 }, viewProjectionMatrix);
		Vector3 screenVertices2 = MyMtMatrix::Transform(ndcVertex, viewportMatrix);
		Novice::DrawLine(int(screenVertices.x), int(screenVertices.y), int(screenVertices2.x), int(screenVertices2.y), 0xaaaaaaff);
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
	Vector3 translate{};
	Vector3 rotate{};

	int kWindowWidth = 1280;
	int kWindowHeight = 720;

	Vector3 cameraTranslate{0.0f,1.9f,-6.49f};
	Vector3 cameraRotate{0.26f,0.0f,0.0f};

	Matrix4x4 worldMatrix{};
	Matrix4x4 cameraMatrix{};

	Matrix4x4 viewMatrix{};
	Matrix4x4 projectionMatrix{};
	Matrix4x4 worldViewProjectionMatrix{};
	Matrix4x4 viewportMatrix{};
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
		worldMatrix = MyMtMatrix::MakeAffineMatrix({ 1.0f,1.0f,1.0f }, rotate, translate);
		cameraMatrix = MyMtMatrix::MakeAffineMatrix({ 1.0f,1.0f,1.0f }, cameraRotate, cameraTranslate);
		viewMatrix = MyMtMatrix::Inverse(cameraMatrix);
		projectionMatrix = MyMtMatrix::MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		worldViewProjectionMatrix = MyMtMatrix::Multiply(worldMatrix, MyMtMatrix::Multiply(viewMatrix, projectionMatrix));
		viewportMatrix = MyMtMatrix::MakeViewportMatrix(0, 0, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		DrawGrid(worldViewProjectionMatrix, viewportMatrix);
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
