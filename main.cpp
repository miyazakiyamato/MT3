#define _USE_MATH_DEFINES
#include <Novice.h>
#include <cstdint>
#include <math.h>
#include "MyMtVector3.h"
#include "MyMtMatrix.h"
#include "imgui.h"

const char kWindowTitle[] = "LD2B_07_ミヤザキ_ヤマト";


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
			a = MyMtVector3::Add(sphere.center,MyMtVector3::Multiply(sphere.radius,{ cosf(lat) * cosf(lon),sinf(lat),cosf(lat) * sinf(lon) }));
			b = MyMtVector3::Add(sphere.center, MyMtVector3::Multiply(sphere.radius, { cosf(lat + kLatEvery) * cosf(lon),sinf(lat + kLatEvery),cosf(lat + kLatEvery) * sinf(lon) }));
			c = MyMtVector3::Add(sphere.center, MyMtVector3::Multiply(sphere.radius, { cosf(lat) * cosf(lon + kLonEvery),sinf(lat),cosf(lat) * sinf(lon + kLonEvery) }));
			
			
			Vector3 ndcVertex[3]{ MyMtMatrix::Transform(a, viewProjectionMatrix),MyMtMatrix::Transform(b, viewProjectionMatrix),MyMtMatrix::Transform(c, viewProjectionMatrix) };
			Vector3 screenVertices[3]{ MyMtMatrix::Transform(ndcVertex[0], viewportMatrix),MyMtMatrix::Transform(ndcVertex[1], viewportMatrix),MyMtMatrix::Transform(ndcVertex[2], viewportMatrix)};
			
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
		Vector3 ndcVertex = MyMtMatrix::Transform({ kGridEvery * xIndex - kGridEvery * kSubdivision / 2,0.0f, -kGridEvery * (kSubdivision / 2)}, viewProjectionMatrix);
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
		Vector3 ndcVertex = MyMtMatrix::Transform({-kGridEvery * (kSubdivision / 2),0.0f,kGridEvery * zIndex - kGridEvery * kSubdivision / 2}, viewProjectionMatrix);
		Vector3 screenVertices = MyMtMatrix::Transform(ndcVertex, viewportMatrix);
		ndcVertex = MyMtMatrix::Transform({kGridEvery * kSubdivision / 2,0.0f,kGridEvery * zIndex - kGridEvery * kSubdivision / 2 }, viewProjectionMatrix);
		Vector3 screenVertices2 = MyMtMatrix::Transform(ndcVertex, viewportMatrix);
		if (zIndex == 5) {
			Novice::DrawLine(int(screenVertices.x), int(screenVertices.y), int(screenVertices2.x), int(screenVertices2.y), 0x000000ff);
		}
		else {
			Novice::DrawLine(int(screenVertices.x), int(screenVertices.y), int(screenVertices2.x), int(screenVertices2.y), 0xaaaaaaff);
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

	Segment segment{ {-2.0f,-1.0f,0.0f},{3.0f,2.0f,2.0f} };
	Vector3 point{ -1.5f,0.6f,0.6f };

	Vector3 project{};
	Vector3 closestPoint{};

	Sphere pointSphere{ point,0.01f };
	Sphere closestPointSphere{ closestPoint,0.01f };

	Vector3 start{};
	Vector3 end{};
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
		
		project = MyMtVector3::Project(MyMtVector3::Subtract(point, segment.origin), segment.diff);
		closestPoint = MyMtVector3::ClosestPoint(point, segment);
		
		pointSphere = { point,0.01f };
		closestPointSphere = { closestPoint,0.01f };
		start = MyMtMatrix::Transform(MyMtMatrix::Transform(segment.origin, worldViewProjectionMatrix), viewportMatrix);
		end = MyMtMatrix::Transform(MyMtMatrix::Transform(MyMtVector3::Add(segment.origin,segment.diff), worldViewProjectionMatrix), viewportMatrix);

		ImGui::Begin("Window");
		ImGui::DragFloat3("point", &point.x, 0.01f);
		ImGui::DragFloat3("segment.origin", &segment.origin.x, 0.01f);
		ImGui::DragFloat3("segment.diff", &segment.diff.x, 0.01f);
		ImGui::InputFloat3("Project", &project.x, "%.3f",ImGuiInputTextFlags_ReadOnly);
		ImGui::End();
		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		DrawGrid(worldViewProjectionMatrix, viewportMatrix);
		Novice::DrawLine((int)start.x, (int)start.y, (int)end.x, (int)end.y, WHITE);
		DrawSphere(pointSphere, worldViewProjectionMatrix, viewportMatrix, RED);
		DrawSphere(closestPointSphere, worldViewProjectionMatrix, viewportMatrix, BLACK);
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