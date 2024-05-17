#define _USE_MATH_DEFINES
#include <Novice.h>
#include <cstdint>
#include <math.h>
#include "MyMtVector3.h"
#include "MyMtMatrix.h"
#include "imgui.h"

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

	Vector3 cameraTranslate{ 0.0f,1.9f,-6.49f };
	Vector3 cameraRotate{ 0.26f,0.0f,0.0f };

	Matrix4x4 worldMatrix{};
	Matrix4x4 cameraMatrix{};

	Matrix4x4 viewMatrix{};
	Matrix4x4 projectionMatrix{};
	Matrix4x4 worldViewProjectionMatrix{};
	Matrix4x4 viewportMatrix{};


	int mouseX = 0;
	int mouseY = 0;
	Vector3 mouse{};
	Vector3 preMouse{};
	Vector3 wheelVelocity{};

	float kCameraSpeed = 0.03f;
	Vector3 cameraVelocity{};

	Sphere sphere1{
		{0.0f,0.0f,0.0f},
		1.0f
	};
	unsigned int sphereColor1 = 0xffffffff;
	Plane plane{
		{0.0f,1.0f,0.0f},
		1.0f
	};

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
		worldMatrix = MyMtMatrix::MakeAffineMatrix({ 1.0f,1.0f,1.0f }, rotate, translate);
		cameraMatrix = MyMtMatrix::MakeAffineMatrix({ 1.0f,1.0f,1.0f }, cameraRotate, cameraTranslate);
		viewMatrix = MyMtMatrix::Inverse(cameraMatrix);
		projectionMatrix = MyMtMatrix::MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
		worldViewProjectionMatrix = MyMtMatrix::Multiply(worldMatrix, MyMtMatrix::Multiply(viewMatrix, projectionMatrix));
		viewportMatrix = MyMtMatrix::MakeViewportMatrix(0, 0, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

		sphereColor1 = IsCollision(sphere1, plane) == true ? 0xff0000ff : 0xffffffff;

		ImGui::Begin("Window");
		ImGui::DragFloat3("CameraTranslate", &cameraTranslate.x, 0.01f);
		ImGui::DragFloat3("CameraRotate", &cameraRotate.x, 0.01f);
		ImGui::DragFloat3("SphereCenter1", &sphere1.center.x, 0.01f);
		ImGui::DragFloat("SphereRadius1", &sphere1.radius, 0.01f);
		ImGui::DragFloat3("Plane.Normal", &plane.normal.x, 0.01f);
		ImGui::DragFloat("PlaneDistance", &plane.distance, 0.01f);
		plane.normal = MyMtVector3::Normalize(plane.normal);
		ImGui::End();
		preMouse = mouse;
		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		DrawGrid(worldViewProjectionMatrix, viewportMatrix);
		DrawSphere(sphere1, worldViewProjectionMatrix, viewportMatrix, sphereColor1);
		DrawPlane(plane, worldViewProjectionMatrix, viewportMatrix,0xffffffff);
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
