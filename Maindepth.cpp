#include <cstdlib>
#include <GL/glut.h>
#include <Kinect.h>
#include <iostream>
#include <sstream>

using namespace std;

// スクリーンの幅と高さ
#define HEIGHT 540
#define WIDTH 960

IKinectSensor* pKinect;
IColorFrameReader* pdepthFrameReader;
unsigned char* colorBuffer;
WAITABLE_HANDLE hColorWaitable;

int colorWidth;
int colorHeight;
float screenRatio;
unsigned int texId;

/*
 Kinect 関数の返却値のエラー判定
*/
void ERROR_CHECK(int ret) {
	if (FAILED(ret)) {
		stringstream ss;
		ss << "failed " << ret << " " << hex << ret << endl;
		throw std::runtime_error(ss.str().c_str());
	}
}

/*
 メモリの安全な解放
*/
template<class Interface>
void SafeRelease(Interface*& pInterfaceToRelease) {
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

/*
 表示用関数
 */
void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// 長方形をテクスチャで描く
	glBegin(GL_QUADS);
	glTexCoord2f(0, 1); glVertex3f(-screenRatio, -1.0, 1.0);
	glTexCoord2f(1, 1); glVertex3f(screenRatio, -1.0, 1.0);
	glTexCoord2f(1, 0); glVertex3f(screenRatio, 1.0, 1.0);
	glTexCoord2f(0, 0); glVertex3f(-screenRatio, 1.0, 1.0);

	glTexCoord2f(0, 1); glVertex3f(-screenRatio, 1.0, 1.0);
	glTexCoord2f(1, 1); glVertex3f(screenRatio, 1.0, 1.0);
	glTexCoord2f(1, 0); glVertex3f(screenRatio, 1.0, -1.0);
	glTexCoord2f(0, 0); glVertex3f(-screenRatio, 1.0, -1.0);

	glTexCoord2f(0, 1); glVertex3f(-screenRatio, 1.0, -1.0);
	glTexCoord2f(1, 1); glVertex3f(screenRatio, 1.0, -1.0);
	glTexCoord2f(1, 0); glVertex3f(screenRatio, -1.0, -1.0);
	glTexCoord2f(0, 0); glVertex3f(-screenRatio, -1.0, -1.0);

	glTexCoord2f(0, 1); glVertex3f(-screenRatio, -1.0, -1.0);
	glTexCoord2f(1, 1); glVertex3f(screenRatio, -1.0, -1.0);
	glTexCoord2f(1, 0); glVertex3f(screenRatio, -1.0, 1.0);
	glTexCoord2f(0, 0); glVertex3f(-screenRatio, -1.0, 1.0);
	glEnd();

	glutSwapBuffers();
}

/*
 アイドリング関数
 */
void idle() {
	// Kinect からの返却値を保存する変数
	HRESULT hResult = S_OK;

	// イベント待ちをする配列
	HANDLE hEvents[] = { reinterpret_cast<HANDLE>(hColorWaitable) };

	// Kinect が画面を取得するまで、永遠に待つ
	WaitForMultipleObjects(1, hEvents, true, INFINITE);

	// Frame
	IColorFrame* pColorFrame = nullptr;
	hResult = pColorFrameReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(hResult)){
		// RGB情報をRGBAフォーマットに変換して、colorBuffer にコピー
		hResult = pColorFrame->CopyConvertedFrameDataToArray(colorWidth * colorHeight * 4, colorBuffer, 
			ColorImageFormat::ColorImageFormat_Rgba);
		if (SUCCEEDED(hResult)){
			// texId をテクスチャ番号に指定して、RGB画像をテクスチャに設定する
			glTexImage2D(GL_TEXTURE_2D, 0, 4, colorWidth, colorHeight, 0,
				GL_RGBA, GL_UNSIGNED_BYTE, colorBuffer);
		}
	}
	SafeRelease(pColorFrame);

	glutPostRedisplay();
}

float theta = 0;

void mouse(int button, int state, int x, int y) {
	theta += 0.1;
	glLoadIdentity();
	gluLookAt(0, 3 * sin(theta), 3 * cos(theta), 0, 0, 0, 0, cos(theta), 0);
	glutPostRedisplay();
}


void openGLInit(int* argc, char** argv) {
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutCreateWindow("KinectColor");
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutMouseFunc(mouse);
	glEnable(GL_DEPTH_TEST); // 深度用に追加

	// テクスチャの初期設定
	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, &texId);
	glBindTexture(GL_TEXTURE_2D, texId);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-screenRatio, screenRatio, -1.0, 1.0, 1.0, 4.0);
	glMatrixMode(GL_MODELVIEW);
	gluLookAt(0, 0, 3, 0, 0, 0, 0, 1, 0);
}

/* 
 * kinect の初期化関数
 */
int kinectInit() {
	// 標準の Kinect Sensor を取得
	ERROR_CHECK(GetDefaultKinectSensor(&pKinect));

	if (pKinect) {
		IColorFrameSource* pColorFrameSource;
		IFrameDescription* pColorFrameDescription;

		ERROR_CHECK(pKinect->Open());

		// 以下では、3つの事をまとめて実施している
		// (1) ColorFrameReader の取得
		//   Kinect から ColorFrameSource を取得し
		//   ColorFrameSource から ColorFrameReader を取得
		// (2) Reader のイベント到着待ちのハンドラの設定
		//   ColorFrameReader に、CoolorWaitable を設定
		// (3) 画面の幅と高さを取得
		//   ColorFrameSource から ColorFrameDescription を取得し
		//   ColorFrameDescription から get_Height(), get_Width()を呼ぶ

		// (1)
		ERROR_CHECK(pKinect->get_ColorFrameSource(&pColorFrameSource));
		ERROR_CHECK(pColorFrameSource->OpenReader(&pColorFrameReader));

		// (2)
		ERROR_CHECK(pColorFrameReader->SubscribeFrameArrived(&hColorWaitable));

		// (3)
		ERROR_CHECK(pColorFrameSource->get_FrameDescription(&pColorFrameDescription));
		pColorFrameDescription->get_Height(&colorHeight);
		pColorFrameDescription->get_Width(&colorWidth);
		screenRatio = (float)WIDTH / (float)HEIGHT;

		// カラー画面を保存する配列を確保
		colorBuffer = new unsigned char[colorHeight * colorWidth * 4];

		// 途中で利用して不要になった ColorFrameSource とColorFrameDescription を開放
		SafeRelease(pColorFrameSource);
		SafeRelease(pColorFrameDescription);

		return 1;
	}
	else {
		cout << "Kinect が接続されていません。\n";
		return 0;
	}
}

int main(int argc, char **argv) {
	// Kinect の初期化
	if (kinectInit() == 0) {
		exit(1);
	}	
	
	// OpenGL の初期化
	openGLInit(&argc, argv);

	// OpenGL のメインループ
	glutMainLoop();
}