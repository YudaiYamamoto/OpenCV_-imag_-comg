#include <cstdlib>
#include <GL/glut.h>
#include <Kinect.h>
#include <iostream>
#include <sstream>

using namespace std;

#define HEIGHT 540
#define WIDTH 960

IKinectSensor* pKinect;
IDepthFrameReader* pDepthFrameReader;
ICoordinateMapper* pCoordinateMapper;
UINT16* depthBuffer;
CameraSpacePoint* pCameraSpacePoint;
unsigned char* colorBuffer;
WAITABLE_HANDLE hDepthWaitable;
int depthWidth;
int depthHeight;
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

	for(int i = 0; i < depthHeight - 1; i++) {
		for(int j = 0; j < depthWidth - 1; j++) {
			int index = i * depthWidth + j;

			if(depthBuffer[index] > 0 && depthBuffer[index + 1] > 0
				&& depthBuffer[index + depthWidth] > 0 && depthBuffer[index + depthWidth + 1] > 0) {
				    glColor3f(1.0, 1.0,0.0);
					glBegin(GL_LINES);
					glVertex3f(pCameraSpacePoint[index].X, pCameraSpacePoint[index].Y, pCameraSpacePoint[index].Z);
					glVertex3f(pCameraSpacePoint[index + 1].X, pCameraSpacePoint[index + 1].Y, pCameraSpacePoint[index + 1].Z);
					glVertex3f(pCameraSpacePoint[index + depthWidth + 1].X, pCameraSpacePoint[index + depthWidth + 1].Y, pCameraSpacePoint[index + depthWidth + 1].Z);
					glVertex3f(pCameraSpacePoint[index + depthWidth].X, pCameraSpacePoint[index + depthWidth].Y, pCameraSpacePoint[index + depthWidth].Z);
					glEnd();
			}
		}
	}

	glutSwapBuffers();
}

/*
 アイドリング関数
 */
void idle() {
	// Kinect からの返却値を保存する変数
	HRESULT hResult = S_OK;

	// イベント待ちをする配列
	HANDLE hEvents[] = { reinterpret_cast<HANDLE>(hDepthWaitable) };

	// Kinect が画面を取得するまで、永遠に待つ
	WaitForMultipleObjects(1, hEvents, true, INFINITE);

	// Frame
	IDepthFrame* pDepthFrame = nullptr;
	hResult = pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	// hResult = pDepthReference->AcquireFrame(&pDepthFrame);
	if (SUCCEEDED(hResult)){
		// RGB情報をRGBAフォーマットに変換
		hResult = pDepthFrame->CopyFrameDataToArray(depthWidth * depthHeight, depthBuffer); 
		if (SUCCEEDED(hResult)){
			hResult = pCoordinateMapper->MapDepthFrameToCameraSpace(depthWidth * depthHeight, 
				depthBuffer, depthWidth * depthHeight, pCameraSpacePoint);		
			if(SUCCEEDED(hResult)) {
			} else {
				cout << "error\n";
			}
		}
	}
	SafeRelease(pDepthFrame);

	glutPostRedisplay();
}

float theta = 0;

void mouse(int button, int state, int x, int y) {
	theta += 0.1;
	glLoadIdentity();
	gluLookAt(0, 2 * sin(theta), 2 - 2 * cos(theta), 0, 0, 2, 0, cos(theta), 0);
	glutPostRedisplay();
}


void openGLInit(int* argc, char** argv) {
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutCreateWindow("KinectDepth");
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
	glBindTexture(GL_TEXTURE_2D, 0);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float n = 0.3;
	glFrustum(-screenRatio * n, screenRatio * n, -n, n, n, 8.0);
	glMatrixMode(GL_MODELVIEW);
	gluLookAt(0, 0, 0, 0, 0, 2, 0, 1, 0);
}

/* 
 * kinect の初期化関数
 */
int kinectInit() {
	// 標準の Kinect Sensor を取得
	ERROR_CHECK(GetDefaultKinectSensor(&pKinect));

	if (pKinect) {
		IDepthFrameSource* pDepthFrameSource;
		IFrameDescription* pDepthFrameDescription;
		ERROR_CHECK(pKinect->Open());

		// 以下では、3つの事をまとめて実施している
		// (1) DepthFrameReader の取得
		//   Kinect から DepthFrameSource を取得し
		//   DepthFrameSource から DepthFrameReader を取得
		// (2) Reader のイベント到着待ちのハンドラの設定
		//   DepthFrameReader に、CoolorWaitable を設定
		// (3) 画面の幅と高さを取得
		//   DepthFrameSource から DepthFrameDescription を取得し
		//   DepthFrameDescription から get_Height(), get_Width()を呼ぶ

		// (1)
		ERROR_CHECK(pKinect->get_DepthFrameSource(&pDepthFrameSource));
		ERROR_CHECK(pDepthFrameSource->OpenReader(&pDepthFrameReader));

		// (2)
		ERROR_CHECK(pDepthFrameReader->SubscribeFrameArrived(&hDepthWaitable));

		// (3)
		ERROR_CHECK(pDepthFrameSource->get_FrameDescription(&pDepthFrameDescription));
		pDepthFrameDescription->get_Height(&depthHeight);
		pDepthFrameDescription->get_Width(&depthWidth);
		screenRatio = (float)WIDTH / (float)HEIGHT;

		// カラー画面を保存する配列を確保
		depthBuffer = new UINT16[depthHeight * depthWidth];
		colorBuffer = new unsigned char[depthHeight * depthWidth * 4];
		pCameraSpacePoint = new CameraSpacePoint[depthHeight * depthWidth];

		// 途中で利用して不要になった DepthFrameSource とDepthFrameDescription を開放
		SafeRelease(pDepthFrameSource);
		SafeRelease(pDepthFrameDescription);

		ERROR_CHECK(pKinect->get_CoordinateMapper(&pCoordinateMapper));

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
