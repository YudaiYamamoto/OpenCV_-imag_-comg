#include <cstdlib>
#include <GL/glut.h>
#include <Kinect.h>
#include <iostream>
#include <sstream>

using namespace std;

#define HEIGHT 540
#define WIDTH 960

IKinectSensor* pKinect;
IMultiSourceFrameReader* pMultiSourceFrameReader;

ICoordinateMapper* pCoordinateMapper;
UINT16* depthBuffer;
unsigned char* colorBuffer;
CameraSpacePoint* pCameraSpacePoint = nullptr;
ColorSpacePoint* pColorSpacePoint = nullptr;
IBody **pBody = nullptr;
WAITABLE_HANDLE waitableHandle;
int depthWidth;
int depthHeight;
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

void drawSphere(CameraSpacePoint pos, float r) {
	glBindTexture(GL_TEXTURE_2D, 0);
	glPushMatrix();
	// glLoadIdentity();
	glTranslatef(pos.X, pos.Y, pos.Z);
	//glColor3f(0, 0, 1);
	glutSolidSphere(r, 40, 40);
	// glutSolidTeapot(r);
	// glTranslatef(pos.X, pos.Y, pos.Z);
	glColor3f(1, 1, 1);
	glPopMatrix();
}

#define PI 3.141592

void cross(float* x, float* y, float* result) {
  result[0] = x[1] * y[2] - x[2] * y[1];
  result[1] = x[2] * y[0] - x[0] * y[2];
  result[2] = x[0] * y[1] - x[1] * y[0];
}

void norm(float* x, float* result) {
  float length = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
  result[0] = x[0] / length;
  result[1] = x[1] / length;
  result[2] = x[2] / length;
}

void drawCylinder(CameraSpacePoint top, CameraSpacePoint bottom, float r, 
		  int slides) {
  // direction
  float vec[3];
  vec[0] = top.X - bottom.X;
  vec[1] = top.Y - bottom.Y;
  vec[2] = top.Z - bottom.Z;

  float mx = vec[0];
  int index = 0;
  if(mx < vec[1]) {
    mx = vec[1];
    index = 1;
  }
  if(mx < vec[2]) {
    mx = vec[2];
    index = 2;
  }

  // 横
  float vec2[3] = {0};
  if(index == 0) {
    vec2[1] = 1;
  } else {
    vec2[0] = 1;
  }

  // 縦
  float vec3[3];
  cross(vec, vec2, vec3);
  norm(vec3, vec3);

  cross(vec, vec3, vec2);
  norm(vec2, vec2);

  // 上面
  glBegin(GL_POLYGON);
  for(int i = 0; i < slides; i++) {
    float theta = 2 * i * PI / slides;
    glVertex3f(top.X + r * vec2[0] * cos(theta) + r * vec3[0] * sin(theta),
	     top.Y + r * vec2[1] * cos(theta) + r * vec3[1] * sin(theta),
	     top.Z + r * vec2[2] * cos(theta) + r * vec3[2] * sin(theta));
  }
  glEnd();

  // 底面
  glBegin(GL_POLYGON);
  for(int i = 0; i < slides; i++) {
    float theta = 2 * i * PI / slides;
    glVertex3f(bottom.X + r * vec2[0] * cos(theta) + r * vec3[0] * sin(theta),
	     bottom.Y + r * vec2[1] * cos(theta) + r * vec3[1] * sin(theta),
	     bottom.Z + r * vec2[2] * cos(theta) + r * vec3[2] * sin(theta));
  }
  glEnd();

  // 側面
  glBegin(GL_QUADS);
  for(int i = 0; i < slides; i++) {
    float theta = 2 * i * PI / slides;
    float theta2 = 2 * (i + 1) * PI / slides;
    glVertex3f(top.X + r * vec2[0] * cos(theta) + r * vec3[0] * sin(theta),
	     top.Y + r * vec2[1] * cos(theta) + r * vec3[1] * sin(theta),
	     top.Z + r * vec2[2] * cos(theta) + r * vec3[2] * sin(theta));
    glVertex3f(top.X + r * vec2[0] * cos(theta2) + r * vec3[0] * sin(theta2),
	     top.Y + r * vec2[1] * cos(theta2) + r * vec3[1] * sin(theta2),
	     top.Z + r * vec2[2] * cos(theta2) + r * vec3[2] * sin(theta2));
    glVertex3f(bottom.X + r * vec2[0] * cos(theta2) + r * vec3[0] * sin(theta2),
	     bottom.Y + r * vec2[1] * cos(theta2) + r * vec3[1] * sin(theta2),
	     bottom.Z + r * vec2[2] * cos(theta2) + r * vec3[2] * sin(theta2));
    glVertex3f(bottom.X + r * vec2[0] * cos(theta) + r * vec3[0] * sin(theta),
	     bottom.Y + r * vec2[1] * cos(theta) + r * vec3[1] * sin(theta),
	     bottom.Z + r * vec2[2] * cos(theta) + r * vec3[2] * sin(theta));
  }
  glEnd();

}

/*
 表示用関数
 */
void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	int depth = 50;

/*
	CameraSpacePoint p;
	p.X = 0;
	p.Y = 0;
	p.Z = -0.2;
	drawSphere(p, 0.5);
	*/
	glBindTexture(GL_TEXTURE_2D, texId);
	for(int i = 0; i < depthHeight - 1; i++) {
		for(int j = 0; j < depthWidth - 1; j++) {
			int index = i * depthWidth + j;
			int index1 = index + 1;
			int indexw = index + depthWidth;
			int indexw1 = index + depthWidth + 1;
			int d1 = depthBuffer[index] - depthBuffer[index1];
			int d2 = depthBuffer[index1] - depthBuffer[indexw1];
			int d3 = depthBuffer[indexw1] - depthBuffer[indexw];
			if(d1 < 0) d1 = -d1;
			if(d2 < 0) d2 = -d2;
			if(d3 < 0) d3 = -d3;
			ColorSpacePoint p0 = pColorSpacePoint[index];
			ColorSpacePoint p1 = pColorSpacePoint[index1];
			ColorSpacePoint p2 = pColorSpacePoint[indexw];
			ColorSpacePoint p3 = pColorSpacePoint[indexw1];

			if(depthBuffer[index] > 0 && depthBuffer[index1] > 0
				&& depthBuffer[indexw] > 0 && depthBuffer[indexw1] > 0
				&& p0.X >= 0 && p0.X < colorWidth && p0.Y >= 0 && p0.Y < colorHeight
				&& p1.X >= 0 && p1.X < colorWidth && p1.Y >= 0 && p1.Y < colorHeight
				&& p2.X >= 0 && p2.X < colorWidth && p2.Y >= 0 && p2.Y < colorHeight
				&& p3.X >= 0 && p3.X < colorWidth && p3.Y >= 0 && p3.Y < colorHeight
				&& d1 < depth && d2 < depth && d3 < depth) {
					glBegin(GL_QUADS);
					glTexCoord2f(p0.X / colorWidth, p0.Y / colorHeight);
					glVertex3f(pCameraSpacePoint[index].X, pCameraSpacePoint[index].Y, pCameraSpacePoint[index].Z);
					glTexCoord2f(p1.X / colorWidth, p1.Y / colorHeight);
					glVertex3f(pCameraSpacePoint[index1].X, pCameraSpacePoint[index1].Y, pCameraSpacePoint[index1].Z);
					glTexCoord2f(p3.X / colorWidth, p3.Y / colorHeight);
					glVertex3f(pCameraSpacePoint[indexw1].X, pCameraSpacePoint[indexw1].Y, pCameraSpacePoint[indexw1].Z);
					glTexCoord2f(p2.X / colorWidth, p2.Y / colorHeight);
					glVertex3f(pCameraSpacePoint[indexw].X, pCameraSpacePoint[indexw].Y, pCameraSpacePoint[indexw].Z);
					glEnd();
			}
		}
	}

	Joint pJoint[JointType_Count];
	glBindTexture(GL_TEXTURE_2D, 0);

	for(int i = 0; i < BODY_COUNT; i++) {
		boolean tracked;
		if(pBody[i] != nullptr) {
			pBody[i]->get_IsTracked(&tracked);
			if(tracked) {
				pBody[i]->GetJoints(JointType_Count, pJoint);
				Joint head = pJoint[JointType_Head];
				Joint rh = pJoint[JointType_HandLeft];
				Joint lh = pJoint[JointType_HandRight];
				glColor3f(1, 0, 0);
				drawCylinder(rh.Position, lh.Position, 0.2, 3);	
				glColor3f(0,0,0);
				drawSphere(head.Position, 0.3);
				/*
				cout << "(" << head.Position.X << ", " << head.Position.Y << ", " 
					<< head.Position.Z << ")\n";
					*/
			}
		}
	}
	glColor3f(1, 1, 1);

	glutSwapBuffers();
}

/*
 アイドリング関数
 */
void idle() {
	// Kinect からの返却値を保存する変数
	HRESULT hResult = S_OK;

	// イベント待ちをする配列
	HANDLE hEvents[] = { reinterpret_cast<HANDLE>(waitableHandle) };

	// Kinect が画面を取得するまで、永遠に待つ
	WaitForMultipleObjects(1, hEvents, true, INFINITE);

	IMultiSourceFrame* pMultiSourceFrame = nullptr;
	// Frame
	hResult = pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);
	if(SUCCEEDED(hResult)) {
		IColorFrameReference* pColorFrameReference = nullptr;
		hResult = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		if(SUCCEEDED(hResult)) {
			IColorFrame* pColorFrame = nullptr;
			hResult = pColorFrameReference->AcquireFrame(&pColorFrame);
			if (SUCCEEDED(hResult)){
				// RGB情報をRGBAフォーマットに変換して、colorBuffer にコピー
				hResult = pColorFrame->CopyConvertedFrameDataToArray(colorWidth * colorHeight * 4, colorBuffer, 
					ColorImageFormat::ColorImageFormat_Rgba);
				if (SUCCEEDED(hResult)){
					// texId をテクスチャ番号に指定して、RGB画像をテクスチャに設定する
					glBindTexture(GL_TEXTURE_2D, texId);
					glTexImage2D(GL_TEXTURE_2D, 0, 4, colorWidth, colorHeight, 0,
						GL_RGBA, GL_UNSIGNED_BYTE, colorBuffer);
				}
			}
			SafeRelease(pColorFrame);
		}
		SafeRelease(pColorFrameReference);

		IDepthFrameReference* pDepthFrameReference = nullptr;
		hResult = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if(SUCCEEDED(hResult)) {
			IDepthFrame* pDepthFrame = nullptr;
			hResult = pDepthFrameReference->AcquireFrame(&pDepthFrame);
			if (SUCCEEDED(hResult)){
				// 深度情報をコピー
				hResult = pDepthFrame->CopyFrameDataToArray(depthWidth * depthHeight, depthBuffer); 
				if (SUCCEEDED(hResult)){
					hResult = pCoordinateMapper->MapDepthFrameToCameraSpace(depthWidth * depthHeight, 
						depthBuffer, depthWidth * depthHeight, pCameraSpacePoint);		
					if(SUCCEEDED(hResult)) {
					} else {
						cout << "error\n";
					}

					hResult = pCoordinateMapper->MapDepthFrameToColorSpace(depthWidth * depthHeight,
						depthBuffer, depthWidth * depthHeight, pColorSpacePoint);
				}
			}
			SafeRelease(pDepthFrame);
		}
		SafeRelease(pDepthFrameReference);

		IBodyFrameReference* pBodyFrameReference = nullptr;
		hResult = pMultiSourceFrame->get_BodyFrameReference(&pBodyFrameReference);

		if(SUCCEEDED(hResult)) {
			IBodyFrame* pBodyFrame = nullptr;
			hResult = pBodyFrameReference->AcquireFrame(&pBodyFrame);
			if (SUCCEEDED(hResult)){
				// 前回まで確保してあったものを遅れて解放
				for(int i = 0; i < BODY_COUNT; i++) {
					SafeRelease(pBody[i]);
				}	
				// Body 情報をコピー
				hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
					
				if (SUCCEEDED(hResult)){
				} else {
					cout << "error\n";
				}
			}
			SafeRelease(pBodyFrame);
		}
		SafeRelease(pBodyFrameReference);
	}
	SafeRelease(pMultiSourceFrame);

	glutPostRedisplay();
}

float theta = 0;
float fai = 0;
float step = 0.1;
int lastx;
int lasty;

void move(int x, int y) {
	int fx = x - lastx;
	int fy = y - lasty;
	lastx = x;
	lasty = y;

	if(fx > fy && fx > -fy) fai += step;
	else if(fx < fy && fx < -fy) fai -= step;
	else if(fx > fy && -fx > fy) theta -= step;
	else theta += step;
	
	glLoadIdentity();
	gluLookAt(1 * sin(fai), 1 * sin(theta), 1 - 1 * cos(theta) * cos(fai), 0, 0, 1, 0, cos(theta), 0);
	glutPostRedisplay();
}

/*
void mouse(int button, int state, int x, int y) {
	theta += 0.1;
	glLoadIdentity();
	gluLookAt(0, 2 * sin(theta), 2 - 2 * cos(theta), 0, 0, 2, 0, cos(theta), 0);
	glutPostRedisplay();
}
*/


void openGLInit(int* argc, char** argv) {
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutCreateWindow("KinectDepth");
	glutDisplayFunc(display);
	glutIdleFunc(idle);
//	glutMouseFunc(mouse);
	glutMotionFunc(move);
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
	// glBindTexture(GL_TEXTURE_2D, 0);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float n = 0.15;
	glFrustum(-screenRatio * n, screenRatio * n, -n, n, n * 2, 8.0);
	glMatrixMode(GL_MODELVIEW);
	gluLookAt(0, 0, 0, 0, 0, 1, 0, 1, 0);
}

/* 
 * kinect の初期化関数
 */
int kinectInit() {
	// 標準の Kinect Sensor を取得
	ERROR_CHECK(GetDefaultKinectSensor(&pKinect));

	if (pKinect) {
		IDepthFrameSource* pDepthFrameSource = nullptr;
		IFrameDescription* pDepthFrameDescription = nullptr;
		IColorFrameSource* pColorFrameSource = nullptr;
		IFrameDescription* pColorFrameDescription = nullptr;
		ERROR_CHECK(pKinect->Open());

		// 以下では、3つの事をまとめて実施している
		// (1) DepthFrameReader の取得
		//   Kinect から DepthFrameSource を取得し
		//   DepthFrameSource から DepthFrameReader を取得
		// (2) 画面の幅と高さを取得
		//   DepthFrameSource から DepthFrameDescription を取得し
		//   DepthFrameDescription から get_Height(), get_Width()を呼ぶ

		// (1)
		ERROR_CHECK(pKinect->get_DepthFrameSource(&pDepthFrameSource));

		// (2)
		ERROR_CHECK(pDepthFrameSource->get_FrameDescription(&pDepthFrameDescription));
		pDepthFrameDescription->get_Height(&depthHeight);
		pDepthFrameDescription->get_Width(&depthWidth);

		// (1)
		ERROR_CHECK(pKinect->get_ColorFrameSource(&pColorFrameSource));

		// (2)
		ERROR_CHECK(pColorFrameSource->get_FrameDescription(&pColorFrameDescription));
		pColorFrameDescription->get_Height(&colorHeight);
		pColorFrameDescription->get_Width(&colorWidth);

		screenRatio = (float)WIDTH / (float)HEIGHT;

		// MultiSourceFrameReader をオープン
		ERROR_CHECK(pKinect->OpenMultiSourceFrameReader(FrameSourceTypes_Color | FrameSourceTypes_Depth
			| FrameSourceTypes_Body, &pMultiSourceFrameReader));
		pMultiSourceFrameReader->SubscribeMultiSourceFrameArrived(&waitableHandle);

		// カラー画面を保存する配列を確保
		depthBuffer = new UINT16[depthHeight * depthWidth];
		colorBuffer = new unsigned char[colorHeight * colorWidth * 4];
		pCameraSpacePoint = new CameraSpacePoint[depthHeight * depthWidth];
		pColorSpacePoint = new ColorSpacePoint[depthHeight * depthWidth];
		pBody = new IBody* [BODY_COUNT];
		for(int i = 0; i < BODY_COUNT; i++) {
			pBody[i] = nullptr;
		}

		// 途中で利用して不要になった DepthFrameSource とDepthFrameDescription を開放
		SafeRelease(pDepthFrameSource);
		SafeRelease(pDepthFrameDescription);
		SafeRelease(pColorFrameSource);
		SafeRelease(pColorFrameDescription);

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
