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
 Kinect �֐��̕ԋp�l�̃G���[����
*/
void ERROR_CHECK(int ret) {
	if (FAILED(ret)) {
		stringstream ss;
		ss << "failed " << ret << " " << hex << ret << endl;
		throw std::runtime_error(ss.str().c_str());
	}
}

/*
 �������̈��S�ȉ��
*/
template<class Interface>
void SafeRelease(Interface*& pInterfaceToRelease) {
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

/*
 �\���p�֐�
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
 �A�C�h�����O�֐�
 */
void idle() {
	// Kinect ����̕ԋp�l��ۑ�����ϐ�
	HRESULT hResult = S_OK;

	// �C�x���g�҂�������z��
	HANDLE hEvents[] = { reinterpret_cast<HANDLE>(hDepthWaitable) };

	// Kinect ����ʂ��擾����܂ŁA�i���ɑ҂�
	WaitForMultipleObjects(1, hEvents, true, INFINITE);

	// Frame
	IDepthFrame* pDepthFrame = nullptr;
	hResult = pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	// hResult = pDepthReference->AcquireFrame(&pDepthFrame);
	if (SUCCEEDED(hResult)){
		// RGB����RGBA�t�H�[�}�b�g�ɕϊ�
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
	glEnable(GL_DEPTH_TEST); // �[�x�p�ɒǉ�

	// �e�N�X�`���̏����ݒ�
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
 * kinect �̏������֐�
 */
int kinectInit() {
	// �W���� Kinect Sensor ���擾
	ERROR_CHECK(GetDefaultKinectSensor(&pKinect));

	if (pKinect) {
		IDepthFrameSource* pDepthFrameSource;
		IFrameDescription* pDepthFrameDescription;
		ERROR_CHECK(pKinect->Open());

		// �ȉ��ł́A3�̎����܂Ƃ߂Ď��{���Ă���
		// (1) DepthFrameReader �̎擾
		//   Kinect ���� DepthFrameSource ���擾��
		//   DepthFrameSource ���� DepthFrameReader ���擾
		// (2) Reader �̃C�x���g�����҂��̃n���h���̐ݒ�
		//   DepthFrameReader �ɁACoolorWaitable ��ݒ�
		// (3) ��ʂ̕��ƍ������擾
		//   DepthFrameSource ���� DepthFrameDescription ���擾��
		//   DepthFrameDescription ���� get_Height(), get_Width()���Ă�

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

		// �J���[��ʂ�ۑ�����z����m��
		depthBuffer = new UINT16[depthHeight * depthWidth];
		colorBuffer = new unsigned char[depthHeight * depthWidth * 4];
		pCameraSpacePoint = new CameraSpacePoint[depthHeight * depthWidth];

		// �r���ŗ��p���ĕs�v�ɂȂ��� DepthFrameSource ��DepthFrameDescription ���J��
		SafeRelease(pDepthFrameSource);
		SafeRelease(pDepthFrameDescription);

		ERROR_CHECK(pKinect->get_CoordinateMapper(&pCoordinateMapper));

		return 1;
	}
	else {
		cout << "Kinect ���ڑ�����Ă��܂���B\n";
		return 0;
	}
}

int main(int argc, char **argv) {
	// Kinect �̏�����
	if (kinectInit() == 0) {
		exit(1);
	}	
	
	// OpenGL �̏�����
	openGLInit(&argc, argv);

	// OpenGL �̃��C�����[�v
	glutMainLoop();
}
