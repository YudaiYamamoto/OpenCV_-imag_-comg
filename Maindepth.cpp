#include <cstdlib>
#include <GL/glut.h>
#include <Kinect.h>
#include <iostream>
#include <sstream>

using namespace std;

// �X�N���[���̕��ƍ���
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

	// �����`���e�N�X�`���ŕ`��
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
 �A�C�h�����O�֐�
 */
void idle() {
	// Kinect ����̕ԋp�l��ۑ�����ϐ�
	HRESULT hResult = S_OK;

	// �C�x���g�҂�������z��
	HANDLE hEvents[] = { reinterpret_cast<HANDLE>(hColorWaitable) };

	// Kinect ����ʂ��擾����܂ŁA�i���ɑ҂�
	WaitForMultipleObjects(1, hEvents, true, INFINITE);

	// Frame
	IColorFrame* pColorFrame = nullptr;
	hResult = pColorFrameReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(hResult)){
		// RGB����RGBA�t�H�[�}�b�g�ɕϊ����āAcolorBuffer �ɃR�s�[
		hResult = pColorFrame->CopyConvertedFrameDataToArray(colorWidth * colorHeight * 4, colorBuffer, 
			ColorImageFormat::ColorImageFormat_Rgba);
		if (SUCCEEDED(hResult)){
			// texId ���e�N�X�`���ԍ��Ɏw�肵�āARGB�摜���e�N�X�`���ɐݒ肷��
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

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-screenRatio, screenRatio, -1.0, 1.0, 1.0, 4.0);
	glMatrixMode(GL_MODELVIEW);
	gluLookAt(0, 0, 3, 0, 0, 0, 0, 1, 0);
}

/* 
 * kinect �̏������֐�
 */
int kinectInit() {
	// �W���� Kinect Sensor ���擾
	ERROR_CHECK(GetDefaultKinectSensor(&pKinect));

	if (pKinect) {
		IColorFrameSource* pColorFrameSource;
		IFrameDescription* pColorFrameDescription;

		ERROR_CHECK(pKinect->Open());

		// �ȉ��ł́A3�̎����܂Ƃ߂Ď��{���Ă���
		// (1) ColorFrameReader �̎擾
		//   Kinect ���� ColorFrameSource ���擾��
		//   ColorFrameSource ���� ColorFrameReader ���擾
		// (2) Reader �̃C�x���g�����҂��̃n���h���̐ݒ�
		//   ColorFrameReader �ɁACoolorWaitable ��ݒ�
		// (3) ��ʂ̕��ƍ������擾
		//   ColorFrameSource ���� ColorFrameDescription ���擾��
		//   ColorFrameDescription ���� get_Height(), get_Width()���Ă�

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

		// �J���[��ʂ�ۑ�����z����m��
		colorBuffer = new unsigned char[colorHeight * colorWidth * 4];

		// �r���ŗ��p���ĕs�v�ɂȂ��� ColorFrameSource ��ColorFrameDescription ���J��
		SafeRelease(pColorFrameSource);
		SafeRelease(pColorFrameDescription);

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