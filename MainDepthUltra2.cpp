#include <cstdlib>
#include <GL/glut.h>
#include <Kinect.h>
#include <iostream>
#include <sstream>

using namespace std;

#define HEIGHT 540
#define WIDTH 960
#define GAP 1

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

// ����`��
void drawSphere(CameraSpacePoint pos, float r) {
	glBindTexture(GL_TEXTURE_2D, 0);
	glPushMatrix();
	// glLoadIdentity();
	glTranslatef(pos.X, pos.Y, pos.Z);
	// glColor3f(0, 0, 1);
	glutSolidSphere(r, 20, 20);
	// glutSolidTeapot(r);
	// glTranslatef(pos.X, pos.Y, pos.Z);
	// glColor3f(1, 1, 1);
	glPopMatrix();
}

#define PI 3.141592

// �O��
void cross(float* x, float* y, float* result) {
  result[0] = x[1] * y[2] - x[2] * y[1];
  result[1] = x[2] * y[0] - x[0] * y[2];
  result[2] = x[0] * y[1] - x[1] * y[0];
}

// �O��
void cross(CameraSpacePoint* xp, CameraSpacePoint* yp, CameraSpacePoint* rp) {
  rp->X = xp->Y * yp->Z - xp->Z * yp->Y;
  rp->Y = xp->Z * yp->X - xp->X * yp->Z;
  rp->Z = xp->X * yp->Y - xp->Y * yp->X;
}

// ����
float dot(float* x, float* y) {
  return x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
}

// ����
float dot(CameraSpacePoint* xp, CameraSpacePoint* yp) {
  return xp->X * yp->X + xp->Y * yp->Y + xp->Z * yp->Z;
}


// ���K��
void norm(float* x, float* result) {
  float length = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
  result[0] = x[0] / length;
  result[1] = x[1] / length;
  result[2] = x[2] / length;
}

// ���K��
void norm(CameraSpacePoint* xp, CameraSpacePoint* rp) {
  float length = sqrt(xp->X * xp->X + xp->Y * xp->Y + xp->Z * xp->Z);
  rp->X = xp->X / length;
  rp->Y = xp->Y / length;
  rp->Z = xp->Z / length;
}

// ����
float distance(CameraSpacePoint* xp, CameraSpacePoint* yp) {
  return sqrt((xp->X - yp->X) * (xp->X - yp->X) 
	      + (xp->Y - yp->Y) * (xp->Y - yp->Y) 
	      + (xp->Z - yp->Z) * (xp->Z - yp->Z));
}

// ����
float distance(float* xp, float* yp) {
  return sqrt((xp[0] - yp[0]) * (xp[0] - yp[0]) 
	      + (xp[1] - yp[1]) * (xp[1] - yp[1]) 
	      + (xp[2] - yp[2]) * (xp[2] - yp[2]));
}

// �����x�N�g��
// �@xp:�N�_, yp:�I�_, dirp: �����x�N�g��
void direction(CameraSpacePoint* xp, CameraSpacePoint* yp, float* dirp) {
  dirp[0] = yp->X - xp->X;
  dirp[1] = yp->Y - xp->Y;
  dirp[2] = yp->Z - xp->Z;
}

// ����
float length(float* xp) {
  return sqrt(xp[0] * xp[0] + xp[1] * xp[1] + xp[2] * xp[2]);
}

// ����
float length(CameraSpacePoint* xp) {
  return sqrt(xp->X * xp->X + xp->Y * xp->Y + xp->Z * xp->Z);
}

// �R�T�C��
float cosine(float* xp, float* yp) {
  return dot(xp, yp) / length(xp) / length(yp);
}

// �R�T�C��
float cosine(CameraSpacePoint* xp, CameraSpacePoint* yp) {
  return dot(xp, yp) / length(xp) / length(yp);
}

// ���S���W
void center(CameraSpacePoint* xp, CameraSpacePoint* yp, CameraSpacePoint* cp) {
  cp->X = (xp->X + yp->X) / 2;
  cp->Y = (xp->Y + yp->Y) / 2;
  cp->Z = (xp->Z + yp->Z) / 2;
}

// CameraSpacePoint ���� float* �ւ̕ϊ�
void pointToArray(CameraSpacePoint* p, float* fp) {
  fp[0] = p->X;
  fp[1] = p->Y;
  fp[2] = p->Z;
}

// float* ���� CameraSpacePoint �ւ̕ϊ�
void arrayToPoint(float* fp, CameraSpacePoint* p) {
  p->X = fp[0];
  p->Y = fp[1];
  p->Z = fp[2];
}

void kamehameha(Joint* pJoint, int id) {
  // �C�̋��� 0-299, ���[�U���ƂɌv��
  static int power[7] = {0};
  Joint handRight = pJoint[JointType_HandRight];
  Joint handLeft = pJoint[JointType_HandLeft];
  CameraSpacePoint right = handRight.Position;
  CameraSpacePoint left = handLeft.Position;

  // �E��ƍ���̋���
  float dist = distance(&right, &left);

  // ������10cm�ȉ��̎��ɂ́A�C�𗭂߂�
  if(dist < 0.1) {
    power[id]++;
    if(power[id] > 300) {
      power[id] = 300;
    }
  } else {
    power[id]--;
    if(power[id] < 0) {
      power[id] = 0;
    }
  }

  // �C�����܂�n�߂�
  if(power[id] > 30) {
    CameraSpacePoint centerPoint;
    center(&right, &left, &centerPoint);
    glColor3f(1, 1, 0.5);
    drawSphere(centerPoint, power[id] / 3000.0);

    // �������ʕ���
    glColor3f(1, 1, 1);
    glBegin(GL_LINES);
    for(int i = 0; i < 100; i++) {
      // �����̒���
      float len = rand() % power[id] / 1500.0;
      // �����ŁA�l�X�ȕ����Ɍ������
      float vec[3];
      vec[0] = (rand() % 200 - 100) / 100.0;
      vec[1] = (rand() % 200 - 100) / 100.0;
      vec[2] = (rand() % 200 - 100) / 100.0;
      norm(vec, vec);
      glVertex3f(centerPoint.X, centerPoint.Y, centerPoint.Z);
      glVertex3f(centerPoint.X + vec[0] * len, 
		 centerPoint.Y + vec[1] * len,
		 centerPoint.Z + vec[2] * len);
    }
    glEnd();

    // �C���\�����܂���
    if(power[id] > 500) {
      CameraSpacePoint elbow = pJoint[JointType_ElbowRight].Position;
      CameraSpacePoint shoulder = pJoint[JointType_ShoulderRight].Position;
      float dir0[3];
      float dir1[3];

      // �I���L�тĂ��邩�ǂ������v�Z���邽�߂̃x�N�g�����擾
      direction(&elbow, &right, dir0);
      direction(&shoulder, &elbow, dir1);

      // �I���L�тĂ���ƃJ���n���g�����
      if(cosine(dir0, dir1) > 0.3) {
	glBegin(GL_LINES);
	for(int i = 0; i < 100; i++) {
	  float len = rand() % power[id] / 30.0;
	  float vec[3];
	  float wide = 0.1;
	  vec[0] = (rand() % 200 - 100) / 100.0;
	  vec[1] = (rand() % 200 - 100) / 100.0;
	  vec[2] = (rand() % 200 - 100) / 100.0;
	  norm(vec, vec);
	  glVertex3f(centerPoint.X, centerPoint.Y, centerPoint.Z);
	  glVertex3f(centerPoint.X + dir0[0] * len + vec[0] * wide, 
		     centerPoint.Y + dir0[1] * len + vec[1] * wide,
		     centerPoint.Z + dir0[2] * len + vec[2] * wide);
	}
	glEnd();
      }
    }
  }
}

// �X�y�V�E������
void specium2(Joint* pJoint, int id) {
  CameraSpacePoint right = pJoint[JointType_HandRight].Position;
  CameraSpacePoint left = pJoint[JointType_HandLeft].Position;
  CameraSpacePoint eright = pJoint[JointType_ElbowRight].Position;
  CameraSpacePoint sright = pJoint[JointType_ShoulderRight].Position;

  // �E�I�ƍ���Ɠ��̋���
  float dist0 = distance(&eright, &left);

  float dir[3], dir2[3];
  direction(&sright, &eright, dir);
  direction(&eright, &right, dir2);

  // ������20cm�ȉ��̎��ɂ́A�����𔭎�
  float th = 0.20;
  if(dist0 < th && cosine(dir, dir2) < 0.2) {
    float dir[3]; // �����̌���
    direction(&sright, &eright, dir);
    norm(dir, dir);

    // �������
    glColor3f(0.8, 0.8, 1.0);
    glBegin(GL_LINES);
    for(int i = 0; i < 100; i++) {
      // �����̒���
      float len = rand() % 300 / 100.0;
      float wide = 0.03;
      // �����ŁA�����������̕��������炷
      float vec[3];
      vec[0] = (rand() % 200 - 100) / 100.0;
      vec[1] = (rand() % 200 - 100) / 100.0;
      vec[2] = (rand() % 200 - 100) / 100.0;
      norm(vec, vec);
      float ratio = (rand() % 1000) / 1000.0;
      float x = ratio * right.X + (1 - ratio) * eright.X;
      float y = ratio * right.Y + (1 - ratio) * eright.Y;
      float z = ratio * right.Z + (1 - ratio) * eright.Z;
      glVertex3f(x, y, z);
      glVertex3f(x + dir[0] * len + vec[0] * wide, 
		 y + dir[1] * len + vec[1] * wide, 
		 z + dir[2] * len + vec[2] * wide);
    }
    glEnd();
  }
}
// �X�y�V�E������newspa
void specium(Joint* pJoint, int id) {
	CameraSpacePoint right = pJoint[JointType_HandRight].Position;
	CameraSpacePoint left = pJoint[JointType_HandLeft].Position;
	CameraSpacePoint eright = pJoint[JointType_ElbowRight].Position;
	CameraSpacePoint sright = pJoint[JointType_ShoulderRight].Position;

	// �E�I�ƍ���Ɠ��̋���
	float dist0 = distance(&eright, &left);

	float dir[3], dir2[3];
	direction(&sright, &eright, dir);
	direction(&eright, &right, dir2);

	// ������20cm�ȉ��̎��ɂ́A�����𔭎�
	float th = 0.20;
	if (dist0 < th && cosine(dir, dir2) < 0.2) {
		float dir[3]; // �����̌���
		direction(&sright, &eright, dir);
		norm(dir, dir);

		// �������
		glColor3f(0.8, 0.8, 1.0);
		glBegin(GL_LINES);
		for (int i = 0; i < 100; i++) {
			// �����̒���
			float len = rand() % 300 / 100.0;
			float wide = 0.03;
			// �����ŁA�����������̕��������炷
			float vec[3];
			vec[0] = (rand() % 200 - 100) / 100.0;
			vec[1] = (rand() % 200 - 100) / 100.0;
			vec[2] = (rand() % 200 - 100) / 100.0;
			norm(vec, vec);
			float ratio = (rand() % 1000) / 1000.0;
			float x = ratio * right.X + (1 - ratio) * eright.X;
			float y = ratio * right.Y + (1 - ratio) * eright.Y;
			float z = ratio * right.Z + (1 - ratio) * eright.Z;
			glVertex3f(x, y, z);
			glVertex3f(x + dir[0] * len + vec[0] * wide,
				y + dir[1] * len + vec[1] * wide,
				z + dir[2] * len + vec[2] * wide);
		}
		glEnd();
	}
}

// �G�����E������
void emerium(Joint* pJoint, int id) {
  CameraSpacePoint right = pJoint[JointType_HandRight].Position;
  CameraSpacePoint left = pJoint[JointType_HandLeft].Position;
  CameraSpacePoint eright = pJoint[JointType_ElbowRight].Position;
  CameraSpacePoint eleft = pJoint[JointType_ElbowLeft].Position;
  CameraSpacePoint head = pJoint[JointType_Head].Position;

  // �E��ƍ���Ɠ��̋���
  float dist0 = distance(&right, &left);
  float dist1 = distance(&right, &head);
  float dist2 = distance(&left, &head);

  // ������30---0cm�ȉ��̎��ɂ́A�����𔭎�
  float th = 0.00;
  if(dist0 < th && dist1 < th && dist2 < th) {
    float dir0[3];
    float dir1[3];
    float dir[3]; // �����̌���
    direction(&right, &eright, dir0);
	dir[2] -= 0.1;
    direction(&left, &eleft, dir1);
    cross(dir0, dir1, dir);
    norm(dir, dir);

    // �������
    glColor3f(0.7, 1.0, 0.7);
    glBegin(GL_LINES);
    for(int i = 0; i < 100; i++) {
      float len = rand() % 300 / 100.0;
      float wide = 0.05;
      float vec[3];
      vec[0] = (rand() % 200 - 100) / 100.0;
      vec[1] = (rand() % 200 - 100) / 100.0;
      vec[2] = (rand() % 200 - 100) / 100.0;
      norm(vec, vec);
      glVertex3f(head.X, head.Y, head.Z);
      glVertex3f(head.X + dir[0] * len + vec[0] * wide, 
		 head.Y + dir[1] * len + vec[1] * wide, 
		 head.Z + dir[2] * len + vec[2] * wide);
    }
    glEnd();
  }
}

void drawStar(CameraSpacePoint centerPoint, float size)
{
    // �������ʕ���
    glBegin(GL_LINES);
    for(int i = 0; i < 100; i++) {
      // �����̒���
      float len = rand() % 1000 / 1000.0 * size;
      // �����ŁA�l�X�ȕ����Ɍ������
      float vec[3];
      vec[0] = (rand() % 200 - 100) / 100.0;
      vec[1] = (rand() % 200 - 100) / 100.0;
      vec[2] = (rand() % 200 - 100) / 100.0;
      norm(vec, vec);
      glVertex3f(centerPoint.X, centerPoint.Y, centerPoint.Z);
      glVertex3f(centerPoint.X + vec[0] * len, 
		 centerPoint.Y + vec[1] * len,
		 centerPoint.Z + vec[2] * len);
    }
    glEnd();
}

// �~���̕`��
void drawCylinder(CameraSpacePoint top, CameraSpacePoint bottom, float r, 
		  int slices) {
  // direction
  float vec[3];
  vec[0] = top.X - bottom.X;
  vec[1] = top.Y - bottom.Y;
  vec[2] = top.Z - bottom.Z;

  float mx = abs(vec[0]);
  int index = 0;
  if(mx < abs(vec[1])) {
    mx = abs(vec[1]);
    index = 1;
  }
  if(mx < abs(vec[2])) {
    mx = abs(vec[2]);
    index = 2;
  }

  // ��
  float vec2[3] = {0};
  if(index == 0) {
    vec2[1] = 1;
  } else {
    vec2[0] = 1;
  }

  // �c
  float vec3[3];
  cross(vec, vec2, vec3);
  norm(vec3, vec3);

  cross(vec, vec3, vec2);
  norm(vec2, vec2);

  // ���
  glBegin(GL_POLYGON);
  for(int i = 0; i < slices; i++) {
    float theta = 2 * i * PI / slices;
    glVertex3f(top.X + r * vec2[0] * cos(theta) + r * vec3[0] * sin(theta),
	     top.Y + r * vec2[1] * cos(theta) + r * vec3[1] * sin(theta),
	     top.Z + r * vec2[2] * cos(theta) + r * vec3[2] * sin(theta));
  }
  glEnd();

  // ���
  glBegin(GL_POLYGON);
  for(int i = 0; i < slices; i++) {
    float theta = 2 * i * PI / slices;
    glVertex3f(bottom.X + r * vec2[0] * cos(theta) + r * vec3[0] * sin(theta),
	     bottom.Y + r * vec2[1] * cos(theta) + r * vec3[1] * sin(theta),
	     bottom.Z + r * vec2[2] * cos(theta) + r * vec3[2] * sin(theta));
  }
  glEnd();

  // ����
  glBegin(GL_QUADS);
  for(int i = 0; i < slices; i++) {
    float theta = 2 * i * PI / slices;
    float theta2 = 2 * (i + 1) * PI / slices;
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

// �~���̕`��
void drawCone(CameraSpacePoint top, CameraSpacePoint bottom, float r, 
		  int slices) {
  // direction
  float vec[3];
  vec[0] = top.X - bottom.X;
  vec[1] = top.Y - bottom.Y;
  vec[2] = top.Z - bottom.Z;

  float mx = abs(vec[0]);
  int index = 0;
  if(mx < abs(vec[1])) {
    mx = abs(vec[1]);
    index = 1;
  }
  if(mx < abs(vec[2])) {
    mx = abs(vec[2]);
    index = 2;
  }

  // ��
  float vec2[3] = {0};
  if(index == 0) {
    vec2[1] = 1;
  } else {
    vec2[0] = 1;
  }

  // �c
  float vec3[3];
  cross(vec, vec2, vec3);
  norm(vec3, vec3);

  cross(vec, vec3, vec2);
  norm(vec2, vec2);

  // ���
  glBegin(GL_POLYGON);
  for(int i = 0; i < slices; i++) {
    float theta = 2 * i * PI / slices;
    glVertex3f(bottom.X + r * vec2[0] * cos(theta) + r * vec3[0] * sin(theta),
	     bottom.Y + r * vec2[1] * cos(theta) + r * vec3[1] * sin(theta),
	     bottom.Z + r * vec2[2] * cos(theta) + r * vec3[2] * sin(theta));
  }
  glEnd();

  // ����
  glBegin(GL_QUADS);
  for(int i = 0; i < slices; i++) {
    float theta = 2 * i * PI / slices;
    float theta2 = 2 * (i + 1) * PI / slices;
    glVertex3f(top.X, top.Y, top.Z);
    glVertex3f(bottom.X + r * vec2[0] * cos(theta2) + r * vec3[0] * sin(theta2),
	     bottom.Y + r * vec2[1] * cos(theta2) + r * vec3[1] * sin(theta2),
	     bottom.Z + r * vec2[2] * cos(theta2) + r * vec3[2] * sin(theta2));
    glVertex3f(bottom.X + r * vec2[0] * cos(theta) + r * vec3[0] * sin(theta),
	     bottom.Y + r * vec2[1] * cos(theta) + r * vec3[1] * sin(theta),
	     bottom.Z + r * vec2[2] * cos(theta) + r * vec3[2] * sin(theta));
  }
  glEnd();
}

void drawEgg(float x, float y, float z, float w, float h) {
	glPushMatrix();
	glTranslatef(x, y, z);
	glScalef(1, h / w, 1);
	glutSolidSphere(w, 40, 40);
	glPopMatrix();
}

void drawEgg(CameraSpacePoint p, float w, float h) {
  drawEgg(p.X, p.Y, p.Z, w, h);
}

void drawTorus(float x, float y, float z, float in, float out) {
	glPushMatrix();
	glTranslatef(x, y, z);
	glRotatef(90, 1, 0, 0);
	glutSolidTorus(in, out, 40, 40);
	glPopMatrix();
}

void drawTorus(CameraSpacePoint p, float in, float out) {
  drawTorus(p.X, p.Y, p.Z, in, out);
}

// �z��p�̑ȉ~���֐�
void drawEgg(float *fp, float w, float h) {
  drawEgg(fp[0], fp[1], fp[2], w, h);
}

// �z��p�̃g�[���X�`��֐�
void drawTorus(float *fp, float in, float out) {
  drawTorus(fp[0], fp[1], fp[2], in, out);
}

// �z��p�̐��`��֐�
void drawStar(float *fp, float size) {
  CameraSpacePoint p;
  arrayToPoint(fp, &p);
  drawStar(p, size);
}

// �z��p�̐��`��֐�
void drawSphere(float *fp, float r) {
  CameraSpacePoint p;
  arrayToPoint(fp, &p);
  drawSphere(p, r);
}

// �z��p�̐��`��֐�
void drawCone(float* top, float* bottom, float r, 
		  int slices) {
  CameraSpacePoint p1, p2;
  arrayToPoint(top, &p1);
  arrayToPoint(bottom, &p2);
  drawCone(p1, p2, r, slices);
}

void drawCylinder(float* top, float* bottom, float r, 
		  int slices) {
  CameraSpacePoint p1, p2;
  arrayToPoint(top, &p1);
  arrayToPoint(bottom, &p2);
  drawCylinder(p1, p2, r, slices);
}

void drawStars(Joint *pJoint) {
	const int num = 100;
	static float stars[num][3] = { 0 };
	static bool startFlag = true;
	CameraSpacePoint right = pJoint[JointType_HandRight].Position;
	CameraSpacePoint left = pJoint[JointType_HandLeft].Position;
	CameraSpacePoint head = pJoint[JointType_Head].Position;
	float dist0 = distance(&right, &left);
	float dist1 = distance(&right, &head);
	float dist2 = distance(&left, &head);
	cout << dist0 << "," << dist1 << "," << dist2 << endl;

	if (startFlag) {
		for (int i = 0; i < num; i++) {
			stars[i][0] = rand() % 4000 / 1000.0 - 2;
			stars[i][1] = rand() % 4000 / 1000.0 - 2;
			stars[i][2] = rand() % 2000 / 1000.0;
		}
		startFlag = false;
	}

	float move = 0.0;

	if (dist0 < 0.3 && dist1 > 0.3 && dist2 > 0.3){
		for (int i = 0; i < num; i++) {
			glColor3f(0.7, 0.7, 1);
			drawStar(stars[i], 0.05);
			stars[i][0] += (rand() % 2000 / 1000.0 - 1) * move;
			stars[i][1] += (rand() % 2000 / 1000.0 - 1) * move;
			stars[i][2] += (rand() % 2000 / 1000.0 - 1) * move;
			stars[i][3] += (rand() % 2000 / 1000.0 - 1) * move;
			stars[i][4] += (rand() % 2000 / 1000.0 - 1) * move;
			stars[i][5] += (rand() % 2000 / 1000.0 - 1) * move;
			stars[i][6] += (rand() % 2000 / 1000.0 - 1) * move;
			stars[i][7] += (rand() % 2000 / 1000.0 - 1) * move;
			stars[i][8] += (rand() % 2000 / 1000.0 - 1) * move;
			stars[i][9] += (rand() % 2000 / 1000.0 - 1) * move;
			stars[i][10] += (rand() % 2000 / 1000.0 - 1) * move;
		}
	}
}

void drawFireFly() {
  const int num = 100;
  static float fly[num][3] = {0};
  static bool startFlyFlag = true;

  if(startFlyFlag) {
    for(int i = 0; i < num; i++) {
      fly[i][0] = rand() % 4000 / 1000.0 - 2;
      fly[i][1] = rand() % 4000 / 1000.0 - 2;
      fly[i][2] = rand() % 2000 / 1000.0;
    }
    startFlyFlag = false;
  }

  float move = 0.03;

  for(int i = 0; i < num; i++) {
    glColor3f(1, 1, 0);
    drawStar(fly[i], 0.01);
    fly[i][0] += (rand() % 2000 / 1000.0 - 1) * move;
    fly[i][1] += (rand() % 2000 / 1000.0 - 1) * move;
    fly[i][2] += (rand() % 2000 / 1000.0 - 1) * move;
  }
}

void drawSnows(Joint *pJoint) {
  const int num = 1000;
  static float snows[num][3] = {0};
  static bool startSnowFlag = true;
  CameraSpacePoint right = pJoint[JointType_HandRight].Position;
  CameraSpacePoint left = pJoint[JointType_HandLeft].Position;
  CameraSpacePoint head = pJoint[JointType_Head].Position;
  float dist1 = distance(&right, &head);
  float dist2 = distance(&left, &head);


  if(startSnowFlag) {
    for(int i = 0; i < num; i++) {
      snows[i][0] = rand() % 4000 / 1000.0 - 2;
      snows[i][1] = rand() % 4000 / 1000.0 - 2;
      snows[i][2] = rand() % 2000 / 1000.0;

    }
    startSnowFlag = false;
  }
 
  if (dist1 < 0.3 && dist2 < 0.3){
	for (int i = 0; i < num; i++) {
		  glColor3f(1, 1, 1);
		  drawSphere(snows[i], 0.01);
		  snows[i][0] += (rand() % 2000 / 1000.0 - 1) * 0.02;
		  snows[i][1] += (rand() % 2000 / 1000.0 - 1) * 0.02 - 0.02;
		  if (snows[i][1] < -2.0) {
			  snows[i][1] = 2.0;
		  }
		  snows[i][2] += (rand() % 2000 / 1000.0 - 1) * 0.02;
	  }
  }
}

void drawCharacter(Joint *pJoint) {
	Joint head = pJoint[JointType_Head];
	Joint rh = pJoint[JointType_HandRight];
	Joint lh = pJoint[JointType_HandLeft];
	CameraSpacePoint p = rh.Position;
	CameraSpacePoint p2 = lh.Position;
				
	p2.X = p.X;
	p2.Y = p.Y + 0.5;
	p2.Z = p.Z;
	
	// ��������
	glColor3f(1, 1, 0);
	drawCone(p2, p, 0.2, 20);

	// ��
	glColor3f(1, 0, 0);
	drawSphere(head.Position, 0.3);

	// ����
	Joint spine = pJoint[JointType_SpineBase];
	glColor3f(0, 1, 0);
	drawSphere(spine.Position, 0.5);
}

/*
 �\���p�֐�
 */
void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	int depth = 50;

	glBindTexture(GL_TEXTURE_2D, texId);
	for (int i = 0; i < depthHeight - GAP; i += GAP) {
		for (int j = 0; j < depthWidth - GAP; j += GAP) {
			int index = i * depthWidth + j;
			int index1 = index + GAP;
			int indexw = index + depthWidth * GAP;
			int indexw1 = index + depthWidth * GAP + GAP;
			int d1 = depthBuffer[index] - depthBuffer[index1];
			int d2 = depthBuffer[index1] - depthBuffer[indexw1];
			int d3 = depthBuffer[indexw1] - depthBuffer[indexw];
			if (d1 < 0) d1 = -d1;
			if (d2 < 0) d2 = -d2;
			if (d3 < 0) d3 = -d3;
			ColorSpacePoint p0 = pColorSpacePoint[index];
			ColorSpacePoint p1 = pColorSpacePoint[index1];
			ColorSpacePoint p2 = pColorSpacePoint[indexw];
			ColorSpacePoint p3 = pColorSpacePoint[indexw1];

			if (depthBuffer[index] > 0 && depthBuffer[index1] > 0
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

	// �e�N�X�`�����[�h����������
	glBindTexture(GL_TEXTURE_2D, 0);

	Joint pJoint[JointType_Count];

	// ����`��
		

	// �z�^��
	//drawFireFly();

	// ���`��
    //drawSnows();

	for(int i = 0; i < BODY_COUNT; i++) {
		boolean tracked;
		if(pBody[i] != nullptr) {
			pBody[i]->get_IsTracked(&tracked);
			if(tracked) {
				pBody[i]->GetJoints(JointType_Count, pJoint);
				
				drawStars(pJoint);
				drawSnows(pJoint);
				//kamehameha(pJoint, i);
				//specium(pJoint, i);
				//emerium(pJoint, i);
				// drawCharacter(pJoint);
			}
		}
	}

	// ���̍s�������Ȃ�����
	glColor3f(1, 1, 1);

	glutSwapBuffers();
}

/*
 �A�C�h�����O�֐�
 */
void idle() {
	// Kinect ����̕ԋp�l��ۑ�����ϐ�
	HRESULT hResult = S_OK;

	// �C�x���g�҂�������z��
	HANDLE hEvents[] = { reinterpret_cast<HANDLE>(waitableHandle) };

	// Kinect ����ʂ��擾����܂ŁA�i���ɑ҂�
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
				// RGB����RGBA�t�H�[�}�b�g�ɕϊ����āAcolorBuffer �ɃR�s�[
				hResult = pColorFrame->CopyConvertedFrameDataToArray(colorWidth * colorHeight * 4, colorBuffer, 
					ColorImageFormat::ColorImageFormat_Rgba);
				if (SUCCEEDED(hResult)){
					// texId ���e�N�X�`���ԍ��Ɏw�肵�āARGB�摜���e�N�X�`���ɐݒ肷��
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
				// �[�x�����R�s�[
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
				// �O��܂Ŋm�ۂ��Ă��������̂�x��ĉ��
				for(int i = 0; i < BODY_COUNT; i++) {
					SafeRelease(pBody[i]);
				}	
				// Body �����R�s�[
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
	// glBindTexture(GL_TEXTURE_2D, 0);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float n = 0.15;
	glFrustum(-screenRatio * n, screenRatio * n, -n, n, n * 2, 8.0);
	glMatrixMode(GL_MODELVIEW);
	gluLookAt(0, 0, 0, 0, 0, 1, 0, 1, 0);
}

/* 
 * kinect �̏������֐�
 */
int kinectInit() {
	// �W���� Kinect Sensor ���擾
	ERROR_CHECK(GetDefaultKinectSensor(&pKinect));

	if (pKinect) {
		IDepthFrameSource* pDepthFrameSource = nullptr;
		IFrameDescription* pDepthFrameDescription = nullptr;
		IColorFrameSource* pColorFrameSource = nullptr;
		IFrameDescription* pColorFrameDescription = nullptr;
		ERROR_CHECK(pKinect->Open());

		// �ȉ��ł́A3�̎����܂Ƃ߂Ď��{���Ă���
		// (1) DepthFrameReader �̎擾
		//   Kinect ���� DepthFrameSource ���擾��
		//   DepthFrameSource ���� DepthFrameReader ���擾
		// (2) ��ʂ̕��ƍ������擾
		//   DepthFrameSource ���� DepthFrameDescription ���擾��
		//   DepthFrameDescription ���� get_Height(), get_Width()���Ă�

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

		// MultiSourceFrameReader ���I�[�v��
		ERROR_CHECK(pKinect->OpenMultiSourceFrameReader(FrameSourceTypes_Color | FrameSourceTypes_Depth
			| FrameSourceTypes_Body, &pMultiSourceFrameReader));
		pMultiSourceFrameReader->SubscribeMultiSourceFrameArrived(&waitableHandle);

		// �J���[��ʂ�ۑ�����z����m��
		depthBuffer = new UINT16[depthHeight * depthWidth];
		colorBuffer = new unsigned char[colorHeight * colorWidth * 4];
		pCameraSpacePoint = new CameraSpacePoint[depthHeight * depthWidth];
		pColorSpacePoint = new ColorSpacePoint[depthHeight * depthWidth];
		pBody = new IBody* [BODY_COUNT];
		for(int i = 0; i < BODY_COUNT; i++) {
			pBody[i] = nullptr;
		}

		// �r���ŗ��p���ĕs�v�ɂȂ��� DepthFrameSource ��DepthFrameDescription ���J��
		SafeRelease(pDepthFrameSource);
		SafeRelease(pDepthFrameDescription);
		SafeRelease(pColorFrameSource);
		SafeRelease(pColorFrameDescription);

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
