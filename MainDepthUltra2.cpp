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

// 球を描く
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

// 外積
void cross(float* x, float* y, float* result) {
  result[0] = x[1] * y[2] - x[2] * y[1];
  result[1] = x[2] * y[0] - x[0] * y[2];
  result[2] = x[0] * y[1] - x[1] * y[0];
}

// 外積
void cross(CameraSpacePoint* xp, CameraSpacePoint* yp, CameraSpacePoint* rp) {
  rp->X = xp->Y * yp->Z - xp->Z * yp->Y;
  rp->Y = xp->Z * yp->X - xp->X * yp->Z;
  rp->Z = xp->X * yp->Y - xp->Y * yp->X;
}

// 内積
float dot(float* x, float* y) {
  return x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
}

// 内積
float dot(CameraSpacePoint* xp, CameraSpacePoint* yp) {
  return xp->X * yp->X + xp->Y * yp->Y + xp->Z * yp->Z;
}


// 正規化
void norm(float* x, float* result) {
  float length = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
  result[0] = x[0] / length;
  result[1] = x[1] / length;
  result[2] = x[2] / length;
}

// 正規化
void norm(CameraSpacePoint* xp, CameraSpacePoint* rp) {
  float length = sqrt(xp->X * xp->X + xp->Y * xp->Y + xp->Z * xp->Z);
  rp->X = xp->X / length;
  rp->Y = xp->Y / length;
  rp->Z = xp->Z / length;
}

// 距離
float distance(CameraSpacePoint* xp, CameraSpacePoint* yp) {
  return sqrt((xp->X - yp->X) * (xp->X - yp->X) 
	      + (xp->Y - yp->Y) * (xp->Y - yp->Y) 
	      + (xp->Z - yp->Z) * (xp->Z - yp->Z));
}

// 距離
float distance(float* xp, float* yp) {
  return sqrt((xp[0] - yp[0]) * (xp[0] - yp[0]) 
	      + (xp[1] - yp[1]) * (xp[1] - yp[1]) 
	      + (xp[2] - yp[2]) * (xp[2] - yp[2]));
}

// 方向ベクトル
// 　xp:起点, yp:終点, dirp: 方向ベクトル
void direction(CameraSpacePoint* xp, CameraSpacePoint* yp, float* dirp) {
  dirp[0] = yp->X - xp->X;
  dirp[1] = yp->Y - xp->Y;
  dirp[2] = yp->Z - xp->Z;
}

// 長さ
float length(float* xp) {
  return sqrt(xp[0] * xp[0] + xp[1] * xp[1] + xp[2] * xp[2]);
}

// 長さ
float length(CameraSpacePoint* xp) {
  return sqrt(xp->X * xp->X + xp->Y * xp->Y + xp->Z * xp->Z);
}

// コサイン
float cosine(float* xp, float* yp) {
  return dot(xp, yp) / length(xp) / length(yp);
}

// コサイン
float cosine(CameraSpacePoint* xp, CameraSpacePoint* yp) {
  return dot(xp, yp) / length(xp) / length(yp);
}

// 中心座標
void center(CameraSpacePoint* xp, CameraSpacePoint* yp, CameraSpacePoint* cp) {
  cp->X = (xp->X + yp->X) / 2;
  cp->Y = (xp->Y + yp->Y) / 2;
  cp->Z = (xp->Z + yp->Z) / 2;
}

// CameraSpacePoint から float* への変換
void pointToArray(CameraSpacePoint* p, float* fp) {
  fp[0] = p->X;
  fp[1] = p->Y;
  fp[2] = p->Z;
}

// float* から CameraSpacePoint への変換
void arrayToPoint(float* fp, CameraSpacePoint* p) {
  p->X = fp[0];
  p->Y = fp[1];
  p->Z = fp[2];
}

void kamehameha(Joint* pJoint, int id) {
  // 気の強さ 0-299, ユーザごとに計測
  static int power[7] = {0};
  Joint handRight = pJoint[JointType_HandRight];
  Joint handLeft = pJoint[JointType_HandLeft];
  CameraSpacePoint right = handRight.Position;
  CameraSpacePoint left = handLeft.Position;

  // 右手と左手の距離
  float dist = distance(&right, &left);

  // 距離が10cm以下の時には、気を溜める
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

  // 気がたまり始めた
  if(power[id] > 30) {
    CameraSpacePoint centerPoint;
    center(&right, &left, &centerPoint);
    glColor3f(1, 1, 0.5);
    drawSphere(centerPoint, power[id] / 3000.0);

    // 光を少量放つ
    glColor3f(1, 1, 1);
    glBegin(GL_LINES);
    for(int i = 0; i < 100; i++) {
      // 光線の長さ
      float len = rand() % power[id] / 1500.0;
      // 乱数で、様々な方向に光を放つ
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

    // 気が十分たまった
    if(power[id] > 500) {
      CameraSpacePoint elbow = pJoint[JointType_ElbowRight].Position;
      CameraSpacePoint shoulder = pJoint[JointType_ShoulderRight].Position;
      float dir0[3];
      float dir1[3];

      // 肘が伸びているかどうかを計算するためのベクトルを取得
      direction(&elbow, &right, dir0);
      direction(&shoulder, &elbow, dir1);

      // 肘が伸びているとカメハメ波を放つ
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

// スペシウム光線
void specium2(Joint* pJoint, int id) {
  CameraSpacePoint right = pJoint[JointType_HandRight].Position;
  CameraSpacePoint left = pJoint[JointType_HandLeft].Position;
  CameraSpacePoint eright = pJoint[JointType_ElbowRight].Position;
  CameraSpacePoint sright = pJoint[JointType_ShoulderRight].Position;

  // 右肘と左手と頭の距離
  float dist0 = distance(&eright, &left);

  float dir[3], dir2[3];
  direction(&sright, &eright, dir);
  direction(&eright, &right, dir2);

  // 距離が20cm以下の時には、光線を発射
  float th = 0.20;
  if(dist0 < th && cosine(dir, dir2) < 0.2) {
    float dir[3]; // 光線の向き
    direction(&sright, &eright, dir);
    norm(dir, dir);

    // 光を放つ
    glColor3f(0.8, 0.8, 1.0);
    glBegin(GL_LINES);
    for(int i = 0; i < 100; i++) {
      // 光線の長さ
      float len = rand() % 300 / 100.0;
      float wide = 0.03;
      // 乱数で、少しずつ光線の方向をずらす
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
// スペシウム光線newspa
void specium(Joint* pJoint, int id) {
	CameraSpacePoint right = pJoint[JointType_HandRight].Position;
	CameraSpacePoint left = pJoint[JointType_HandLeft].Position;
	CameraSpacePoint eright = pJoint[JointType_ElbowRight].Position;
	CameraSpacePoint sright = pJoint[JointType_ShoulderRight].Position;

	// 右肘と左手と頭の距離
	float dist0 = distance(&eright, &left);

	float dir[3], dir2[3];
	direction(&sright, &eright, dir);
	direction(&eright, &right, dir2);

	// 距離が20cm以下の時には、光線を発射
	float th = 0.20;
	if (dist0 < th && cosine(dir, dir2) < 0.2) {
		float dir[3]; // 光線の向き
		direction(&sright, &eright, dir);
		norm(dir, dir);

		// 光を放つ
		glColor3f(0.8, 0.8, 1.0);
		glBegin(GL_LINES);
		for (int i = 0; i < 100; i++) {
			// 光線の長さ
			float len = rand() % 300 / 100.0;
			float wide = 0.03;
			// 乱数で、少しずつ光線の方向をずらす
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

// エメリウム光線
void emerium(Joint* pJoint, int id) {
  CameraSpacePoint right = pJoint[JointType_HandRight].Position;
  CameraSpacePoint left = pJoint[JointType_HandLeft].Position;
  CameraSpacePoint eright = pJoint[JointType_ElbowRight].Position;
  CameraSpacePoint eleft = pJoint[JointType_ElbowLeft].Position;
  CameraSpacePoint head = pJoint[JointType_Head].Position;

  // 右手と左手と頭の距離
  float dist0 = distance(&right, &left);
  float dist1 = distance(&right, &head);
  float dist2 = distance(&left, &head);

  // 距離が30---0cm以下の時には、光線を発射
  float th = 0.00;
  if(dist0 < th && dist1 < th && dist2 < th) {
    float dir0[3];
    float dir1[3];
    float dir[3]; // 光線の向き
    direction(&right, &eright, dir0);
	dir[2] -= 0.1;
    direction(&left, &eleft, dir1);
    cross(dir0, dir1, dir);
    norm(dir, dir);

    // 光を放つ
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
    // 光を少量放つ
    glBegin(GL_LINES);
    for(int i = 0; i < 100; i++) {
      // 光線の長さ
      float len = rand() % 1000 / 1000.0 * size;
      // 乱数で、様々な方向に光を放つ
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

// 円柱の描画
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
  for(int i = 0; i < slices; i++) {
    float theta = 2 * i * PI / slices;
    glVertex3f(top.X + r * vec2[0] * cos(theta) + r * vec3[0] * sin(theta),
	     top.Y + r * vec2[1] * cos(theta) + r * vec3[1] * sin(theta),
	     top.Z + r * vec2[2] * cos(theta) + r * vec3[2] * sin(theta));
  }
  glEnd();

  // 底面
  glBegin(GL_POLYGON);
  for(int i = 0; i < slices; i++) {
    float theta = 2 * i * PI / slices;
    glVertex3f(bottom.X + r * vec2[0] * cos(theta) + r * vec3[0] * sin(theta),
	     bottom.Y + r * vec2[1] * cos(theta) + r * vec3[1] * sin(theta),
	     bottom.Z + r * vec2[2] * cos(theta) + r * vec3[2] * sin(theta));
  }
  glEnd();

  // 側面
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

// 円錐の描画
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

  // 底面
  glBegin(GL_POLYGON);
  for(int i = 0; i < slices; i++) {
    float theta = 2 * i * PI / slices;
    glVertex3f(bottom.X + r * vec2[0] * cos(theta) + r * vec3[0] * sin(theta),
	     bottom.Y + r * vec2[1] * cos(theta) + r * vec3[1] * sin(theta),
	     bottom.Z + r * vec2[2] * cos(theta) + r * vec3[2] * sin(theta));
  }
  glEnd();

  // 側面
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

// 配列用の楕円球関数
void drawEgg(float *fp, float w, float h) {
  drawEgg(fp[0], fp[1], fp[2], w, h);
}

// 配列用のトーラス描画関数
void drawTorus(float *fp, float in, float out) {
  drawTorus(fp[0], fp[1], fp[2], in, out);
}

// 配列用の星描画関数
void drawStar(float *fp, float size) {
  CameraSpacePoint p;
  arrayToPoint(fp, &p);
  drawStar(p, size);
}

// 配列用の星描画関数
void drawSphere(float *fp, float r) {
  CameraSpacePoint p;
  arrayToPoint(fp, &p);
  drawSphere(p, r);
}

// 配列用の星描画関数
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
	
	// おかもち
	glColor3f(1, 1, 0);
	drawCone(p2, p, 0.2, 20);

	// 頭
	glColor3f(1, 0, 0);
	drawSphere(head.Position, 0.3);

	// 胴体
	Joint spine = pJoint[JointType_SpineBase];
	glColor3f(0, 1, 0);
	drawSphere(spine.Position, 0.5);
}

/*
 表示用関数
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

	// テクスチャモードを解除する
	glBindTexture(GL_TEXTURE_2D, 0);

	Joint pJoint[JointType_Count];

	// 星を描く
		

	// ホタル
	//drawFireFly();

	// 雪を描く
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

	// この行を消さないこと
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
