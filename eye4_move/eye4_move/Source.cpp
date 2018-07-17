#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/opencv_lib.hpp"

int width_size = 600;
int height_size = 600;
int face_x;
int face_y;
int face_width;
int face_height;
double part = 1.02;


using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
	VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, width_size);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, height_size);
	if (!cap.isOpened())
	{
		printf("�J���������o�ł��܂���ł���");
		return -1;
	}
	Mat input_img;

	namedWindow("input_img", CV_WINDOW_AUTOSIZE);

	while (1)
	{
		cap >> input_img;

		if (input_img.empty())
		{
			cout << "Not found";
			return -1;
		}

		Mat hsv_skin_img = Mat(Size(width_size, height_size), CV_8UC3);
		/*	Mat smooth_img;
		Mat hsv_img;*/

		string cascadeName = "haarcascades/haarcascade_frontalface_alt.xml";
		CascadeClassifier cascade;
		if (!cascade.load(cascadeName))
		{
			cout << "Not load";
			return -1;
		}
		vector<Rect> faces;// �}���`�X�P�[���i��j�T��
		cascade.detectMultiScale(input_img, faces, 1.1, 2, 0, Size(30, 30));
		// �摜�C�o�͋�`�C�k���X�P�[���C�Œ��`���C�i�t���O�j�C�ŏ���`


		for (int i = 0; i < faces.size(); i++)
		{
			rectangle(input_img, Point(faces[i].x, faces[i].y), Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height), Scalar(0, 200, 0), 3, CV_AA);
			face_x = faces[i].x;
			face_y = faces[i].y;
			face_width = faces[i].width;
			face_height = faces[i].height;
		}

		for (int y = face_y; y < face_y + face_height; y++)
		{
			for (int x = face_x; x < face_x + face_width; x++)
			{
				int a = input_img.step*y + (x * 3);
				if (input_img.data[a + 0] < input_img.data[a + 1] && input_img.data[a + 1] < input_img.data[a + 2]) //���o
				{
					input_img.data[a + 0] = 255; //���F������ϐF_B
					input_img.data[a + 1] = 255; //���F������ϐF_G
					input_img.data[a + 2] = 255; //���F������ϐF_R
				}
			}
		}
		for (int y = face_y*part; y < face_y*part + face_height / 2; y++)
		{
			for (int x = face_x; x < face_x + face_width; x++){
				int a = input_img.step*y+(x * 3);
				if (input_img.data[a + 0] > input_img.data[a + 1] || input_img.data[a + 1] > input_img.data[a + 2])
				{
					input_img.data[a + 0] = 0;
					input_img.data[a + 1] = 255;
					input_img.data[a + 2] = 255;
				}
			}
		}
		imshow("input_img", input_img);
		if (waitKey(30) >= 0)
		{
			break;
		}
	}
}