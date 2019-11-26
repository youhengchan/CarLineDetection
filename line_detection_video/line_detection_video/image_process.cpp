// image_process.cpp

#include "image_process.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <numeric>

using namespace std;
using namespace cv;

// ���캯������
// ʹ�ó�ʼ���б� ":" �ķ�ʽ��ʹ�ó�ʼ������Ч�ʸ�
/* ':' ���������ã�
*  1.�Ը���Ĺ��캯�����г�ʼ��
*  2.�Ա����еĳ�Ա�������г�ʼ��
*  3.�Ա����е�const���ͱ������г�ʼ����ֻ���������ʼ����
*/
Image_Process::Image_Process(Mat image) : 
	leftBottomPoint(image.cols / 5), 
	leftTopPoint(image.cols / 100 * 45), 
	rightTopPoint(image.cols / 100 * 55), 
	rightBottomPoint(image.cols / 5 * 4),
	trapezoidHeight(image.rows / 2)
{
	this->image_src = image;
}

// ��Ա��������
Mat Image_Process::process()
{
	// ��ȡͼƬ 
	Mat image;
	image = image_src;
	if (image.empty())
	{
		cout << "��ȡͼƬ����" << endl;
	}

	// תΪ�Ҷ�ͼƬ
	Mat image_gray;
	cvtColor(image, image_gray, COLOR_BGR2GRAY);

#ifdef DEBUG
	// չʾ�м����
	imshow("Grayscale Img", image_gray);
	waitKey();
#endif // DEBUG

	

	// ��˹�˲�
	Mat image_gau;
	GaussianBlur(image_gray, image_gau, Size(5,5), 0, 0);

#ifdef DEBUG
	// չʾ�м����
	imshow("Gaussian Img", image_gau);
	waitKey();
#endif

#ifdef SIDEVERSION
    // �������������𳵵��߽�������޸�
    int morph_elem = 0; // 0: Rect  1: Cross  2: Ellipse
    int morph_size = 10;
    int morph_operator = 0;  // Opening operation
    int operation = morph_operator + 2;

    Mat element = getStructuringElement(morph_elem, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
    morphologyEx(image_gau, image_gau, operation, element);

#ifdef DEBUG
    // չʾ�м����
    imshow("Opening Img", image_gau);
    waitKey();
#endif

#endif


	// ��Ե��� 
	Mat image_canny;
	Canny(image_gau, image_canny, 100, 200, 3);
#ifdef DEBUG
    // չʾ�м����
    imshow("Edge Detection With Canny Img", image_canny);
    waitKey();
#endif

#ifdef SIDEVERSION	

    // ��ֵ��
    threshold(image_gau, image_gau, 50, 255, THRESH_OTSU);
#ifdef DEBUG
    imshow("Img threshold", image_gau);
    waitKey();
#endif // DEBUG
#endif


	// ��ȡROI����
	Mat mask = Mat::zeros(image_canny.size(), CV_8UC1);
	Mat dstImage;

    Point PointArray[4];
	// �����ĸ�����������

#ifdef FRONTVERSION

	PointArray[0] = Point(leftBottomPoint, mask.rows-1);
	PointArray[1] = Point(leftTopPoint, trapezoidHeight);
	PointArray[2] = Point(rightTopPoint, trapezoidHeight);
	PointArray[3] = Point(rightBottomPoint, mask.rows-1);

#elif defined SIDEVERSION// �����泵����

    PointArray[0] = Point(leftBottomPoint, mask.rows * 0.65);
    PointArray[1] = Point(leftTopPoint, mask.rows / 10);
    PointArray[2] = Point(rightTopPoint, mask.rows / 10);
    PointArray[3] = Point(rightBottomPoint, mask.rows * 0.65);

#endif

	// ����λ�ͼ����
	fillConvexPoly(mask, PointArray, 4, Scalar(255));

#ifdef DEBUG
	// չʾ�м����
	imshow("Mask with ROI", mask);
	waitKey();
#endif

	// ͼƬ�����
	bitwise_and(mask, image_canny, dstImage);

#ifdef DEBUG
	// չʾ�м����
	imshow("Img And Operation Result", dstImage);
	waitKey();
#endif

	// ����任
	vector<Vec4i> lines;
	int rho = 1;                      // ֱ������������Ϊ1������
	double theta = CV_PI / 180;       // ֱ�������ǶȲ�����Ϊ����
	int threshold = 10;               // ���ٸ�����
	int max_line_gap = 50;            // ����ֱ�߲��ж�Զ��ʱ����Ϊ��������Ĭ��Ϊ0��
	int min_line_len = 25;            // ����߶γ���10����

	HoughLinesP(dstImage, lines, rho, theta, threshold, min_line_len, max_line_gap);


	// ǰ�򳵵��߼�����
	Mat image_draw = Mat::zeros(image_canny.size(), CV_8UC3);
	vector<int> right_x, right_y, left_x, left_y;
	double slope_right_sum = 0;
	double b_right_sum = 0;
	double slope_left_sum = 0;
	double b_left_sum = 0;
	double slope_right_mean = 0;
	double slope_left_mean = 0;
	double b_right_mean = 0;
	double b_left_mean = 0; 

	vector<double> slope_right, slope_left, b_right, b_left;
	// ��¼��߼��ĳ����߸߶�
	double ySide = 0;
    // ������⵽������ֱ��
	for (size_t i = 0; i < lines.size(); i++){
		Vec4i L;
		double slope = 0, b = 0;  // б�ʺͽؾ�
		L = lines[i];
		slope = (double)(L[3] - L[1]) / (L[2] - L[0]);  // k = (y2 - y1) / (x2 - x1)
		b = L[1] - L[0] * slope;
		 
#ifdef FRONTVERSION
        // ���ü����ĳ����ߵ�б�ʷ�Χ
		if (slope >= 0.45)
		{
			slope_right.push_back(slope);
			b_right.push_back(b);
		}
		else if (slope <= -0.45)
		{
			slope_left.push_back(slope);
			b_left.push_back(b);
		}
#elif defined SIDEVERSION  
		// ���ڵ��೵���߼�⣬ȫ���浽�ұ�
		if (slope > -0.5 && slope < 0.5) {
			slope_right.push_back(slope);
			b_right.push_back(b);
			ySide = ySide + L[1] + L[3];
		}
#endif

	}
	
    // �ۼ����������ֱ�ߵ�б�ʡ��ؾಢ��ƽ��
	slope_right_sum = accumulate(slope_right.begin(), slope_right.end(), 0.0);
	b_right_sum = accumulate(b_right.begin(), b_right.end(), 0.0);
	slope_right_mean = slope_right_sum / slope_right.size();
	b_right_mean = b_right_sum / b_right.size();
	ySide /= slope_right.size();

#ifdef DEBUG
	cout << "ySide = " << ySide << endl;
#endif // DEBUG

	
#ifdef FRONTVERSION
	slope_left_sum = accumulate(slope_left.begin(), slope_left.end(), 0.0);
	b_left_sum = accumulate(b_left.begin(), b_left.end(), 0.0);
	slope_left_mean = slope_left_sum / slope_left.size();
	b_left_mean = b_left_sum / b_left.size();
#endif

#ifdef DEBUG
    cout << "Right slope = " << slope_right_sum << endl;
	cout << "Slope right size = " << slope_right.size() << endl;
    cout << "Left slope = " << -slope_right_sum << endl;
#endif // DEBUG

	

	int y1 = trapezoidHeight;
	int y2 = mask.rows;
	
	int x1r = (y1 - b_right_mean) / slope_right_mean;
	int x2r = (y2 - b_right_mean) / slope_right_mean;

	// ���೵���߼��
	int singleL = 0;
	int singleR = image.cols - 1;

#ifdef FRONTVERSION
	int x1l = (y1 - b_left_mean) / slope_left_mean;
	int x2l = (y2 - b_left_mean) / slope_left_mean;
#endif



	// д��༶ѧ������
	putTextZH(image, "���ϴ�ѧ ��ý1701 CHX", Point(50, 50), Scalar(0, 255, 0), 30, "΢���ź�");

	// ����ǰ��ѹ�߼���������ROI�������
#ifdef  FRONTVERSION
	line(image, PointArray[0], PointArray[1], Scalar(0, 0, 255), 3, LINE_AA);
	line(image, PointArray[1], PointArray[2], Scalar(0, 0, 255), 3, LINE_AA);
	line(image, PointArray[2], PointArray[3], Scalar(0, 0, 255), 3, LINE_AA);
#endif

    // ǰ�򳵵��߻��Ƴ����߼����

	// ԭͼ�л��Ƴ�����
	// Ĭ��Ϊ�����⣬ֻƥ��һ��������
#ifdef SIDEVERSION
	line(image, Point(singleL, ySide / 2), Point(singleR, ySide / 2), Scalar(120, 255, 119), 5, LINE_AA);
#endif

#ifdef FRONTVERSION
	// ����FRONTVERSION ���������������
	line(image, Point(x1r, y1), Point(x2r, y2), Scalar(120, 255, 119), 5, LINE_AA);
	line(image, Point(x1l, y1), Point(x2l, y2), Scalar(120, 255, 119), 5, LINE_AA);
#endif
	return image;
}

Image_Process::~Image_Process(){}




