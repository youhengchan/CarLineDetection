// image_process.cpp

#include "image_process.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <numeric>

using namespace std;
using namespace cv;

// 构造函数定义
// 使用初始化列表 ":" 的方式比使用初始化函数效率高
/* ':' 的三个作用：
*  1.对父类的构造函数进行初始化
*  2.对本类中的成员变量进行初始化
*  3.对本类中的const类型变量进行初始化（只能在这里初始化）
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

// 成员函数定义
Mat Image_Process::process()
{
	// 读取图片 
	Mat image;
	image = image_src;
	if (image.empty())
	{
		cout << "读取图片出错！" << endl;
	}

	// 转为灰度图片
	Mat image_gray;
	cvtColor(image, image_gray, COLOR_BGR2GRAY);

#ifdef DEBUG
	// 展示中间过程
	imshow("Grayscale Img", image_gray);
	waitKey();
#endif // DEBUG

	

	// 高斯滤波
	Mat image_gau;
	GaussianBlur(image_gray, image_gau, Size(5,5), 0, 0);

#ifdef DEBUG
	// 展示中间过程
	imshow("Gaussian Img", image_gau);
	waitKey();
#endif

#ifdef SIDEVERSION
    // 开操作，对受损车道线进行填充修复
    int morph_elem = 0; // 0: Rect  1: Cross  2: Ellipse
    int morph_size = 10;
    int morph_operator = 0;  // Opening operation
    int operation = morph_operator + 2;

    Mat element = getStructuringElement(morph_elem, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
    morphologyEx(image_gau, image_gau, operation, element);

#ifdef DEBUG
    // 展示中间过程
    imshow("Opening Img", image_gau);
    waitKey();
#endif

#endif


	// 边缘检测 
	Mat image_canny;
	Canny(image_gau, image_canny, 100, 200, 3);
#ifdef DEBUG
    // 展示中间过程
    imshow("Edge Detection With Canny Img", image_canny);
    waitKey();
#endif

#ifdef SIDEVERSION	

    // 二值化
    threshold(image_gau, image_gau, 50, 255, THRESH_OTSU);
#ifdef DEBUG
    imshow("Img threshold", image_gau);
    waitKey();
#endif // DEBUG
#endif


	// 提取ROI区域
	Mat mask = Mat::zeros(image_canny.size(), CV_8UC1);
	Mat dstImage;

    Point PointArray[4];
	// 梯形四个点所在数组

#ifdef FRONTVERSION

	PointArray[0] = Point(leftBottomPoint, mask.rows-1);
	PointArray[1] = Point(leftTopPoint, trapezoidHeight);
	PointArray[2] = Point(rightTopPoint, trapezoidHeight);
	PointArray[3] = Point(rightBottomPoint, mask.rows-1);

#elif defined SIDEVERSION// 检测侧面车道线

    PointArray[0] = Point(leftBottomPoint, mask.rows * 0.65);
    PointArray[1] = Point(leftTopPoint, mask.rows / 10);
    PointArray[2] = Point(rightTopPoint, mask.rows / 10);
    PointArray[3] = Point(rightBottomPoint, mask.rows * 0.65);

#endif

	// 多边形绘图函数
	fillConvexPoly(mask, PointArray, 4, Scalar(255));

#ifdef DEBUG
	// 展示中间过程
	imshow("Mask with ROI", mask);
	waitKey();
#endif

	// 图片与操作
	bitwise_and(mask, image_canny, dstImage);

#ifdef DEBUG
	// 展示中间过程
	imshow("Img And Operation Result", dstImage);
	waitKey();
#endif

	// 霍夫变换
	vector<Vec4i> lines;
	int rho = 1;                      // 直线搜索步长，为1个像素
	double theta = CV_PI / 180;       // 直线搜索角度步长，为弧度
	int threshold = 10;               // 多少个交点
	int max_line_gap = 50;            // 两条直线并列多远的时候认为是两条（默认为0）
	int min_line_len = 25;            // 最低线段长度10像素

	HoughLinesP(dstImage, lines, rho, theta, threshold, min_line_len, max_line_gap);


	// 前向车道线检测拟合
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
	// 记录侧边检测的车道线高度
	double ySide = 0;
    // 遍历检测到的所有直线
	for (size_t i = 0; i < lines.size(); i++){
		Vec4i L;
		double slope = 0, b = 0;  // 斜率和截距
		L = lines[i];
		slope = (double)(L[3] - L[1]) / (L[2] - L[0]);  // k = (y2 - y1) / (x2 - x1)
		b = L[1] - L[0] * slope;
		 
#ifdef FRONTVERSION
        // 设置检测出的车道线的斜率范围
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
		// 对于单侧车道线检测，全部存到右边
		if (slope > -0.5 && slope < 0.5) {
			slope_right.push_back(slope);
			b_right.push_back(b);
			ySide = ySide + L[1] + L[3];
		}
#endif

	}
	
    // 累加求出的所有直线的斜率、截距并求平均
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

	// 单侧车道线检测
	int singleL = 0;
	int singleR = image.cols - 1;

#ifdef FRONTVERSION
	int x1l = (y1 - b_left_mean) / slope_left_mean;
	int x2l = (y2 - b_left_mean) / slope_left_mean;
#endif



	// 写入班级学号姓名
	putTextZH(image, "湖南大学 数媒1701 CHX", Point(50, 50), Scalar(0, 255, 0), 30, "微软雅黑");

	// 对于前向压线检测进行梯形ROI区域绘制
#ifdef  FRONTVERSION
	line(image, PointArray[0], PointArray[1], Scalar(0, 0, 255), 3, LINE_AA);
	line(image, PointArray[1], PointArray[2], Scalar(0, 0, 255), 3, LINE_AA);
	line(image, PointArray[2], PointArray[3], Scalar(0, 0, 255), 3, LINE_AA);
#endif

    // 前向车道线绘制车道线检测结果

	// 原图中绘制车道线
	// 默认为单侧检测，只匹配一条车道线
#ifdef SIDEVERSION
	line(image, Point(singleL, ySide / 2), Point(singleR, ySide / 2), Scalar(120, 255, 119), 5, LINE_AA);
#endif

#ifdef FRONTVERSION
	// 定义FRONTVERSION 宏后开启两条车道线
	line(image, Point(x1r, y1), Point(x2r, y2), Scalar(120, 255, 119), 5, LINE_AA);
	line(image, Point(x1l, y1), Point(x2l, y2), Scalar(120, 255, 119), 5, LINE_AA);
#endif
	return image;
}

Image_Process::~Image_Process(){}




