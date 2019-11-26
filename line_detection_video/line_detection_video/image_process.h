// image_process.h

#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

// 注释DEBUG宏关闭调试模式
// #define DEBUG

// 开启FRONT VERSION宏 开启前向车道检测模式
 #define FRONTVERSION

#ifndef FRONTVERSION

// 开启SIDE VERSION宏 开启侧向车道检测模式
#define SIDEVERSION
#endif

#include <iostream>
#include <opencv2/opencv.hpp>
// 用于显示中文字符
#include "./puttextzh/puttextzh/puttextzh.h"

using namespace std;
using namespace cv;

class Image_Process
{
public:
	
	const int leftBottomPoint;
	const int leftTopPoint;
	const int rightTopPoint;
	const int rightBottomPoint;
	const int trapezoidHeight;
	
	Mat image_src;
	Mat image_dst;

	Image_Process(Mat image);     // 构造函数声明
	~Image_Process();             // 析构函数声明

	Mat process();                // 图像处理函数声明

};
#endif