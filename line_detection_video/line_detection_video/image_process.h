// image_process.h

#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

// ע��DEBUG��رյ���ģʽ
// #define DEBUG

// ����FRONT VERSION�� ����ǰ�򳵵����ģʽ
 #define FRONTVERSION

#ifndef FRONTVERSION

// ����SIDE VERSION�� �������򳵵����ģʽ
#define SIDEVERSION
#endif

#include <iostream>
#include <opencv2/opencv.hpp>
// ������ʾ�����ַ�
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

	Image_Process(Mat image);     // ���캯������
	~Image_Process();             // ������������

	Mat process();                // ͼ����������

};
#endif