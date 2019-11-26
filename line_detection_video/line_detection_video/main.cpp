#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>
#include <numeric>
#include "image_process.h"


using namespace std;
using namespace cv;

int main(int argc, char** argv)
{

	cout << "开始车道线检测" << endl;
	
	// 设置读入文件名称
	string inputFileName = "test.mp4";
	VideoCapture capture(inputFileName);
	cout << "开始视频读取" << endl;

	Mat frame;
	// 检测是否正常打开:成功打开时，isOpened返回ture
	if (!capture.isOpened())
	{
		cout << "无法打开该视频！" << endl;
		return -1;
	}

	// 自动抓取输入视频帧率
	double autoDetectFps = capture.get(CAP_PROP_FPS);

#ifdef DEDUG
	// 输出调试信息，查看检测到视频帧率
	cout << "FPS = " << autoDetectFps << endl;
#endif
	
	// 自动抓取视频帧尺寸大小
	Size size = Size((int)(capture.get(CAP_PROP_FRAME_WIDTH)) * 2, (int)(capture.get(CAP_PROP_FRAME_HEIGHT)));

#ifdef DEBUG
	cout << "Frame Height = " << size.width << endl
		<< "Frame Width = " << size.height << endl;
#endif // DEBUG

	// 准备将处理的视频流写入结果文件
	VideoWriter writer;
	
	// 找到输入文件名的"."的下标位置
	size_t lastDotIndex = inputFileName.find_last_of('.');
	// 确定输入源文件的文件格式
	string inputFileType = inputFileName.substr(lastDotIndex);
	// 设置输出的文件名称
	const string outputFileName = inputFileName.substr(0, lastDotIndex) + ".result" + inputFileType;
	// 设置输出视屏格式与输入视频相同
	int ex = static_cast<int>(capture.get(CAP_PROP_FOURCC));
	// 打开视频输出文件
	writer.open(outputFileName, ex, autoDetectFps, size, true);
	
	// 打开输出文件失败
	if (!writer.isOpened()) {
		cout << "Could not Open the output file for vedio writing" << endl;
		exit(-1);
	}

	cout << "视频正在处理中..." << endl;

    Mat tempResult(frame.size(), frame.type());
	
    while (capture.isOpened())
	{
		capture >> frame;
		if (frame.empty())
		{
			break;
		}
		// 对每一帧图像进行处理
		Image_Process image2(frame);
        Mat dstImg(frame.rows, frame.cols * 2, frame.type());

        Rect rectd = Rect(0, 0, frame.cols, frame.rows);
        Rect recti = Rect(frame.cols, 0, frame.cols, frame.rows);
        frame.copyTo(Mat(dstImg, rectd));

        tempResult = image2.process();

        tempResult.copyTo(Mat(dstImg, recti));
		//将图片写到视频文件中
		writer << dstImg; 
		imshow("车道线实时检测", dstImg);
		waitKey(8);
	}

	cout << "视频处理完毕，按任意键结束！" << endl;
	waitKey();
	
	return 0;
}

