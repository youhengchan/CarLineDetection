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

	cout << "��ʼ�����߼��" << endl;
	
	// ���ö����ļ�����
	string inputFileName = "test.mp4";
	VideoCapture capture(inputFileName);
	cout << "��ʼ��Ƶ��ȡ" << endl;

	Mat frame;
	// ����Ƿ�������:�ɹ���ʱ��isOpened����ture
	if (!capture.isOpened())
	{
		cout << "�޷��򿪸���Ƶ��" << endl;
		return -1;
	}

	// �Զ�ץȡ������Ƶ֡��
	double autoDetectFps = capture.get(CAP_PROP_FPS);

#ifdef DEDUG
	// ���������Ϣ���鿴��⵽��Ƶ֡��
	cout << "FPS = " << autoDetectFps << endl;
#endif
	
	// �Զ�ץȡ��Ƶ֡�ߴ��С
	Size size = Size((int)(capture.get(CAP_PROP_FRAME_WIDTH)) * 2, (int)(capture.get(CAP_PROP_FRAME_HEIGHT)));

#ifdef DEBUG
	cout << "Frame Height = " << size.width << endl
		<< "Frame Width = " << size.height << endl;
#endif // DEBUG

	// ׼�����������Ƶ��д�����ļ�
	VideoWriter writer;
	
	// �ҵ������ļ�����"."���±�λ��
	size_t lastDotIndex = inputFileName.find_last_of('.');
	// ȷ������Դ�ļ����ļ���ʽ
	string inputFileType = inputFileName.substr(lastDotIndex);
	// ����������ļ�����
	const string outputFileName = inputFileName.substr(0, lastDotIndex) + ".result" + inputFileType;
	// �������������ʽ��������Ƶ��ͬ
	int ex = static_cast<int>(capture.get(CAP_PROP_FOURCC));
	// ����Ƶ����ļ�
	writer.open(outputFileName, ex, autoDetectFps, size, true);
	
	// ������ļ�ʧ��
	if (!writer.isOpened()) {
		cout << "Could not Open the output file for vedio writing" << endl;
		exit(-1);
	}

	cout << "��Ƶ���ڴ�����..." << endl;

    Mat tempResult(frame.size(), frame.type());
	
    while (capture.isOpened())
	{
		capture >> frame;
		if (frame.empty())
		{
			break;
		}
		// ��ÿһ֡ͼ����д���
		Image_Process image2(frame);
        Mat dstImg(frame.rows, frame.cols * 2, frame.type());

        Rect rectd = Rect(0, 0, frame.cols, frame.rows);
        Rect recti = Rect(frame.cols, 0, frame.cols, frame.rows);
        frame.copyTo(Mat(dstImg, rectd));

        tempResult = image2.process();

        tempResult.copyTo(Mat(dstImg, recti));
		//��ͼƬд����Ƶ�ļ���
		writer << dstImg; 
		imshow("������ʵʱ���", dstImg);
		waitKey(8);
	}

	cout << "��Ƶ������ϣ��������������" << endl;
	waitKey();
	
	return 0;
}

