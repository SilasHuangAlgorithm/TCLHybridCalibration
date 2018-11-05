/*******************************************************************************
*Copyright(c)  Co., Inc. 1998-2018. All rights reserved.
*All rights reserved
*
*文件名: TCLHybridCalibration
*描述: 手机双摄像头一次拍图标定
*
*修改历史:
*1.作者: Silas Huang
*  日期: 2018.8.3
*  修改: 混合标定代码创建与整理
********************************************************************************/

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "OpenCVconfig.h"
#include <cctype>
#include <stdio.h>
#include <time.h>
#include <io.h>
#include <vector>
#include <string.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdlib.h>
#include <ctype.h>
#include <fstream>
#include "monoCalib.h"
#include "stereoCalib.h"

using namespace std;
using namespace cv;

static int print_help()
{
	cout <<
		" Given a list of chessboard images, the number of corners (nx, ny)\n"
		" on the chessboards, and a flag: useCalibrated for \n"
		"   calibrated (0) or\n"
		"   uncalibrated \n"
		"     (1: use cvStereoCalibrate(), 2: compute fundamental\n"
		"         matrix separately) stereo. \n"
		" Calibrate the cameras and display the\n"
		" rectified results along with the computed disparity images.   \n" << endl;
	cout << "Usage:\n ./stereo_calib -w board_width -h board_height [-nr /*dot not view results*/] <image list XML/YML file>\n" << endl;
	return 0;
}

void getFiles(string path, vector<string>& files)
{
	//long hFile = 0; //x86使用该处定义
	intptr_t hFile = 0; //x64使用该处定义
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}



int main(int argc, char** argv)
{
	Size boardSize;
	float squareSize;
	string imagelistfn;
	bool showRectified = true;

	for (int i = 1; i < argc; i++)
	{
		if (string(argv[i]) == "-w")
		{
			if (sscanf(argv[++i], "%d", &boardSize.width) != 1 || boardSize.width <= 0)
			{
				cout << "invalid board width" << endl;
				return print_help();
			}
		}
		else if (string(argv[i]) == "-h")
		{
			if (sscanf(argv[++i], "%d", &boardSize.height) != 1 || boardSize.height <= 0)
			{
				cout << "invalid board height" << endl;
				return print_help();
			}
		}
		else if (string(argv[i]) == "-s")
		{
			if (sscanf(argv[++i], "%f", &squareSize) != 1 || squareSize <= 0)
			{
				cout << "invalid square size" << endl;
				return print_help();
			}
		}
		else if (string(argv[i]) == "-nr")
			showRectified = false;
		else if (string(argv[i]) == "--help")
			return print_help();
		else if (argv[i][0] == '-')
		{
			cout << "invalid option " << argv[i] << endl;
			return 0;
		}
		else
			imagelistfn = argv[i];
	}

	if (imagelistfn == "")
	{
		//imagelistfn = "stereo_calib.xml";
		//boardSize = Size(9, 6);
	}
	else if (boardSize.width <= 0 || boardSize.height <= 0)
	{
		cout << "if you specified XML file with chessboards, you should also specify the board width and height (-w and -h options)" << endl;
		return 0;
	}

	vector<string> imagesname;
	char *filePath = ".\\calibration image pairs";

	getFiles(filePath, imagesname);

	if (imagesname.empty())
	{
		cout << "the string list is empty" << endl;
		return print_help();
	}

	//处理1对图标定方式的输入
	{
		Mat imgAux = imread(imagesname[0]);
		Mat imgMain = imread(imagesname[1]);

		resize(imgMain, imgMain, Size(1600, 1200));

		string name0 = ".\\calibration image pairs\\L_aux.bmp";
		string name1 = ".\\calibration image pairs\\L_main.bmp";
		string name2 = ".\\calibration image pairs\\M_aux.bmp";
		string name3 = ".\\calibration image pairs\\M_main.bmp";
		string name4 = ".\\calibration image pairs\\R_aux.bmp";
		string name5 = ".\\calibration image pairs\\R_main.bmp";

		Mat img0 = Mat::zeros(Size(1600, 1200), CV_8UC3);
		Mat img1 = Mat::zeros(Size(1600, 1200), CV_8UC3);
		Mat img2 = Mat::zeros(Size(1600, 1200), CV_8UC3);
		Mat img3 = Mat::zeros(Size(1600, 1200), CV_8UC3);
		Mat img4 = Mat::zeros(Size(1600, 1200), CV_8UC3);
		Mat img5 = Mat::zeros(Size(1600, 1200), CV_8UC3);

		imgAux(Rect(0, 0, 580, 1200)).copyTo(img0(Rect(0, 0, 580, 1200)));
		imgAux(Rect(580, 0, 520, 1200)).copyTo(img2(Rect(580, 0, 520, 1200)));
		imgAux(Rect(1100, 0, 500, 1200)).copyTo(img4(Rect(1100, 0, 500, 1200)));//imgAux(Rect(1080, 0, 520, 1200)).copyTo(img4(Rect(1080, 0, 520, 1200)));
		
		imgMain(Rect(0, 0, 533, 1200)).copyTo(img1(Rect(0, 0, 533, 1200)));
		imgMain(Rect(533, 0, 533, 1200)).copyTo(img3(Rect(533, 0, 533, 1200)));
		imgMain(Rect(1066, 0, 534, 1200)).copyTo(img5(Rect(1066, 0, 534, 1200)));//imgMain(Rect(1066, 0, 534, 1200)).copyTo(img5(Rect(1066, 0, 534, 1200)));

		imwrite(name0, img0);
		imwrite(name1, img1);
		imwrite(name2, img2);
		imwrite(name3, img3);
		imwrite(name4, img4);
		imwrite(name5, img5);

		
		
		
		//imwrite("img_bilinear.bmp", imgMain);

		imagesname.clear();
		imagesname.push_back(name0);
		imagesname.push_back(name1);
		imagesname.push_back(name2);
		imagesname.push_back(name3);
		imagesname.push_back(name4);
		imagesname.push_back(name5);

		//imshow("img", img0);
		//waitKey();

	}

	vector<string> imagesname0, imagesname1, imagesname2, imagesnameFinal;

	for (int i = 0; i < imagesname.size(); i++)
	{
		if (i % 2 == 0)
		{
			imagesname0.push_back(imagesname[i]);
		}
		else
		{
			imagesname1.push_back(imagesname[i]);
		}

		//if (i == 0 || i == 1 || i == 2|| i ==3 )
		//{
		//	imagesname2.push_back(imagesname[i]);
		//}
	}

	//单目标定函数
	string outfile_aux = "mono_intrinsics_aux.yml";
	string outfile_main = "mono_intrinsics_main.yml";

	bool showAllImages = 1;
	Mat cameraMatrix[2], distCoeffs[2];
	
	Rect stereoROI;

	float aspectRatio = 0.0;
	int areaMax = 1280*950;
	int IdMax = -1;
	Rect rectMax;
	
	int64 t = getTickCount();
	//考虑所有标定组合，共 1+3+3+1=8 种
	if (imagesname.size() == 6)
	{
		int areaTmp = 0;
		//float ratioTmp = 0;

		for (int num = 0; num < 8; num++)
		{
			if (0 == num)
			{
				imagesname2 = imagesname;
				//case 0
				cout << "     <<< CASE 0 >>>\n";
				StereoCalib(imagesname2, stereoROI, cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], boardSize, squareSize, 1, 0);
				//waitKey();
				areaTmp = stereoROI.area(); 
				//if (0 != stereoROI.width)
				//{
				//	ratioTmp = (float)stereoROI.height / (float)stereoROI.width;
				//}

				//单目标定
				MonoCalib(imagesname0, cameraMatrix[0], distCoeffs[0], boardSize, squareSize, outfile_aux, 0);//标定副摄像头
				MonoCalib(imagesname1, cameraMatrix[1], distCoeffs[1], boardSize, squareSize, outfile_main, 0);//标定主摄像头
			}
			else
			{
				if (1 == num)
				{
					//case 1
					cout << "     <<< CASE 1 >>>\n";
					imagesname2.push_back(imagesname[0]);
					imagesname2.push_back(imagesname[1]);
				}
				if (2 == num)
				{
					//case 2
					cout << "     <<< CASE 2 >>>\n";
					imagesname2.push_back(imagesname[2]);
					imagesname2.push_back(imagesname[3]);
				}
				if (3 == num)
				{
					//case 3
					cout << "     <<< CASE 3 >>>\n";
					imagesname2.push_back(imagesname[4]);
					imagesname2.push_back(imagesname[5]);
				}
				if (4 == num)
				{
					//case 4
					cout << "     <<< CASE 4 >>>\n";
					imagesname2.push_back(imagesname[0]);
					imagesname2.push_back(imagesname[1]);
					imagesname2.push_back(imagesname[2]);
					imagesname2.push_back(imagesname[3]);
				}
				if (5 == num)
				{
					//case 5
					cout << "     <<< CASE 5 >>>\n";
					imagesname2.push_back(imagesname[2]);
					imagesname2.push_back(imagesname[3]);
					imagesname2.push_back(imagesname[4]);
					imagesname2.push_back(imagesname[5]);
				}
				if (6 == num)
				{
					//case 6
					cout << "     <<< CASE 6 >>>\n";
					imagesname2.push_back(imagesname[0]);
					imagesname2.push_back(imagesname[1]);
					imagesname2.push_back(imagesname[4]);
					imagesname2.push_back(imagesname[5]);
				}
				if (7 == num)
				{
					//case 7
					cout << "     <<< CASE 7 >>>\n";
					imagesname2 = imagesname;
				}
				StereoCalib(imagesname2, stereoROI, cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], boardSize, squareSize, 1, 0);
				//waitKey();
				areaTmp = stereoROI.area();
				//if (0 != stereoROI.width)
				//{
				//	ratioTmp = (float)stereoROI.height / (float)stereoROI.width;
				//}
				
			}

			//找到标定结果最好，ROI面积最大
			if (areaTmp > areaMax )
			{
				IdMax = num;
				areaMax = areaTmp;
				imagesnameFinal = imagesname2;
				rectMax = stereoROI;
				//if (abs(ratioTmp - 0.75) < abs(aspectRatio - 0.75))
				//{
				//	aspectRatio = ratioTmp;
				//	IdMax = num;
				//	rectMax = stereoROI;
				//	//areaMax = areaTmp;
				//	imagesnameFinal = imagesname2;
				//}
			}
			imagesname2.clear();
		}
	}
	else
	{
		cout << "错误：请输入3对标定图像！\n";
	}

	//确认标定结果最好的那一组结果
	if (rectMax.width > 1280 && rectMax.height > 930)
	{
		cout << "\n\n***** 最佳标定结果为：CASE" << IdMax << " *****" << endl;

		if (IdMax == 0)
		{

			cameraMatrix[0].release();
			cameraMatrix[1].release();
			distCoeffs[0].release();
			distCoeffs[1].release();
			StereoCalib(imagesname, stereoROI, cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], boardSize, squareSize, 1, 0);
			waitKey();
		}
		else
		{
			//单目标定
			//MonoCalib(imagesname0, cameraMatrix[0], distCoeffs[0], boardSize, squareSize, outfile_aux, 0);//标定副摄像头
			//MonoCalib(imagesname1, cameraMatrix[1], distCoeffs[1], boardSize, squareSize, outfile_main, 0);//标定主摄像头

			StereoCalib(imagesnameFinal, stereoROI, cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], boardSize, squareSize, 1, 0);
			waitKey();
		}
	}
	else
	{
		cout << "  标定结果不理想，请重新拍图\n";
	}

	t = getTickCount() - t;
	printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());
	//单目标定
	//MonoCalib(imagesname0, cameraMatrix[0], distCoeffs[0], boardSize, squareSize, outfile_aux, 0);//标定副摄像头
	//MonoCalib(imagesname1, cameraMatrix[1], distCoeffs[1], boardSize, squareSize, outfile_main, 0);//标定主摄像头

	//双目标定
	//StereoCalib(imagesname2, stereoROI, cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], boardSize, squareSize, 1, 0);

	return 0;

}