#pragma once
#include "OpenCVconfig.h"

using namespace std;
using namespace cv;

void StereoCalib(const vector<string>& imagelist, Rect& validROI, Mat cameraMatrix0, Mat distCoeffs0, Mat cameraMatrix1, Mat distCoeffs1, Size boardSize, float squareSizeTmp,  bool showImage = false, bool saveInverseMap = false);

