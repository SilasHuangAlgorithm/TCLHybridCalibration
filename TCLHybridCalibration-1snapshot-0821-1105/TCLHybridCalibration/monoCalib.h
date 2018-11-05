#pragma once
#include "OpenCVconfig.h"

using namespace std;
using namespace cv;

int MonoCalib(const vector<string>& imagelist, Mat &cameraMatrix, Mat &distCoeffs, Size boardSize, float squareSize, string monoPramas, bool showImage = false);