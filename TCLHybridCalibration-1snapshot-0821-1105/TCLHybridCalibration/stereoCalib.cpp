#include "stereoCalib.h"



void StereoCalib(const vector<string>& imagelist, Rect& validROI, Mat cameraMatrix0, Mat distCoeffs0, Mat cameraMatrix1, Mat distCoeffs1, Size boardSize, float squareSizeTmp, bool showImage, bool saveInverseMap)
{
	cout << "...Starting stereo calibration..." << endl;
	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	ofstream result;
	result.open("ARE & RMS.txt");

	bool displayCorners = true;
	const int maxScale = 2;
	const float squareSize = squareSizeTmp;  // Set this to your actual square size

											 // ARRAY AND VECTOR STORAGE:
	vector<vector<Point2f> > imagePoints[2];
	vector<vector<Point3f> > objectPoints;
	Size imageSize;

	int i, j, k, nimages = (int)imagelist.size() / 2;

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	vector<string> goodImageList;

	for (i = j = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			const string& filename = imagelist[i * 2 + k];
			Mat img = imread(filename, 0);


			if (img.empty())
				break;
			if (imageSize == Size())
				imageSize = img.size();
			//else if( img.size() != imageSize )
			//{
			//    cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
			//    //break;
			//}
			bool found = false;
			vector<Point2f>& corners = imagePoints[k][j];
			for (int scale = 1; scale <= maxScale; scale++)
			{

				Mat timg;
				if (scale == 1)
					timg = img;
				else
					resize(img, timg, Size(), scale, scale, INTER_LINEAR_EXACT);
				found = findChessboardCorners(timg, boardSize, corners,
					CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
				//found = findCirclesGrid( timg, boardSize, corners, 1); //找圆点板圆心


				if (found)
				{
					if (scale > 1)
					{
						Mat cornersMat(corners);
						cornersMat *= 1. / scale;
					}
					break;
				}
			}
			
			if (showImage && displayCorners)
			{
				cout << "  " << filename << endl;
				Mat cimg, cimg1;
				cvtColor(img, cimg, COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640. / MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf, INTER_LINEAR_EXACT);
				imshow("corners", cimg1);
				char c = (char)waitKey(100);
				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
					exit(-1);
			}
			else
				cout << "  " << filename << endl;
				//putchar('.');
			if (!found)
				break;
			cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
					30, 0.01));
		}
		if (k == 2)
		{
			goodImageList.push_back(imagelist[i * 2]);
			goodImageList.push_back(imagelist[i * 2 + 1]);
			j++;
		}
	}
	cout <<"  "<< j << " pairs have been successfully detected.\n";
	nimages = j;

	if (nimages == 0)
	{
		cout << "Error: No image pairs to run the calibration\n";
		return;
	}

	//if( nimages < 2 )
	//{
	//    cout << "Error: too little pairs to run the calibration\n";
	//    return;
	//}



	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	objectPoints.resize(nimages);

	for (i = 0; i < nimages; i++)
	{
		for (j = 0; j < boardSize.height; j++)
			for (k = 0; k < boardSize.width; k++)
				objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
	}

	cout << "  Running stereo calibration ...\n";

	Mat cameraMatrix[2], distCoeffs[2];
	//cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);
	//cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);
	cameraMatrix[0] = cameraMatrix0;
	distCoeffs[0] = distCoeffs0;
	cameraMatrix[1] = cameraMatrix1;
	distCoeffs[1] = distCoeffs1;

	Mat R, T, E, F;
	double rms;
	if (!cameraMatrix[0].empty() && !cameraMatrix[1].empty())
	{
		rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
			cameraMatrix[0], distCoeffs[0],
			cameraMatrix[1], distCoeffs[1],
			imageSize, R, T, E, F,
			CALIB_FIX_INTRINSIC +
			CALIB_USE_INTRINSIC_GUESS,
			TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 1e-6));
	}
	else
	{
		cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);
		cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);

		rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
			cameraMatrix[0], distCoeffs[0],
			cameraMatrix[1], distCoeffs[1],
			imageSize, R, T, E, F,
			CALIB_FIX_ASPECT_RATIO +
			CALIB_ZERO_TANGENT_DIST +
			CALIB_RATIONAL_MODEL +
			CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,//+
			TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 30, 1e-6));
	}

	if (rms < 1.0)
	{
		cout << "  双目标定结果良好\n";
	}
	else
	{
		cout << "  双目标定结果不好，请重新选图\n\n";
		return;
	}

	cout << "    RMS error = " << rms << endl;
	result << "均方根误差为(Root Mean Square Error) :" << rms << endl;

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0


	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
					imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "    Average reprojection err = " << err / npoints << endl;
	result << "平均重投影误差为(Average Reprojection Error)： " << err / npoints << endl;
	// save intrinsic parameters
	FileStorage fs("stereo_intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	//stereoRectify(cameraMatrix[0], distCoeffs[0],
	//	cameraMatrix[1], distCoeffs[1],
	//	imageSize, R, T, R1, R2, P1, P2, Q,
	//	CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

	fs.open("stereo_extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		/*fs.release();*/
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";

	// OpenCV can handle left-right
	// or up-down camera arrangements
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	// COMPUTE AND DISPLAY RECTIFICATION
	//if (!showImage)
	//	return;

	Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)
	bool useCalibrated = true;
	if (useCalibrated)
	{
		// we already computed everything
	}
	// OR ELSE HARTLEY'S METHOD
	else
		// use intrinsic parameters of each camera, but
		// compute the rectification transformation directly
		// from the fundamental matrix
	{
		vector<Point2f> allimgpt[2];
		for (k = 0; k < 2; k++)
		{
			for (i = 0; i < nimages; i++)
				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
		}
		F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
		Mat H1, H2;
		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

		R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
		R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
		P1 = cameraMatrix[0];
		P2 = cameraMatrix[1];
	}

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	//cout << rmap[0][0];

	Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	
	for (i = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;
			remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
			cvtColor(rimg, cimg, COLOR_GRAY2BGR);
			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
			if (useCalibrated)
			{
				Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
					cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
				rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
				
				//保存校准后图像
				//Mat img_rec = rimg(roi_rec);
				//String name_rec = goodImageList[i * 2 + k].substr(0, goodImageList[i * 2 + k].length() - 4) + "_rec.bmp";
				//imwrite(name_rec, rimg);

			}
		}

		if (!isVerticalStereo)
			for (j = 0; j < canvas.rows; j += 16)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (j = 0; j < canvas.cols; j += 16)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
		if (showImage)
		{
			imshow("rectified", canvas);
			char c = (char)waitKey(1000);
			if (c == 27 || c == 'q' || c == 'Q')
				break;
		}
		
	}

	//提取主摄像头有效区域 validROI的图像内容，
	Rect roi_rect(cvRound(validRoi[1].x), cvRound(validRoi[1].y), cvRound(validRoi[1].width), cvRound(validRoi[1].height));
	if (fs.isOpened() && !validRoi[0].empty() && !validRoi[1].empty())
	{
		validROI = roi_rect;
		fs << "roi_rec" << roi_rect;
		//cout << "  Valid ROI rect saved!\n";
		cout << "  Height = " << roi_rect.height << "  Width = " << roi_rect.width << endl;
		fs.release();
	}
	else
	{
		cout << "  Warning: can not save the ROI rect\n";
	}


	//保存逆映射矩阵，得到校准图与原图位置对应关系
	if (saveInverseMap)
	{
		cout << "  Begin saving Inverse Map\n";
		int64 t = getTickCount();
		FileStorage fs("inverseMap.yml", FileStorage::WRITE);

		int HEIGHT, WIDTH;
		HEIGHT = imageSize.height;
		WIDTH = imageSize.width;

		Mat MapX1 = Mat::zeros(imageSize, CV_32FC1);
		Mat MapY1 = Mat::zeros(imageSize, CV_32FC1);
		//主摄像头逆映射表
		Mat MapX2 = Mat::zeros(imageSize, CV_32FC1);
		Mat MapY2 = Mat::zeros(imageSize, CV_32FC1);

		for (int i = 0; i < HEIGHT; i++)
		{
			for (int j = 0; j < WIDTH; j++)
			{
				vector<Point2f> pTmp, pTmp1, pTmp2;

				pTmp.push_back(Point2f(j, i));

				undistortPoints(pTmp, pTmp1, cameraMatrix[0], distCoeffs[0], R1, P1);
				MapX1.at<float>(i, j) = pTmp1[0].x;
				MapY1.at<float>(i, j) = pTmp1[0].y;

				undistortPoints(pTmp, pTmp2, cameraMatrix[1], distCoeffs[1], R2, P2);
				MapX2.at<float>(i, j) = pTmp2[0].x;
				MapY2.at<float>(i, j) = pTmp2[0].y;
			}
		}
		MapX2.convertTo(MapX2, CV_16SC1);
		MapY2.convertTo(MapY2, CV_16SC1);

		if (fs.isOpened())
		{
			fs << "MapX2" << MapX2 << "MapY2" << MapY2;
			cout << "  Inverse Map saved successfully!\n";
			fs.release();
		}

		t = getTickCount() - t;
		printf("  Time elapsed: %fms\n", t * 1000 / getTickFrequency());
	}

	cout << "...Finish stereo calibration...\n\n";

}