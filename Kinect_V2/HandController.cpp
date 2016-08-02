#define _CRT_SECURE_NO_WARNINGS
#include "HandController.h"

using namespace cv;

HandController::HandController()
{
	handCheck = false;
}


HandController::~HandController()
{
}

void HandController::DetectHand(Mat *pImage)
{
	Mat inRangeImage;
	
	SmoothHand(pImage, &inRangeImage);

	Mat temp = (inRangeImage).clone();
	temp = Scalar(0);

	std::vector<std::vector<cv::Point>> contours_f;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(inRangeImage, contours_f, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	if (contours_f.size() != 0)
	{
		int idx = 0, largestComp = 0;
		double maxArea = 0;
		for (; idx >= 0; idx = hierarchy[idx][0])
		{
			const std::vector<Point>& c = contours_f[idx];
			double area = fabs(contourArea(Mat(c))); //contourArea(contours[idx]),
													 //std::cout << "area: " <<area << std::endl;
			if (area > maxArea)
			{
				maxArea = area;
				largestComp = idx;
			}
		}

		drawContours(inRangeImage, contours_f, largestComp, cv::Scalar(255, 255, 255), CV_FILLED, 8, hierarchy);

		Rect r0 = boundingRect(Mat(contours_f[largestComp]));
		std::pair<cv::Point, double> maxCircle = FindMaxCircle(contours_f[largestComp], *pImage);

		imshow("Contour", inRangeImage);

		//convexity defects
		ConvexityHullAndDefects(*pImage, contours_f, largestComp);

		if (handCheck == true)
		{
			char buffer[64] = { 0 };
			sprintf(buffer, "Hand");
			putText(*pImage, buffer, Point(300, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 3, 8);
		}

		if (handCheck == true)
		{
			circle(temp, maxCircle.first, maxCircle.second + 24, cv::Scalar(225, 255, 255), 2, CV_AA);

			cv::Mat andImg;
			cv::bitwise_and(inRangeImage, temp, andImg);
			Mat gray;
			Mat src_mat;
			Mat canny_mat;
			Canny(andImg, canny_mat, 30, 128, 3, false);

			std::vector<std::vector<cv::Point>> fingerContours;

			findContours(canny_mat, fingerContours, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			int counting = fingerContours.size() - 1;
			if (counting <= 0)
				handCheck = false;
			char buffer[64] = { 0 };
			std::cout << "counting: " << counting << std::endl;
			sprintf(buffer, "finger: %d", counting);
			putText(*pImage, buffer, Point(100, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 3, 8);

			imshow("AND", andImg);
		}

		namedWindow("Contour", 0);

		imshow("Source", *pImage);
	}
}

void HandController::SmoothHand(Mat *pImage, Mat *inrangeImage)
{
	Scalar lowerb(0, 255, 153);
	Scalar upperb(0, 255, 153);
	inRange(*pImage, lowerb, upperb, *inrangeImage);
	erode(*inrangeImage, *inrangeImage, Mat());
	dilate(*inrangeImage, *inrangeImage, cv::Mat(), Point(-1, -1), 2);
}

std::pair<cv::Point, double> HandController::FindMaxCircle(const std::vector<Point>& polyCurves, const cv::Mat& frame)
{
	std::pair<cv::Point, double> circle;
	double dist = -1;
	double maxdist = -1;

	for (int i = 0; i < frame.cols; i += 10) {
		for (int j = 0; j < frame.rows; j += 10) {
			dist = pointPolygonTest(polyCurves, cv::Point(i, j), true);
			if (dist > maxdist) {
				maxdist = dist;
				circle.first = cv::Point(i, j);
			}
		}
	}
	circle.second = maxdist;

	return circle;
}

void HandController::ConvexityHullAndDefects(Mat &original, std::vector<std::vector<cv::Point>>& contours, int largIdx)
{
	std::vector<Vec4i> hierarchy;
	/// Find the convex hull,contours and defects for each contour
	std::vector<std::vector<Point> >hull(contours.size());
	std::vector<std::vector<int> >inthull(contours.size());
	std::vector<std::vector<Vec4i> >defects(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		convexHull(Mat(contours[i]), hull[i], false);
		convexHull(Mat(contours[i]), inthull[i], false);
		if (inthull[i].size() > 3)
			convexityDefects(contours[i], inthull[i], defects[i]);
	}

	//find  hull and contour and defects end here
	//this will find largest contour

	//search for largest contour has end

	if (contours.size() > 0)
	{
		Mat tempImg = Mat(cv::Size(original.cols, original.rows), CV_8UC3, Scalar(0, 0, 0));
		drawContours(tempImg, contours, largIdx, CV_RGB(75, 224, 191), 2, 8, hierarchy);
		//       bool matchChecking = false;
		//       if(!tempImg.empty())
		//          matchChecking = HandMatching(tempImg);
		DrawDefects(defects[largIdx], contours[largIdx], original, contours);
	}

	imshow("convexity", original);
}

void HandController::DrawDefects(std::vector<Vec4i> convexityDefectsSet, std::vector<Point> mycontour, Mat &original, std::vector< std::vector< cv::Point>> contours)
{
	if ((contourArea(mycontour) > 3000) && contourArea(mycontour) < 8000)
	{
		std::vector<std::vector<int> >hull(contours.size());
		std::vector<std::vector<Vec4i>> convDef(contours.size());
		std::vector<std::vector<Point>> defect_points(contours.size());
		std::vector<Point> tips;
		std::vector<Point> defects;
		double maxArea = 0;
		for (int i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) > 4000)
			{
				convexHull(contours[i], hull[i], false);
				convexityDefects(contours[i], hull[i], convDef[i]);

				for (int k = 0; k < convDef[i].size(); k++)
				{
					if (convDef[i][k][3] > 20 * 256) // filter defects by depth
					{
						int ind_0 = convDef[i][k][0];
						int ind_1 = convDef[i][k][1];
						int ind_2 = convDef[i][k][2];
						defect_points[i].push_back(contours[i][ind_2]);
						cv::circle(original, contours[i][ind_0], 5, Scalar(0, 0, 0), -1);   //손 꼭지점
																							//cv::circle(original, contours[i][ind_1], 5, Scalar(255, 0, 0), -1);
						cv::circle(original, contours[i][ind_2], 5, Scalar(255, 0, 0), -1);                                                   //cv::circle(original, contours[i][ind_2], 5, Scalar(0, 0, 255), -1);
						tips.push_back(contours[i][ind_0]);
						defects.push_back(contours[i][ind_2]);
					}
				}
			}
			std::cout << "area: " << contourArea(contours[i]) << std::endl;
		}
		//size 제한
		std::cout << "tip: " << tips.size() << std::endl;
		std::cout << "defect: " << defects.size() << std::endl;
		if (tips.size() >= 4 && defects.size() >= 4)
			handCheck = true;
	}
}
