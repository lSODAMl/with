#include "KinectView.h"

using namespace cv;

KinectView::KinectView()
{
}

KinectView::~KinectView()
{
}

void KinectView::ShowScreen(Mat *depthImage)
{
	medianBlur(*depthImage, *depthImage, 9);

	Mat edge, draw;
	Canny(*depthImage, edge, 50, 150, 3);
	edge.convertTo(draw, CV_8U);
	// 

	std::vector<std::vector<cv::Point>> contours;
	findContours(draw, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	drawContours(*depthImage, contours, -1, Scalar(0, 0, 0), 3);

	Mat show = (*depthImage).clone();
	imshow("depthImage", show);
}
