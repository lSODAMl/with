#pragma once
#include <windows.h>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;

class HandController
{
public:
	HandController();
	~HandController();

public:
	void DetectHand(Mat *pImage);
	void SmoothHand(Mat *pImage, Mat *inrangeImage);
	std::pair<cv::Point, double> FindMaxCircle(const std::vector<Point>& polyCurves, const cv::Mat& frame);
	void ConvexityHullAndDefects(Mat &original, std::vector<std::vector<cv::Point>>& contours, int largIdx);
	void DrawDefects(std::vector<Vec4i> convexityDefectsSet, std::vector<Point> mycontour, Mat &original, std::vector< std::vector< cv::Point>> contours);

private:
	bool handCheck;
};

