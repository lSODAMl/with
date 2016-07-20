#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
using namespace cv;
using namespace std;

Point GetHandCenter(const Mat& mask, double& radius) {
	Mat dst;
	distanceTransform(mask, dst, CV_DIST_L2, 5);
	int maxIdx[2];
	minMaxIdx(dst, NULL, &radius, NULL, maxIdx, mask);

	return Point(maxIdx[1], maxIdx[0]);
}

float InnerAngle(float px1, float py1, float px2, float py2, float cx1, float cy1)
{

	float dist1 = std::sqrt((px1 - cx1)*(px1 - cx1) + (py1 - cy1)*(py1 - cy1));
	float dist2 = std::sqrt((px2 - cx1)*(px2 - cx1) + (py2 - cy1)*(py2 - cy1));

	float Ax, Ay;
	float Bx, By;
	float Cx, Cy;

	Cx = cx1;
	Cy = cy1;
	if (dist1 < dist2)
	{
		Bx = px1;
		By = py1;
		Ax = px2;
		Ay = py2;


	}
	else {
		Bx = px2;
		By = py2;
		Ax = px1;
		Ay = py1;
	}


	float Q1 = Cx - Ax;
	float Q2 = Cy - Ay;
	float P1 = Bx - Ax;
	float P2 = By - Ay;


	float A = std::acos((P1*Q1 + P2*Q2) / (std::sqrt(P1*P1 + P2*P2) * std::sqrt(Q1*Q1 + Q2*Q2)));

	A = A * 180 / CV_PI;

	return A;
}

void DrawTrackingRectangle(Mat* copy, std::vector<cv::Point> largestContours)
{
	Rect r0 = boundingRect(largestContours);
	RotatedRect rotate_rect = minAreaRect(largestContours);
	Point2f points[4];
	rotate_rect.points(points);
	std::vector< std::vector< Point> > polylines;
	polylines.resize(1);
	for (int j = 0; j < 4; ++j)
		polylines[0].push_back(points[j]);

	//draw them on the bounding image.
	cv::polylines(*copy, polylines, true, Scalar(0, 0, 255), 2);
	rectangle(*copy, r0, Scalar(0, 255, 0), 2);  //Draw rectangle in image
}

std::pair<cv::Point, double> findMaxInscribedCircle(const vector<Point>& polyCurves, const cv::Mat& frame)
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


void DrawHullAndDefects(Mat* copy, vector<Point>largestContours, Rect r0)
{
	// Draw hull
	vector<Point> handContour = largestContours;
	vector<int> hull;
	convexHull(handContour, hull);
	vector<Point> ptsHull;
	for (int k = 0; k < hull.size(); k++)
	{
		int i = hull[k];
		ptsHull.push_back(handContour[i]);
	}

	approxPolyDP(largestContours, largestContours, 2.0, false);
	std::pair<cv::Point, double> maxCircle = findMaxInscribedCircle(largestContours, *copy);
	circle(*copy, maxCircle.first, maxCircle.second, cv::Scalar(220, 75, 20), 1, CV_AA);


	int dx, dy;
	dx = r0.x + r0.width / 2;
	dy = r0.y + r0.height / 2;
	Point center(dx, dy);

	// Draw defects
	vector<Vec4i> defects;
	convexityDefects(handContour, hull, defects);
	vector<Point> validPoints;
	for (int k = 0; k < defects.size(); k++)
	{
		Vec4i v = defects[k];
		Point p1 = handContour[v[0]];
		Point p2 = handContour[v[1]];
		Point p3 = handContour[v[2]];
		double inAngle = InnerAngle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
		double length = std::sqrt(std::pow(p1.x - p3.x, 2) + std::pow(p1.y - p3.y, 2));
		double angle = std::atan2(center.y - p1.y, center.x - p1.x) * 180 / CV_PI;

		if (angle > -30 && angle < 160 && abs(inAngle) > 0 && abs(inAngle) < 120 && length > 0.1 * r0.height)
		{
			validPoints.push_back(p1);
		}
	}
	for (size_t i = 0; i < validPoints.size(); i++)
	{
		cv::circle(*copy, validPoints[i], 9, cv::Scalar(50, 255, 100), 2);
	}

	drawContours(*copy, vector<vector<Point>>(1, ptsHull), 0, Scalar(255, 0, 0), 2);

}

void HandDetection(Mat *depthFrame)
{
	Mat grey, inRangeFrame;
	cvtColor(*depthFrame, grey, CV_RGB2GRAY);
	GaussianBlur(grey, grey, Size(35, 35), 0);
	inRange(grey, 10, 127, inRangeFrame);
	imshow("InRange", inRangeFrame);

	std::vector<std::vector<cv::Point>> contours_f;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(inRangeFrame, contours_f, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	if (contours_f.size() != 0) {
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


		// Draw tracking rectangle
		Rect r0 = boundingRect((contours_f[largestComp]));
		DrawTrackingRectangle(depthFrame, contours_f[largestComp]);

		// Draw hull
		DrawHullAndDefects(depthFrame, contours_f[largestComp], r0);


		imshow("Source", *depthFrame);
	}


}

int main(int argc, const char * argv[])
{
	// Create a VideoCapture object to read from video file
	VideoCapture cap(0);

	Mat frame;

	Mat roi;

	vector<vector<Point> > contours;

	// Play the video in a loop till it ends
	while (char(waitKey(33)) != 'q' && cap.isOpened()) {
		cap >> frame;
		rectangle(frame, Point(200, 40), Point(470, 270), Scalar(0, 255, 0), 2);
		imshow("frame", frame);

		roi = frame(Rect(200, 40, 270, 230));

		HandDetection(&roi);


	}

	return 0;

}


