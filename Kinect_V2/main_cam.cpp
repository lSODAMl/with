#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
using namespace cv;
using namespace std;

Point getHandCenter(const Mat& mask, double& radius) {
	Mat dst;
	distanceTransform(mask, dst, CV_DIST_L2, 5);  
	int maxIdx[2];    
	minMaxIdx(dst, NULL, &radius, NULL, maxIdx, mask);   

	return Point(maxIdx[1], maxIdx[0]);
}

float innerAngle(float px1, float py1, float px2, float py2, float cx1, float cy1)
{

	float dist1 = std::sqrt((px1 - cx1)*(px1 - cx1) + (py1 - cy1)*(py1 - cy1));
	float dist2 = std::sqrt((px2 - cx1)*(px2 - cx1) + (py2 - cy1)*(py2 - cy1));

	float Ax, Ay;
	float Bx, By;
	float Cx, Cy;

	//find closest point to C  
	//printf("dist = %lf %lf\n", dist1, dist2);  

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


int main(int argc, const char * argv[])
{
	// Create a VideoCapture object to read from video file
	VideoCapture cap(0);

	Mat frame;
	Mat grey;
	Mat th;
	Mat roi;
	Mat th_copy;

	vector<vector<Point> > contours;

	// Play the video in a loop till it ends
	while (char(waitKey(33)) != 'q' && cap.isOpened()) {
		cap >> frame;
		rectangle(frame, Point(200, 40), Point(470, 270), Scalar(0, 255, 0), 2);
		imshow("frame", frame);

		roi = frame(Rect(200, 40, 270, 230));
		cvtColor(roi, grey, CV_RGB2GRAY);
		GaussianBlur(grey, grey, Size(35, 35), 0);
		cvtColor(frame, frame, CV_RGB2GRAY);
		// input, output, over 127 => 255 else 0
		//threshold(grey, th, 0, 127, CV_THRESH_BINARY);
		inRange(grey, 10, 127, th);
		//imshow("th", th);

		th.copyTo(th_copy);

		findContours(th, contours, RETR_TREE, CV_CHAIN_APPROX_NONE);

		if(contours.size()!=0){
			th = 0;
			int maxK = 0;
			double maxArea = -1;
			for (int k = 0; k < contours.size(); k++)
			{
				double area = contourArea(contours[k]);
				if (area > maxArea)
				{
					maxK = k;
					maxArea = area;
				}
			}


			Rect r0 = boundingRect(Mat(contours[maxK]));
			rectangle(roi, r0, Scalar(0, 0, 255), 2);
			int dx, dy;
			dx = r0.x + r0.width / 2;
			dy = r0.y + r0.height / 2;
			Point center(dx, dy);

			double s;
			getHandCenter(th_copy, s);
// 			if (s > 0)
// 				circle(roi, center, s * 2, Scalar(0, 0, 255), 1.5);

			vector<Point> handContour = contours[maxK];

			vector<int> hull;
			convexHull(handContour, hull);

			vector<Point> ptsHull;
			for (int k = 0; k < hull.size(); k++)
			{
				int i = hull[k];
				ptsHull.push_back(handContour[i]);
			}

			drawContours(roi, vector<vector<Point>>(1, ptsHull), 0, Scalar(255, 0, 0), 2);
			drawContours(th_copy, vector<vector<Point>>(1, ptsHull), 0, Scalar(255, 0, 0), 2);

			vector<Vec4i> defects;
			convexityDefects(handContour, hull, defects);

			vector<Point> validPoints;



			for (int k = 0; k < defects.size(); k++)
			{
				Vec4i v = defects[k];
				Point p1 = handContour[v[0]];
				Point p2 = handContour[v[1]];
				Point p3 = handContour[v[2]];
				double inAngle = innerAngle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
				double length = std::sqrt(std::pow(p1.x - p3.x, 2) + std::pow(p1.y - p3.y, 2));
				double angle = std::atan2(center.y - p1.y, center.x - p1.x) * 180 / CV_PI;

				if (angle > -30 && angle < 160 && abs(inAngle) > 0 && abs(inAngle) < 120&& length > 0.1 * r0.height)
				{
					validPoints.push_back(p1);
				}
			}
			for (size_t i = 0; i < validPoints.size(); i++)
			{
				cv::circle(roi, validPoints[i], 9, cv::Scalar(50, 255, 100), 2);
			}



			imshow("th_copy", th_copy);
			imshow("roi", roi);

	}


	}

	return 0;

}