#include "Kinect.h"
#include <cmath>
using namespace cv;

#define PI 3.141592

using namespace std;

Kinect::Kinect()
{
	m_pKinectSensor = NULL;
	m_pDepthFrameReader = NULL;
	m_pDepthRGBX = new RGBQUAD[cDepthWidth * cDepthHeight];
}

Kinect::~Kinect()
{
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();// close the Kinect Sensor
	}
	SafeRelease(m_pKinectSensor);

	SafeRelease(m_pDepthFrameReader);// done with color frame reader

	if (m_pDepthRGBX)
	{
		delete[] m_pDepthRGBX;
		m_pDepthRGBX = NULL;
	}
}

void Kinect::InitKinect()
{
	HRESULT hr;

	//get the Kinect sensor
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		printf("Failed to find the Kinect sensor!\n");
		exit(10);
	}

	//get the depth frame source & depth frame reader
	if (m_pKinectSensor)
	{
		IDepthFrameSource* pDepthFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		//release depth frame source
		SafeRelease(pDepthFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		printf("No ready Kinect found! \n");
		exit(10);
	}
}

void Kinect::UpdateInfo()
{
	if (!m_pDepthFrameReader)
	{
		return;
	}

	IDepthFrame* pDepthFrame = NULL;

	HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

	if (SUCCEEDED(hr))
	{
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxDistance = 0;
		UINT nBufferSize = 0;
		UINT16 *pBuffer = NULL;

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		}

		if (SUCCEEDED(hr))
		{
			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			nDepthMaxDistance = USHRT_MAX;

			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
			//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}

		if (SUCCEEDED(hr))
		{
			ProcessDepth(pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
			CalculateDepth(pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pDepthFrame);
}

int pos[3];
void onMouse(int event, int x, int y, int flags, void*param)
{
	if (event == EVENT_LBUTTONDOWN) {
		//std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
		pos[0] = x;
		pos[1] = y;
		pos[2] = 1;
	}
}

void Kinect::CalculateDepth(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	const UINT16* p = pBuffer;
	UINT16 depth;

	// Make sure we've received valid data
	if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
	{
		RGBQUAD* pRGBX = m_pDepthRGBX;

		// end pixel is start + width*height - 1
		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		while (pBuffer < pBufferEnd)
		{
			USHORT depth = *pBuffer;
			// To convert to a byte, we're discarding the most-significant
			// rather than least-significant bits.
			// We're preserving detail, although the intensity will "wrap."
			// Values outside the reliable depth range are mapped to 0 (black).
			// Note: Using conditionals in this loop could degrade performance.
			// Consider using a lookup table instead when writing production code.
			BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);
			pRGBX->rgbRed = intensity;
			pRGBX->rgbGreen = intensity;
			pRGBX->rgbBlue = intensity;

			++pRGBX;
			++pBuffer;
		}

		// Draw the data with OpenCV
		Mat ForCalculateImage(nHeight, nWidth, CV_8UC4, m_pDepthRGBX);

		Mat show = ForCalculateImage.clone();
		imshow("ForCalculateImage", show);

		setMouseCallback("ForCalculateImage", onMouse, (void*)&show);
		if (pos[2] == 1) {
			depth = p[pos[0] + pos[1] * 512];
			SetStandardDepth(depth);
			//   std::cout << "standard depth : " << GetStandardDepth() << std::endl;
			pos[2] = 0;
		}

	}
}

void Kinect::ProcessDepth(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	const UINT16* p = pBuffer;
	// Make sure we've received valid data
	if (m_pDepthRGBX && pBuffer && (nWidth == cDepthWidth) && (nHeight == cDepthHeight))
	{
		RGBQUAD* pRGBX = m_pDepthRGBX;

		// end pixel is start + width*height - 1
		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		while (pBuffer < pBufferEnd)
		{
			USHORT depth = *pBuffer;
			SetDepthRGB(pRGBX, depth);
			++pRGBX;
			++pBuffer;
		}

		// Draw the data with OpenCV
		Mat DepthImage(nHeight, nWidth, CV_8UC4, m_pDepthRGBX);
		Mat bImage;

		HandDetection(&DepthImage, &bImage);

		medianBlur(DepthImage, DepthImage, 9);

		Mat edge, draw;
		Canny(DepthImage, edge, 50, 150, 3);
		edge.convertTo(draw, CV_8U);
		// 

		std::vector<std::vector<cv::Point>> contours;
		findContours(draw, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		drawContours(DepthImage, contours, -1, Scalar(0, 0, 0), 3);

		Mat show = DepthImage.clone();
		imshow("DepthImage", show);
	}
}

void Kinect::SetDepthRGB(RGBQUAD* pRGBX, USHORT depth)
{
	BYTE R, G, B;
	UINT std_depth = GetStandardDepth();

	if (std_depth < 600)
		std_depth = 600;
	if (depth < std_depth)
	{
		pRGBX->rgbBlue = 65;
		pRGBX->rgbGreen = 54;
		pRGBX->rgbRed = 233;
	}

	else if (depth < std_depth + 100 && depth > std_depth)
	{
		pRGBX->rgbBlue = 0;
		pRGBX->rgbGreen = 255;
		pRGBX->rgbRed = 153;
	}


	else if (depth < std_depth + 200 && std_depth + 100)
	{
		pRGBX->rgbBlue = 221;
		pRGBX->rgbGreen = 218;
		pRGBX->rgbRed = 166;
	}

	else if (depth <std_depth + 300 && depth > std_depth + 200)
	{
		pRGBX->rgbBlue = 0;
		pRGBX->rgbGreen = 242;
		pRGBX->rgbRed = 255;
	}
	else if (depth <std_depth + 400 && depth > std_depth + 300)
	{
		pRGBX->rgbBlue = 153;
		pRGBX->rgbGreen = 0;
		pRGBX->rgbRed = 255;
	}
	else if (depth > std_depth + 400)
	{
		pRGBX->rgbBlue = 255;
		pRGBX->rgbGreen = 255;
		pRGBX->rgbRed = 255;
	}
}

void Kinect::SetStandardDepth(UINT16 depth)
{
	//standard_depth = depth; std::cout << depth << std::endl;
}

void Kinect::HandDetection(Mat *DepthImage, Mat *inRangeImage)
{

	int inAngleMin = 200, inAngleMax = 300, angleMin = 180, angleMax = 359, lengthMin = 10, lengthMax = 80;

	Scalar lowerb(0, 255, 153);
	Scalar upperb(0, 255, 153);
	inRange(*DepthImage, lowerb, upperb, *inRangeImage);
	erode(*inRangeImage, *inRangeImage, Mat());
	dilate(*inRangeImage, *inRangeImage, cv::Mat(), Point(-1, -1), 2);
	Mat temp = (*inRangeImage).clone();
	temp = Scalar(0);


	showimgcontours(*inRangeImage, *DepthImage);



	std::vector<std::vector<cv::Point>> contours_f;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(*inRangeImage, contours_f, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));


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

		//std::cout << "Max area: " << maxArea << std::endl;
		//drawContours(*inRangeImage, contours_f, largestComp, Scalar(255, 255, 255), 2, 8, hierarchy);
		drawContours(*inRangeImage, contours_f, largestComp, cv::Scalar(255, 255, 255), CV_FILLED, 8, hierarchy);
		Rect r0 = boundingRect(Mat(contours_f[largestComp]));
		std::pair<cv::Point, double> maxCircle = findMaxInscribedCircle(contours_f[largestComp], *DepthImage);
		circle(temp, maxCircle.first, maxCircle.second + 25, cv::Scalar(225, 255, 255), 2, CV_AA);
		cv::Mat andImg;
		cv::bitwise_and(*inRangeImage, temp, andImg);
		Mat gray;
		Mat src_mat;
		Mat canny_mat;
		Canny(andImg, canny_mat, 30, 128, 3, false);

		std::vector<std::vector<cv::Point>> fingerContours;

		findContours(canny_mat, fingerContours, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		int counting = fingerContours.size() - 1;
		char buffer[64] = { 0 };
		std::cout << "counting: " << counting << std::endl;
		sprintf(buffer, "finger: %d", counting);
		putText(*DepthImage, buffer, Point(300, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 3, 8);

		imshow("AND", andImg);


		int dx, dy;
		dx = r0.x + r0.width / 2;
		dy = r0.y + r0.height / 2;
		Point center(dx, dy);
		//std::cout << "x : " << dx << "y : " << dy << std::endl;
		std::ostringstream foo(std::ostringstream::ate);

		foo.str("center is");
		foo << dx << "," << dy;
		palmCenter = center;
		//      circle(*inRangeImage, center, 1, Scalar(255, 0, 0), 5);
		/*      circle(*DepthImage, center, radius_hand, Scalar(0, 255, 255), 5);*/
		//cv::putText(image, foo.str(), center, FONT_HERSHEY_SIMPLEX, 1, 1);
		//   putText(*DepthImage, foo.str(), center, FONT_HERSHEY_SIMPLEX, 1, Scalar::all(0), 1, 8);
		namedWindow("Contour", 0);

		imshow("Contour", *inRangeImage);
		imshow("Source", *DepthImage);
		imshow("circle", temp);
	}
}


std::pair<cv::Point, double> Kinect::findMaxInscribedCircle(const vector<Point>& polyCurves, const cv::Mat& frame)
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


void Kinect::showimgcontours(Mat &threshedimg, Mat &original)
{
	std::vector<std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;
	int largest_area = 0;
	int largest_contour_index = 0;

	findContours(threshedimg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	/// Find the convex hull,contours and defects for each contour
	std::vector<std::vector<Point> >hull(contours.size());
	std::vector<std::vector<int> >inthull(contours.size());
	std::vector<std::vector<Vec4i> >defects(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		convexHull(Mat(contours[i]), hull[i], false);
		convexHull(Mat(contours[i]), inthull[i], false);
		if (inthull[i].size()>3)
			convexityDefects(contours[i], inthull[i], defects[i]);
	}
	//find  hulland contour and defects end here
	//this will find largest contour
	for (int i = 0; i < contours.size(); i++) // iterate through each contour.
	{
		double a = contourArea(contours[i], false);  //  Find the area of contour
		if (a > largest_area)
		{
			largest_area = a;
			largest_contour_index = i;                //Store the index of largest contour
		}

	}
	//search for largest contour has end

	if (contours.size() > 0)
	{
		drawContours(original, contours, largest_contour_index, CV_RGB(0, 255, 0), 2, 8, hierarchy);
		//if want to show all contours use below one
		//drawContours(original,contours,-1, CV_RGB(0, 255, 0), 2, 8, hierarchy);
		drawContours(original, hull, largest_contour_index, CV_RGB(0, 0, 255), 2, 8, hierarchy);
		//if want to show all hull, use below one
		//drawContours(original,hull,-1, CV_RGB(0, 255, 0), 2, 8, hierarchy);
		condefects(defects[largest_contour_index], contours[largest_contour_index], original, contours);
	}

	imshow("convexity", original);
}

double dist(Point x, Point y)
{
	return (x.x - y.x)*(x.x - y.x) + (x.y - y.y)*(x.y - y.y);
}


void Kinect::condefects(std::vector<Vec4i> convexityDefectsSet, std::vector<Point> mycontour, Mat &original, std::vector< std::vector< cv::Point>> contours)
{
	if (contourArea(mycontour) > 3000)
	{
		std::vector<std::pair<cv::Point, double>> palm_centers;
		Point longest;
		double distance;
		double maxDistance = 0;
		int maxIndex = -1;
		for (size_t i = 0; i < convexityDefectsSet.size(); i++)
		{
			int startIdx = convexityDefectsSet[i].val[0];
			Point p1(mycontour[startIdx]);
			distance = sqrt(pow(p1.x - palmCenter.x, 2) + pow(p1.y - palmCenter.y, 2));

			if (distance<50 && distance > maxDistance)
			{
				maxDistance = distance;
				maxIndex = i;
			}
		}

		//std::cout << distance << std::endl;
		Point prevPoint = convexityDefectsSet[0].val[0];
		Point prevDefect = convexityDefectsSet[0].val[0];


		vector<std::vector<int> >hull(contours.size());
		vector<vector<Vec4i>> convDef(contours.size());
		vector<vector<Point>> defect_points(contours.size());
		vector<Point> redDot;

		for (int i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i])>3000)
			{
				convexHull(contours[i], hull[i], false);
				convexityDefects(contours[i], hull[i], convDef[i]);

				for (int k = 0; k < convDef[i].size(); k++)
				{
					if (convDef[i][k][3]> 20 * 256) // filter defects by depth
					{
						int ind_0 = convDef[i][k][0];
						int ind_1 = convDef[i][k][1];
						int ind_2 = convDef[i][k][2];
						defect_points[i].push_back(contours[i][ind_2]);
						cv::circle(original, contours[i][ind_0], 5, Scalar(0, 0, 0), -1);   //손 꼭지점
						//cv::circle(original, contours[i][ind_1], 5, Scalar(255, 0, 0), -1);
						//cv::circle(original, contours[i][ind_2], 5, Scalar(0, 0, 255), -1);
						redDot.push_back(contours[i][ind_2]);
					}
				}
			}
		}

		//std::cout << "finger: " <<redDot.size() << std::endl;

		Point2f points[4];
		RotatedRect rotate_rect;
		rotate_rect = minAreaRect(mycontour);
		rotate_rect.points(points);
		std::vector< std::vector< Point> > polylines;
		polylines.resize(1);
		for (int j = 0; j < 4; ++j)
			polylines[0].push_back(points[j]);
		/*cv::polylines(original, polylines, true, Scalar(255, 255, 0), 2);*/

		double width;
		Point2f widthP1;
		Point2f widthP2;
		int x = 0;
		int widthP1Idx1;
		int widthP1Idx2;
		if (Distance(points[x], points[x + 1]) > Distance(points[x + 1], points[x + 2]))
		{
			width = Distance(points[x + 1], points[x + 2]);
			widthP1Idx1 = x + 1;
			widthP1Idx2 = x + 2;
		}
		else
		{
			width = Distance(points[x], points[x + 1]);
			//          widthP1 = points[x];
			//          widthP2 = points[x + 1];
			widthP1Idx1 = x;
			widthP1Idx2 = x + 1;
		}

		int compIdx = 0;
		for (int i = 0; i < redDot.size(); i++)
		{
			if (DistanceLinePoint(points[widthP1Idx1], points[widthP1Idx2], redDot[i]) < DistanceLinePoint(points[(widthP1Idx1 + 2) % 4], points[(widthP1Idx2 + 2) % 4], redDot[i]))
				compIdx++;
		}

		if (compIdx > (redDot.size() / 2 + 1))
		{
			widthP1 = points[widthP1Idx1];
			widthP2 = points[widthP1Idx2];
		}
		else
		{
			widthP1Idx1 = (widthP1Idx1 + 2) % 4;
			widthP1Idx2 = (widthP1Idx2 + 2) % 4;
			widthP1 = points[widthP1Idx1];
			widthP2 = points[widthP1Idx2];
		}

		width = Distance(widthP1, widthP2);

		line(original, widthP1, widthP2, Scalar(0, 0, 0), 3);

		double max = 0;
		int maxIdx = -1;

		for (int i = 0; i < redDot.size(); i++)
		{
			double compare = DistanceLinePoint(widthP1, widthP2, redDot[i]);
			if (compare>max)
			{
				max = compare;
				maxIdx = i;
			}
		}
		Point belowPt;
		//Point centerSmallRect;

		if (maxIdx >= 0)
		{
			//circle(original, redDot[maxIdx], 5, Scalar(0, 0, 0), 2);
			belowPt = redDot[maxIdx];

			//widthPt1, widthPt2, returnP1, return P2 -> center


			double x;
			double y;
			if (widthP1.y > widthP2.y)
			{
				y = widthP1.y - widthP2.y;
				x = widthP1.x - widthP2.x;
			}
			else if (widthP1.y < widthP2.y)
			{
				y = widthP2.y - widthP1.y;
				x = widthP2.x - widthP1.x;
			}
			else
			{
				y = 0;
				x = 0;
			}

			double result = atan2(y, x) * 180 / PI;

			Point2f rotatePt1 = FindIntersect(points[widthP1Idx1], points[(widthP1Idx1 - 1) % 4], tan(result*PI / 180), belowPt);
			Point2f rotatePt2 = FindIntersect(points[widthP1Idx2], points[(widthP1Idx2 + 1) % 4], tan(result*PI / 180), belowPt);

			double centerX = (widthP1.x + widthP2.x + rotatePt1.x + rotatePt2.x) / 4.0;
			double centerY = (widthP1.y + widthP2.y + rotatePt1.y + rotatePt2.y) / 4.0;
			//      RotatedRect rRect = RotatedRect(Point2f(centerX, centerY), Size2f(width, max/2), rotate_rect.angle);
			//      Point2f vertices[4];
			//      rRect.points(vertices);

			//           RotatedRect sRect = RotatedRect(Point2f(widthP1.x, widthP1.y), Point2f(widthP2.x, widthP2.y), Point2f(rotatePt1.x, rotatePt1.y));
			//           Point2f vertices[4];
			//           sRect.points(vertices);
			//           std::vector< std::vector< Point> > sPolylines;
			//           sPolylines.resize(1);
			//           for (int j = 0; j < 4; ++j)
			//              sPolylines[0].push_back(points[j]);
			//           cv::polylines(original, sPolylines, true, Scalar(255, 0, 0), 2);

			std::vector<cv::Point> palm;
			palm.push_back(Point(widthP1.x - 2, widthP1.y + 2));
			palm.push_back(Point(widthP2.x + 2, widthP2.y + 2));
			palm.push_back(Point(rotatePt2.x + 2, rotatePt2.y));
			palm.push_back(Point(rotatePt1.x - 1, rotatePt1.y));

			const cv::Point *pts = (const cv::Point*)Mat(palm).data;
			int npts = Mat(palm).rows;

			cv::polylines(original, &pts, &npts, 1, true, Scalar(255, 0, 0), 2, 8, 0);

			//circle(original, Point(centerX, centerY), 10, Scalar(0, 0, 0), 3);
			//line(original, rotatePt1, rotatePt2, Scalar(0, 0, 255), 3);

			int fingerCount = 0;


		}

	}
}// condefects ends here


double Kinect::Distance(const Point& p1, const Point& p2) {

	double distance;

	// 피타고라스의 정리
	// pow(x,2) x의 2승,  sqrt() 제곱근
	distance = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));

	return distance;
}

double Kinect::DistanceLinePoint(const Point p1, const Point p2, const Point c)
{
	double denominator;
	double numerator;

	denominator = abs((p2.x - p1.x)*(c.y - p1.y) - (p2.y - p1.y)*(c.x - p1.x));
	numerator = sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y));

	if (numerator != 0)
	{
		return denominator / numerator;
	}
}

Point2f Kinect::FindIntersect(const Point p1, const Point p2, double angle, Point c)
{
	double slop = 0;
	double yIntercept = 0;

	if ((p1.x - p2.x) != 0)
		slop = (p1.y - p2.y) / (p1.x - p2.x);
	yIntercept = p2.y - slop*p2.x;

	double slopCriteria = 0;
	double yCritera = 0;

	slopCriteria = angle;
	yCritera = c.y - slopCriteria*c.x;

	Point2f interPt;
	if ((slop - slopCriteria) != 0)
	{
		interPt.x = (-(yIntercept - yCritera) / (slop - slopCriteria));
		interPt.y = (slop*yCritera - slopCriteria*yIntercept) / (slop - slopCriteria);
		return interPt;
	}
}

float Kinect::innerAngle(Point p1, Point p2, Point c)
{

	float dist1 = sqrt((p1.x - c.x)*(p1.x - c.x) + (p1.y - c.y)*(p1.y - c.y));
	float dist2 = sqrt((p2.x - c.x)*(p2.x - c.x) + (p2.y - c.y)*(p2.y - c.y));

	float Ax, Ay;
	float Bx, By;
	float Cx, Cy;

	//find closest point to C
	//printf("dist = %lf %lf\n", dist1, dist2);

	Cx = c.x;
	Cy = c.y;
	if (dist1 < dist2)
	{
		Bx = p1.x;
		By = p1.y;
		Ax = p2.x;
		Ay = p2.y;


	}
	else {
		Bx = p2.x;
		By = p2.y;
		Ax = p1.x;
		Ay = p1.y;
	}


	float Q1 = Cx - Ax;
	float Q2 = Cy - Ay;
	float P1 = Bx - Ax;
	float P2 = By - Ay;


	float A = acos((P1*Q1 + P2*Q2) / (sqrt(P1*P1 + P2*P2) * sqrt(Q1*Q1 + Q2*Q2)));

	A = A * 180 / PI;

	return A;
}