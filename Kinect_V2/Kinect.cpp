#include "Kinect.h"
using namespace cv;
int pos[3];

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
			nDepthMaxDistance = USHRT_MAX;
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

void onMouse(int event, int x, int y, int flags, void*param)
{
	if (event == EVENT_LBUTTONDOWN) {
		std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
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
			std::cout << "standard depth : " << GetStandardDepth() << std::endl;
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
	standard_depth = depth; std::cout << depth << std::endl;
}

void Kinect::HandDetection(Mat *DepthImage, Mat *inRangeImage)
{
	int inAngleMin = 200, inAngleMax = 300, angleMin = 180, angleMax = 359, lengthMin = 10, lengthMax = 80;

	Scalar lowerb(0, 242, 255);
	Scalar upperb(0, 242, 255);
	inRange(*DepthImage, lowerb, upperb, *inRangeImage);
	erode(*inRangeImage, *inRangeImage, Mat());
	dilate(*inRangeImage, *inRangeImage, cv::Mat(), Point(-1, -1), 2);

	std::vector<std::vector<cv::Point>> contours_f;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(*inRangeImage, contours_f, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	size_t largestContour = 0;
	for (size_t i = 1; i < contours_f.size(); i++)
	{
		if (cv::contourArea(contours_f[i]) > cv::contourArea(contours_f[largestContour]))
			largestContour = i;
	}

	cv::drawContours(*inRangeImage, contours_f, largestContour, cv::Scalar(255, 0, 0), 1);

	//convex hull
	if (!contours_f.empty())
	{
		//calculate the convex hull of largest contour(in order to speed the process)
		std::vector<std::vector<cv::Point>> hull(1);
		cv::convexHull(cv::Mat(contours_f[largestContour]), hull[0], false);
		cv::drawContours(*inRangeImage, hull, 0, cv::Scalar(255, 0, 0), 2);

		//use convexDefects function.
		//The "convexDefects" will try to approximate those gaps using straight lines.
		//we can then use that information to fine the points where our fingertips are placed.

		if (hull[0].size() > 2)
		{
			std::vector<int> hullIndexes;
			cv::convexHull(cv::Mat(contours_f[largestContour]), hullIndexes, true);
			std::vector<cv::Vec4i> convexityDefects;
			cv::convexityDefects(cv::Mat(contours_f[largestContour]), hullIndexes, convexityDefects);

			//pointer filtering
			cv::Rect boundingBox = cv::boundingRect(hull[0]);
			cv::rectangle(*inRangeImage, boundingBox, cv::Scalar(255, 0, 0));
			cv::Point center = cv::Point(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2);
			std::vector<cv::Point> validPoints;

			for (size_t i = 0; i < convexityDefects.size(); i++)
			{
				cv::Point p1 = contours_f[largestContour][convexityDefects[i][0]];
				cv::Point p2 = contours_f[largestContour][convexityDefects[i][1]];
				cv::Point p3 = contours_f[largestContour][convexityDefects[i][2]];
				double angle = std::atan2(center.y - p1.y, center.x - p1.x) * 180 / CV_PI;
				double inAngle = innerAngle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
				double length = std::sqrt(std::pow(p1.x - p3.x, 2) + std::pow(p1.y - p3.y, 2));
				if (angle > angleMin - 180 && angle < angleMax - 180 && inAngle > inAngleMin - 180 && inAngle < inAngleMax - 180 && length > lengthMin / 100.0 * boundingBox.height && length < lengthMax / 100.0 * boundingBox.height)
				{
					validPoints.push_back(p1);
				}
			}
			for (size_t i = 0; i < validPoints.size(); i++)
			{
				cv::circle(*inRangeImage, validPoints[i], 9, cv::Scalar(255, 255, 255), 3);

				imshow("Hand", *inRangeImage);
				std::cout << validPoints.size() << std::endl;
			}

		}
	}
}

float Kinect::innerAngle(float px1, float py1, float px2, float py2, float cx1, float cy1)
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