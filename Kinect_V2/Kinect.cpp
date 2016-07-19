#include "Kinect.h"
using namespace cv;

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
		Mat depthImage(nHeight, nWidth, CV_8UC4, m_pDepthRGBX);
		Mat inRangeImage;

		HandDetection(&depthImage, &inRangeImage);
		ShowContourDepthImage(&depthImage);

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



void Kinect::HandDetection(Mat *depthImage, Mat *inRangeImage)
{
	int inAngleMin = 200, inAngleMax = 300, angleMin = 180, angleMax = 359, lengthMin = 10, lengthMax = 80;

	Scalar lowerb(0, 255, 153);
	Scalar upperb(0, 255, 153);
	inRange(*depthImage, lowerb, upperb, *inRangeImage);
	erode(*inRangeImage, *inRangeImage, Mat());
	dilate(*inRangeImage, *inRangeImage, cv::Mat(), Point(-1, -1), 2);
	//imshow("white", *inRangeImage);
	Mat temp = *inRangeImage;
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

		std::cout << "Max area: " << maxArea << std::endl;
		drawContours(*inRangeImage, contours_f, largestComp, Scalar(255, 0, 255), 2, 8, hierarchy);

		Rect r0 = boundingRect(Mat(contours_f[largestComp]));

		rectangle(*inRangeImage, r0, Scalar(255, 0, 0), 2);
		rectangle(*depthImage, r0, Scalar(0, 255, 0), 2);  //Draw rectangle in image




		Point2f points[4];
		Point2f center;
		float radius;
		Rect rect;
		RotatedRect rotate_rect;

		//compute the bounding rect, rotated bounding rect, minum enclosing circle.
		rect = boundingRect(contours_f[largestComp]);
		rotate_rect = minAreaRect(contours_f[largestComp]);
		minEnclosingCircle(contours_f[largestComp], center, radius);
		rotate_rect.points(points);

		std::vector< std::vector< Point> > polylines;
		polylines.resize(1);
		for (int j = 0; j < 4; ++j)
			polylines[0].push_back(points[j]);

		//draw them on the bounding image.
		cv::polylines(*depthImage, polylines, true, Scalar(0, 255, 0), 2);

		int dx, dy;
		dx = r0.x + r0.width / 2;
		dy = r0.y + r0.height / 2;

		Point centerHand(dx, dy);
		std::cout << "x : " << dx << "y : " << dy << std::endl;
		std::ostringstream foo(std::ostringstream::ate);
		foo.str("center is");
		foo << dx << "," << dy;

		circle(*inRangeImage, center, 1, Scalar(255, 0, 0), 5);

		//cv::putText(image, foo.str(), center, FONT_HERSHEY_SIMPLEX, 1, 1);
		putText(*depthImage, foo.str(), centerHand, FONT_HERSHEY_SIMPLEX, 1, Scalar::all(0), 1, 8);
		namedWindow("Contour", 0);

		imshow("Contour", *inRangeImage);
		imshow("Source", *depthImage);
	}


}

void Kinect::ShowContourDepthImage(Mat *depthImage)
{
	medianBlur(*depthImage, *depthImage, 9);
	Mat edge, draw;
	Canny(*depthImage, edge, 50, 150, 3);
	edge.convertTo(draw, CV_8U);
	std::vector<std::vector<cv::Point>> contours;
	findContours(draw, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	drawContours(*depthImage, contours, -1, Scalar(0, 0, 0), 3);

	Mat show = (*depthImage).clone();
	imshow("DepthImage", show);
}