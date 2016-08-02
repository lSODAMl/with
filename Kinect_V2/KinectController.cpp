#include "KinectController.h"

using namespace cv;

KinectController::KinectController()
{

}

KinectController::~KinectController()
{

}

void KinectController::InitKinect()
{
	HRESULT hr;

	//get sensor
	hr = GetDefaultKinectSensor(kModel.KinectSensorAddress());
	if (FAILED(hr))
	{
		printf("Failed to find sensor!\n");
		exit(10);
	}

	//get the depth frame source & depth frame reader
	if (kModel.KinectSensor())
	{
		IDepthFrameSource* pDepthFrameSource = NULL;

		hr = kModel.KinectSensor()->Open();

		if (SUCCEEDED(hr))
		{
			hr = kModel.KinectSensor()->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(kModel.DepthFrameReaderAddress());
		}

		//release depth frame source
		kModel.SafeRelease(pDepthFrameSource);
	}

	if (!kModel.KinectSensor() || FAILED(hr))
	{
		printf("No ready Device found! \n");
		exit(10);
	}
}

UINT16 * KinectController::UpdateKinect(int &nWidth, int &nHeight)
{
	if (!kModel.DepthFrameReader())
		return false;

	HRESULT hr = kModel.DepthFrameReader()->AcquireLatestFrame(kModel.DepthFrameAddress());

	if (SUCCEEDED(hr))
	{
		UINT nBufferSize = 0;
		UINT16 *pBuffer = NULL;

		if (SUCCEEDED(hr))
			hr = kModel.DepthFrame()->get_FrameDescription(kModel.FrameDescriptionAddress());

		if (SUCCEEDED(hr))
			hr = kModel.FrameDescription()->get_Width(&nWidth);

		if (SUCCEEDED(hr))
			hr = kModel.FrameDescription()->get_Height(&nHeight);

		if (SUCCEEDED(hr))
			hr = kModel.DepthFrame()->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);

		if (SUCCEEDED(hr))
			return pBuffer;
	}
	return NULL;
}

RGBQUAD* KinectController::Processkinect()
{
	int nWidth, nHeight;

	UINT16 *pBuffer = UpdateKinect(nWidth, nHeight);
	if (pBuffer != NULL)
		SetDepth(pBuffer, nWidth, nHeight);

// 	Mat DepthImage(nHeight, nWidth, CV_8UC4, kModel.DepthRGBX());
	return kModel.DepthRGBX();
// 	KinectView::ShowScreen(&kModel);
}

void KinectController::MaintainKinect()
{
	kModel.SafeReleaseAddress(kModel.DepthFrame());
	kModel.DepthFrame(NULL);
	kModel.SafeReleaseAddress(kModel.FrameDescription());
	kModel.FrameDescription(NULL);
}

void KinectController::SetDepth(const UINT16* pBuffer, int nWidth, int nHeight)
{
	// Make sure we've received valid data
	RGBQUAD* m_pDepthRGBX = kModel.DepthRGBX();

	if (m_pDepthRGBX && pBuffer && (nWidth == kModel.cDepthWidth) && (nHeight == kModel.cDepthHeight))
	{
		RGBQUAD* pRGBX = m_pDepthRGBX;

		// end pixel is start + width*height - 1
		const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

		while (pBuffer < pBufferEnd)
		{
			USHORT depth = *pBuffer;
			SetRGB(pRGBX, depth);
			++pRGBX;
			++pBuffer;
		}
	}
}

void KinectController::SetRGB(RGBQUAD* pRGBX, USHORT depth)
{
	BYTE R =0, G = 0, B = 0;
	UINT std_depth = 0;

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

int KinectController::GetWidth()
{
	return kModel.cDepthWidth;
}

int KinectController::GetHeight()
{
	return kModel.cDepthHeight;
}

RGBQUAD* KinectController::GetRGBX()
{
	return kModel.DepthRGBX();
}

