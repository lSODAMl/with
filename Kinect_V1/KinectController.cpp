#include "KinectController.h"

KinectController::KinectController()
{
}

KinectController::~KinectController()
{
}

void KinectController::InitKinect()
{
	HRESULT hr = NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);

	if (SUCCEEDED(hr))
	{
		kModel.NextDepthFrameEvent(CreateEvent(NULL, TRUE, FALSE, NULL));
		kModel.NextColorFrameEvent(CreateEvent(NULL, TRUE, FALSE, NULL));
		kModel.NextIRFrameEvent(CreateEvent(NULL, TRUE, FALSE, NULL));

		hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480, 0, 2, kModel.NextDepthFrameEvent(), kModel.DepthStreamHandleAddress());
		hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, kModel.NextColorFrameEvent(), kModel.ColorStreamHandleAddress());
		//hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR_INFRARED , NUI_IMAGE_RESOLUTION_640x480, 0, 2, kModel.NextIRFrameEvent(), kModel.IRStreamHandleAddress());

		if (FAILED(hr))
		{
			std::cout << "Could not open Stream" << std::endl;
			exit(10);
		}
	}

	if (FAILED(hr))
	{
		std::cout << "Failed to find sensor!" << std::endl;
		exit(10);
	}
}

BYTE * KinectController::UpdColorKinect()
{
	WaitForSingleObject(kModel.NextColorFrameEvent(), 0);
	HRESULT hr = NuiImageStreamGetNextFrame(kModel.ColorStreamHandle(), 1000, kModel.ColorFrameAddress());
	 
	if (FAILED(hr))
		std::cout << "Create Color Image Failed" << std::endl;

	INuiFrameTexture *pTexture = kModel.ColorFrame()->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect(0, &LockedRect, NULL, 0);

	if (LockedRect.Pitch != 0)
	{
	
		BYTE *pBuffer = (BYTE*)LockedRect.pBits;
		return pBuffer;
	}
	return NULL;
}

BYTE * KinectController::UpdDepthKinect()
{
	WaitForSingleObject(kModel.NextDepthFrameEvent(), 0);
	HRESULT hr = NuiImageStreamGetNextFrame(kModel.DepthStreamHandle(), 1000, kModel.DepthFrameAddress());

	if (FAILED(hr))
		std::cout << "Create Depth Image Failed" << std::endl;

	INuiFrameTexture *pTexture = kModel.DepthFrame()->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect(0, &LockedRect, NULL, 0);
	
	if (LockedRect.Pitch != 0)
	{
		BYTE *pBuffer = (BYTE*)LockedRect.pBits;
		return pBuffer;
	}
	return NULL;
}

BYTE * KinectController::UpdIRKinect()
{
	WaitForSingleObject(kModel.NextIRFrameEvent(), 0);
	HRESULT hr = NuiImageStreamGetNextFrame(kModel.IRStreamHandle(), 1000, kModel.IRFrameAddress());

	if (FAILED(hr))
		std::cout << "Create Depth Image Failed" << std::endl;

	INuiFrameTexture *pTexture = kModel.IRFrame()->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture->LockRect(0, &LockedRect, NULL, 0);

	if (LockedRect.Pitch != 0)
	{
		BYTE *pBuffer = (BYTE*)LockedRect.pBits;
		return pBuffer;
	}
	return NULL;
}

void KinectController::RunKinect()
{
	BYTE *pBuffer = NULL;
	
	// Depth
	
	pBuffer = UpdDepthKinect();

	if (pBuffer != NULL)
	{
		SetDepth((USHORT*)pBuffer);
		cv::Mat depthImage(GetHeight(), GetWidth(), CV_8UC4, GetRGBX());
		cv::imshow("depthImage", depthImage);
	}

	// Color
	
	pBuffer = UpdColorKinect();

	if (pBuffer != NULL)
	{
		cv::Mat colorImage(GetHeight(), GetWidth(), CV_8UC4, pBuffer);
		cv::imshow("colorImage", colorImage);
	}
	
	// IR
	/*
	pBuffer = UpdIRKinect();

	if (pBuffer != NULL)
	{
		SetIR((USHORT*)pBuffer);
		cv::Mat IRImage(GetHeight(), GetWidth(), CV_8UC4, GetRGBX());
		cv::imshow("IRImage", IRImage);
	}
	*/
}

void KinectController::MaintainKinect()
{
	NuiImageStreamReleaseFrame(kModel.DepthStreamHandle(), kModel.DepthFrame());
	NuiImageStreamReleaseFrame(kModel.ColorStreamHandle(), kModel.ColorFrame());
	NuiImageStreamReleaseFrame(kModel.IRStreamHandle(), kModel.IRFrame());
}

void KinectController::SetDepth(USHORT * pBufferRun)
{
	RGBQUAD *pRGBX = kModel.DepthRGBX();
	for (int y = 0; y < kModel.kinectHeight; y++)
	{
		for (int x = 0; x < kModel.kinectWidth; x++)
		{
			USHORT depth = *pBufferRun;
			SetRGB(pRGBX, depth);
			pBufferRun++;
			pRGBX++;
		}
	}
}

void KinectController::SetColor(USHORT * pBufferRun)
{
	cv::Mat colorImage(GetHeight(), GetWidth(), CV_8UC4, pBufferRun);
	cv::imshow("CCC", colorImage);
}

void KinectController::SetRGB(RGBQUAD *pRGBX, USHORT depth)
{
	depth = (depth & 0xfff8) >> 3;

	UINT std_depth = 0;

	if (std_depth < 1000)
		std_depth = 1000;
	if (depth < std_depth)
	{
		pRGBX->rgbBlue = 255;
		pRGBX->rgbGreen = 0;
		pRGBX->rgbRed = 0;
	}

	else if (depth < std_depth + 100 && depth > std_depth)
	{
		pRGBX->rgbBlue = 0;
		pRGBX->rgbGreen = 255;
		pRGBX->rgbRed = 0;
	}


	else if (depth < std_depth + 200 && std_depth + 100)
	{
		pRGBX->rgbBlue = 0;
		pRGBX->rgbGreen = 0;
		pRGBX->rgbRed = 255;
	}

	else if (depth <std_depth + 300 && depth > std_depth + 200)
	{
		pRGBX->rgbBlue = 255;
		pRGBX->rgbGreen = 0;
		pRGBX->rgbRed = 0;
	}
	else if (depth <std_depth + 400 && depth > std_depth + 300)
	{
		pRGBX->rgbBlue = 0;
		pRGBX->rgbGreen = 255;
		pRGBX->rgbRed = 0;
	}
	else if (depth > std_depth + 400)
	{
		pRGBX->rgbBlue = 255;
		pRGBX->rgbGreen = 255;
		pRGBX->rgbRed = 255;
	}
}

void KinectController::SetIR(USHORT * pBufferRun)
{
	RGBQUAD *pRGBX = kModel.DepthRGBX();
	for (int y = 0; y < kModel.kinectHeight; y++)
	{
		for (int x = 0; x < kModel.kinectWidth; x++)
		{
			SHORT pixel = *pBufferRun >> 8;
			BYTE intensity = pixel;
			pRGBX->rgbBlue = pRGBX->rgbGreen = pRGBX->rgbRed = intensity;
			pBufferRun++;
			pRGBX++;
		}
	}
}

RGBQUAD* KinectController::GetRGBX()
{
	return kModel.DepthRGBX();
}

int KinectController::GetWidth()
{
	return kModel.kinectWidth;
}

int KinectController::GetHeight()
{
	return kModel.kinectHeight;
}
