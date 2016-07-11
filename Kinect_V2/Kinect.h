#pragma once
#include "opencv2/opencv.hpp"
#include <windows.h>
#include <Kinect.h>// Kinect Header files
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

class Kinect
{
public:
	Kinect();
	~Kinect();

public:
	void        CalculateDepth(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);
	void		InitKinect();
	void		UpdateInfo();
	void        ProcessDepth(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);
	void		SetDepthRGB(RGBQUAD* pRGBX, USHORT depth);
	void		HandDetection(Mat *depth, Mat *inRangeImage);
	template<class Interface>
	void		SafeRelease(Interface *& pInterfaceToRelease);
	float		innerAngle(float px1, float py1, float px2, float py2, float cx1, float cy1);
	UINT16      GetStandardDepth() { return standard_depth; };
	void		SetStandardDepth(UINT16 depth);


public:
	static const int        cDepthWidth  = 512;
	static const int        cDepthHeight = 424;

private:

	IKinectSensor*          m_pKinectSensor;// Current Kinect
	IDepthFrameReader*      m_pDepthFrameReader;// Color reader
	RGBQUAD*                m_pDepthRGBX;
	UINT16				    standard_depth;

};

template<class Interface>
void Kinect::SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

