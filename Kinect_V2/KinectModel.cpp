#include "KinectModel.h"

KinectModel::KinectModel()
{
	KinectSensor(NULL);
	DepthFrameReader(NULL);
	DepthRGBX(new RGBQUAD[cDepthWidth * cDepthHeight]);
}

KinectModel::~KinectModel()
{
	if (KinectSensor())
	{
		KinectSensor()->Close();	 
	}

	SafeRelease(m_pKinectSensor);
	SafeRelease(m_pDepthFrameReader);

	if (DepthRGBX())
	{
		delete[] DepthRGBX();
		DepthRGBX(NULL);
	}
}


