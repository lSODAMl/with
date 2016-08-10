#include "KinectModel.h"

KinectModel::KinectModel()
{
	depthStreamHandle	= INVALID_HANDLE_VALUE;
	colorStreamHandle	= INVALID_HANDLE_VALUE;
	irStreamHandle		= INVALID_HANDLE_VALUE;
	depthRGBX			= new RGBQUAD[kinectWidth*kinectHeight];
	depthFrame			= NULL;
	colorFrame			= NULL;
	irFrame				= NULL;
}

KinectModel::~KinectModel()
{
	NuiShutdown();
	delete[] depthRGBX;
}
