#pragma once
#include "KinectModel.h"
#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "opencv2/opencv.hpp"
class KinectController
{
public:
	KinectController();
	~KinectController();

public:
	void		InitKinect();
	BYTE*		UpdColorKinect();
	BYTE*		UpdDepthKinect();
	BYTE*		UpdIRKinect();
	void		RunKinect();
	void		MaintainKinect();
	void		SetDepth(USHORT *pBufferRun);
	void		SetColor(USHORT *pBufferRun);
	void		SetRGB(RGBQUAD *pRGBX, USHORT depth);
	void		SetIR(USHORT * pBufferRun);
	RGBQUAD*	GetRGBX();
	int			GetWidth();
	int			GetHeight();

private:
	KinectModel kModel;
};

