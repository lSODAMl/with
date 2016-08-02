#pragma once
#include <windows.h>
#include "KinectModel.h"
#include "KinectView.h"
#include "HandController.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

class KinectController
{
public:
	KinectController();
	~KinectController();

public:
	void		InitKinect();
	UINT16 *	UpdateKinect(int &nWidth, int &nHeight);
	RGBQUAD*	Processkinect();
	void		MaintainKinect();
	void		SetDepth(const UINT16* pBuffer, int nWidth, int nHeight);
	void		SetRGB(RGBQUAD* pRGBX, USHORT depth);
	int			GetWidth();
	int			GetHeight();
	RGBQUAD*	GetRGBX();


private:
	KinectModel kModel;
};

