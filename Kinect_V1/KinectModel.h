#pragma once
#include <Windows.h>
#include <NuiApi.h>
#include <iostream>

class KinectModel
{
public:
	KinectModel();
	~KinectModel();

public:
	RGBQUAD* DepthRGBX() const { return depthRGBX; }
	void DepthRGBX(RGBQUAD* val) { depthRGBX = val; }

	HANDLE DepthStreamHandle() const { return depthStreamHandle; }
	HANDLE* DepthStreamHandleAddress() { return &depthStreamHandle; }
	void DepthStreamHandle(HANDLE val) { depthStreamHandle = val; }

	HANDLE ColorStreamHandle() const { return colorStreamHandle; }
	HANDLE* ColorStreamHandleAddress() { return &colorStreamHandle; }
	void ColorStreamHandle(HANDLE val) { colorStreamHandle = val; }

	HANDLE IRStreamHandle() const { return irStreamHandle; }
	HANDLE* IRStreamHandleAddress() { return &irStreamHandle; }
	void IRStreamHandle(HANDLE val) { irStreamHandle = val; }

	HANDLE NextDepthFrameEvent() const { return nextDepthFrameEvent; }
	void NextDepthFrameEvent(HANDLE val) { nextDepthFrameEvent = val; }

	HANDLE NextColorFrameEvent() const { return nextColorFrameEvent; }
	void NextColorFrameEvent(HANDLE val) { nextColorFrameEvent = val; }

	HANDLE NextIRFrameEvent() const { return nextIRFrameEvent; }
	void NextIRFrameEvent(HANDLE val) { nextIRFrameEvent = val; }

	const NUI_IMAGE_FRAME* DepthFrame() const { return depthFrame; }
	NUI_IMAGE_FRAME* DepthFrame(NUI_IMAGE_FRAME* val) { depthFrame = val; }
	const NUI_IMAGE_FRAME** DepthFrameAddress() { return &depthFrame; }

	const NUI_IMAGE_FRAME* ColorFrame() const { return colorFrame; }
	NUI_IMAGE_FRAME* ColorFrame(NUI_IMAGE_FRAME* val) { colorFrame = val; }
	const NUI_IMAGE_FRAME** ColorFrameAddress() { return &colorFrame; }

	const NUI_IMAGE_FRAME* IRFrame() const { return irFrame; }
	NUI_IMAGE_FRAME* IRFrame(NUI_IMAGE_FRAME* val) { irFrame = val; }
	const NUI_IMAGE_FRAME** IRFrameAddress() { return &irFrame; }


public:	
	static const int			kinectWidth = 640;
	static const int		    kinectHeight = 480;

private:
	RGBQUAD*					depthRGBX;
	HANDLE						depthStreamHandle;
	HANDLE						colorStreamHandle;
	HANDLE						irStreamHandle;
	HANDLE						nextDepthFrameEvent;
	HANDLE						nextColorFrameEvent;
	HANDLE						nextIRFrameEvent;
	const NUI_IMAGE_FRAME*		depthFrame;
	const NUI_IMAGE_FRAME*		colorFrame;
	const NUI_IMAGE_FRAME*		irFrame;
};

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

