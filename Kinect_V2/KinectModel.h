#pragma once
#include <windows.h>
#include <Kinect.h>
#include <iostream>
class KinectModel
{
public:
	KinectModel();
	~KinectModel();

public:
	template<class Interface>
	void	SafeRelease(Interface *& pInterfaceToRelease);

	template<class Interface>
	void	SafeReleaseAddress(Interface* pInterfaceToRelease);

public:
	IKinectSensor* KinectSensor() const { return m_pKinectSensor; }
	IKinectSensor** KinectSensorAddress()  { return &m_pKinectSensor; }
	void KinectSensor(IKinectSensor* val) { m_pKinectSensor = val; }

	IDepthFrameReader* DepthFrameReader() const { return m_pDepthFrameReader; }
	IDepthFrameReader** DepthFrameReaderAddress() { return &m_pDepthFrameReader; }
	void DepthFrameReader(IDepthFrameReader* val) { m_pDepthFrameReader = val; }

	IFrameDescription* FrameDescription() const { return m_pFrameDescription; }
	IFrameDescription** FrameDescriptionAddress() { return &m_pFrameDescription; }
	void FrameDescription(IFrameDescription* val) { m_pFrameDescription = val; }

	IDepthFrame* DepthFrame() const { return m_pDepthFrame; }
	IDepthFrame** DepthFrameAddress() { return &m_pDepthFrame; }
	void DepthFrame(IDepthFrame* val) { m_pDepthFrame = val; }


	RGBQUAD* DepthRGBX() const { return m_pDepthRGBX; }
	void DepthRGBX(RGBQUAD* val) { m_pDepthRGBX = val; }

public:
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;

private:
	IKinectSensor*          m_pKinectSensor;	
	IDepthFrameReader*      m_pDepthFrameReader;
	IFrameDescription*		m_pFrameDescription;
	IDepthFrame*			m_pDepthFrame;
	RGBQUAD*                m_pDepthRGBX;
};

template<class Interface>
void KinectModel::SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

template<class Interface>
void KinectModel::SafeReleaseAddress(Interface* pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
		pInterfaceToRelease->Release();
}

