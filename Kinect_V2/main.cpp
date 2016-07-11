#include "Kinect.h"
using namespace cv;


int main()
{
	Kinect kinect;
	kinect.InitKinect();
	while (1)
	{
		kinect.UpdateInfo();
		if (waitKey(1) >= 0)
		{
			break;
		}
	}

	return 0;
}