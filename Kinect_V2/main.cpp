#include "KinectController.h"

int main()
{
	KinectController kController;
	HandController hController;
	KinectView	kView;
	kController.InitKinect();

	while (1)
	{
		kController.Processkinect();
		Mat depthImage(kController.GetHeight(), kController.GetWidth(), CV_8UC4, kController.GetRGBX());
		hController.DetectHand(&depthImage);
		kView.ShowScreen(&depthImage);
		kController.MaintainKinect();
		

		if (cv::waitKey(1) >= 0)
		{
			break;
		}
	}
	return 0;
}