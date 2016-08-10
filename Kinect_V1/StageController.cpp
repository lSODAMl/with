#include "StageController.h"


StageController::StageController()
{
}


StageController::~StageController()
{
}

void StageController::InitStage()
{
	kCtrl.InitKinect();
}

void StageController::RunStage()
{
	InitStage();
	int whatStageChoice = 1;
	bool flag = true;
	while (1)
	{
		if (flag == false) {
			std::cin >> whatStageChoice;
		}

		switch (whatStageChoice)
		{
		case 1:
			Stage1();

			break;
		case 2:
			break;
		case 3:
			break;
		}
	}
}

void StageController::Stage1()
{
	std::cout << "������ �ϳ��� ��(value)�� ����ִ� �ĺ���(identifer) �Դϴ�. ���� ��� �������� ���α׷��� ���� ���� ������� ������, ���Ŀ� ����� �� �ֵ��� ���� 5�� �ʿ��ϴٰ� ������ ���ô�. �������� var�̶�� �Ἥ ������ �����ϰ�, ���⿡ ���� 5�� ��� ���߿� ����� �� �ֵ��� ���� ������ �� �ֽ��ϴ�. ������ ���� ������:" << std::endl;
	std::cout << "var = 5" << std::endl;
	std::cout << "���̽㿡�� ������ ������ �����ϴ�; var ó�� �׳� �̸��� ����ؼ� �ĺ��ڸ� ��Ÿ����, = �����ڸ� ����ؼ� ���ϴ� ���� �����ϸ� ���Դϴ�!"<<std::endl;
	std::cout << "Instrcution" << std::endl << std::endl;
	std::cout << "���� my_var�� �����ϰ� ������ 10�� �����ϼ���." << std::endl;

	bool flag = false;
	while (1) 
	{
		BYTE *pBuffer = NULL;

		// Depth

		pBuffer = kCtrl.UpdDepthKinect();

		if (pBuffer != NULL)
		{
			kCtrl.SetDepth((USHORT*)pBuffer);
			cv::Mat depthImage(kCtrl.GetHeight(),kCtrl.GetWidth(), CV_8UC4, kCtrl.GetRGBX());
			cv::imshow("depthImage", depthImage);
			if (cvWaitKey(10) == 0x001b)
			{
				break;
			}
		}

		kCtrl.MaintainKinect();

		py.InitPython();
		flag = py.Varaibles();
		if (flag == true)
			break;
	}
	Py_Finalize();
}

void StageController::Stage2()
{
}

void StageController::Stage3()
{
}
