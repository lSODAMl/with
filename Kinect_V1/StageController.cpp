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
	std::cout << "변수는 하나의 값(value)을 담고있는 식별자(identifer) 입니다. 예를 들어 여러분의 프로그램에 지금 당장 사용하진 않지만, 이후에 사용할 수 있도록 숫자 5가 필요하다고 가정해 봅시다. 여러분은 var이라고 써서 변수를 지정하고, 여기에 숫자 5를 담아 나중에 사용할 수 있도록 값을 유지할 수 있습니다. 다음과 같이 말이죠:" << std::endl;
	std::cout << "var = 5" << std::endl;
	std::cout << "파이썬에서 변수의 선언은 쉽습니다; var 처럼 그냥 이름을 사용해서 식별자를 나타내고, = 연산자를 사용해서 원하는 값을 지정하면 끝입니다!"<<std::endl;
	std::cout << "Instrcution" << std::endl << std::endl;
	std::cout << "변수 my_var을 선언하고 값으로 10을 지정하세요." << std::endl;

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
