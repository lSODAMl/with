#pragma once
#include "KinectController.h"
#include "Python.h"
class StageController
{
public:
	StageController();
	~StageController();

public:
	void InitStage();
	void RunStage();
	void Stage1();
	void Stage2();
	void Stage3();
private:
	Python py;
	KinectController kCtrl;
};

