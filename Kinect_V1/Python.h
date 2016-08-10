#pragma once
#ifdef _DEBUG 
#undef _DEBUG 
#include <Python.h> 
#define _DEBUG 
#else 
#include <Python.h> 
#endif
#include <iostream>
#include <windows.h>
class Python
{
public:
	Python();
	~Python();
public:
	void	InitPython();
	bool	Varaibles();
	bool	DataTypes();
};

