#include "Python.h"

Python::Python()
{
}


Python::~Python()
{
}

void Python::InitPython()
{
	Py_Initialize();
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append(\"C:\\scripts\")");
}




bool Python::Varaibles()
{
	bool flag = false;
	PyObject* module = PyImport_ImportModule("Py_Syntax");
	PyImport_ReloadModule(module);

	if (module) {
		PyObject* retModule = PyObject_GetAttrString(module, "Variables");
		if (retModule)
		{
			PyObject *retObj = PyObject_CallFunction(retModule,NULL);
			if (retObj != nullptr && PyLong_Check(retObj) == true)
			{
				int result = (int)PyLong_AsLong(retObj);
				if (result == 10)
				{
					std::cout << result << std::endl;
					flag = true;
				}
			}
			Py_XDECREF(retObj);
			Py_XDECREF(retModule);
		}
		Py_XDECREF(module);
		Py_XDECREF(module);
	}
	return flag;
}

bool Python::DataTypes()
{
	bool flag = false;
	PyObject* module = PyImport_ImportModule("Py_Syntax");
	PyImport_ReloadModule(module);

	if (module) {
		PyObject* retModule = PyObject_GetAttrString(module, "DataTypes");
		if (retModule)
		{
			PyObject *retObj = PyObject_CallFunction(retModule,NULL);
			PyObject *item = PyList_GetItem(retObj, 0);
			int result = (int)PyLong_AsLong(item);
			std::cout << result << std::endl;

			Py_XDECREF(retObj);
			Py_XDECREF(retModule);
		}
		Py_XDECREF(module);
		Py_XDECREF(module);
	}
	return flag;
}



