#include <python3.8/Python.h>
 
int main() {
    // 初始化Python解释器
    Py_Initialize();
 
    // 添加当前目录到sys.path，以便可以导入本地Python脚本
    PyObject* sysPath = PySys_GetObject("path");
    PyObject* path = PyUnicode_FromString(".");
    PyList_Append(sysPath, path);
    Py_DECREF(path);
 
    // 导入Python脚本
    PyObject* pModule = PyImport_ImportModule("plot_script");
    if (!pModule) {
        PyErr_Print();
        return 1;
    }
 
    // 调用脚本中的函数
    PyObject* pDict = PyModule_GetDict(pModule);
    if (!pDict) {
        PyErr_Print();
        return 1;
    }
 
    PyEval_CallObject(PyDict_GetItemString(pDict, "main"), NULL);
 
    // 关闭Python解释器
    Py_Finalize();
    return 0;
}