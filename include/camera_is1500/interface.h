
#ifdef WIN32
#include "windows.h"
#endif
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <unistd.h>
#include "libsfaccess.h"

#include <string>
#include <vector>

using namespace std;

bool overInit();
vector<float> overGetData();
// vector<float>* overGetDataPtr();
bool overClose();
