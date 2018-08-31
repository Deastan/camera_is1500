#ifdef WIN32
#include "windows.h"
#endif
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <unistd.h>
#include "libsfaccess.h"
#include "interface.h"
#include <string>


class Oversfa
{
  private:
    // SfAccess& sfa;
    double nbr;

  public:
    // Member functions definitions including constructor
    Oversfa();


    // Methode
    bool overInit();
    float overGetData();
}; // end of the class
