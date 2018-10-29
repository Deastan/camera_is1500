#include <iostream>
#include "interface.h"
#include <string>
#include <vector>

// int main ()
// {
//   overInit();
//     // v = overGetData();
//    std::cout << "Start the program" << std::endl;
//    // std::vector<float> vNormal;
//    // vNormal.push_back(0);
//    // vNormal.push_back(0);
//    // vNormal.push_back(0);
//    // vNormal.push_back(0);
//    // vNormal.push_back(0);
//    // vNormal.push_back(0);
//    // vNormal = vNormal;
//    // vector<float>&* ptrV1 = overGetData();
//    vector<float> v;
//    vector<float> *ptrV = &v;
//    // std::vector<float> v;
//    // v.push_back(0);
//    // v.push_back(0);
//    // v.push_back(0);
//    // v.push_back(0);
//    // v.push_back(0);
//    // v.push_back(0);
//
//    while(true)
//    {
//
//      // overInit();
//      ptrV = overGetDataPtr();
//      v = *ptrV;
//      std::cout <<  "main.cpp      : " << v[0] << " " << v[1] << " " << v[2] << " "
//      << v[3] << " " << v[4] << " " << v[5] << std::endl;
//      // overClose();
//      // const float* p1 = &v[0];
//      // const float* p2 = &v[1];
//      // const float* p3 = &v[2];
//      // const float* p4 = &v[3];
//      // const float* p5 = &v[4];
//      // const float* p6 = &v[5];
//
//      // std::cout << "***************start*****************" << '\n';
//      // std::cout << "the int at address " << p1 << " is " << *p1 << '\n';
//      // std::cout << "the int at address " << p2 << " is " << *p2 << '\n';
//      // std::cout << "the int at address " << p3 << " is " << *p3 << '\n';
//      // std::cout << "the int at address " << p4 << " is " << *p4 << '\n';
//      // std::cout << "the int at address " << p5 << " is " << *p5 << '\n';
//      // std::cout << "the int at address " << p6 << " is " << *p6 << '\n';
//      // std::cout << "****************end******************" << '\n';
//      // std::cout << "***************start*****************" << '\n';
//      // std::cout << "the int at address " << &v1 << " is " << v1 << '\n';
//      // std::cout << "the int at address " << &v2 << " is " << v2 << '\n';
//      // std::cout << "the int at address " << &v3 << " is " << v3 << '\n';
//      // std::cout << "the int at address " << &v4 << " is " << v4 << '\n';
//      // std::cout << "the int at address " << &v5 << " is " << v5 << '\n';
//      // std::cout << "the int at address " << &v6 << " is " << v6 << '\n';
//      // std::cout << "****************end******************" << '\n';
//    }
//
//
//
//    std::cout << "End the programm" << std::endl;
//    return 0;
// }

#include <iostream>
#include "interface.h"
// #include "Oversfa.h"



int main ()
{

   std::cout << "Start the programm" << std::endl;
   std::vector<float> v;
   v.push_back(0);
   v.push_back(0);
   v.push_back(0);
   v.push_back(0);
   v.push_back(0);
   v.push_back(0);

   overInit();
   while(true)
   {

     v = overGetData();
     std::cout << "main.cpp      : " << v[0] << " " << v[1] << " " << v[2] << " "
     << v[3] << " " << v[4] << " " << v[5] << std::endl;
     // overClose();
   }



   std::cout << "End the programm" << std::endl;
   return 0;
}
