#include "interface.h"

using namespace std;

// Convert radians to degrees
#define RAD2DEG(rad) ((rad) * 180.0 / 3.1415927)

// TODO Use pointer of reference to this object
// Define as a global value
// Not the best !
bool alreadyOpen = false;
SfAccess sfa;

// Open the "Buffer of the camera"
bool overInit()//Oversfa &oversfa)
{
  if(alreadyOpen == false)
  {
    // TODO change the path of the sfaccess to put the one inside ROS
    // TODO change to a relative path !!!
    // Becareful : Don't Forget to add the file in the path !!!!
    // sfa.setIniPath("../param/sfaccess.ini");
    // sfa.setIniPath("/home/jonathan/Documents/Kyburz/IS-1500_2017d/Camera/Linux/SDKs/sfAccessSDK/x86_64/sfaccess.ini");
    // sfa.setIniPath("/home/jonathan/Documents/IS-1500_Software/Linux/SDKs/sfAccessSDK/x86_64/sfaccess.ini");
    // sfa.setIniPath("/home/ew/Documents/wrapperCameraIS-1500/IS-1500_Software/Linux/SDKs/sfAccessSDK/x86_64/sfaccess.ini");
    sfa.setIniPath("../sfaccess.ini");

    // sfa.setIniPath("/home/jonathan/catkin_ws_kyb/src/camera_is1500/param/sfaccess.ini");
    if(sfa.open());
    {
      // std::cout << "sfAcccess is opened" << std::endl;
      alreadyOpen = true;
    }
  }
  return alreadyOpen; // reput true if it isn't work
}

// Get the data of the IMU which transform and save it inside a vector of flot
// Vector v is organised as roll  pitch    yaw   posx   posy   posz
// OUTPUT std::vector<float> v
// TODO Improvement :
// - Replace the output with an object if we want to add a collected picture
vector<float> overGetData()
{
  SfAccess::TrkData trkData;
  bool trkValid = false;
  sfa.getTrackingDataLatest(&trkValid, &trkData);

  // v is a table of float with the data of the IMU of the camera
  // organised as roll  pitch    yaw   posx   posy   posz
  std::vector<float> v;

  v.push_back(RAD2DEG(trkData.trkRot[0]));
  v.push_back(RAD2DEG(trkData.trkRot[1]));
  v.push_back(RAD2DEG(trkData.trkRot[2]));
  v.push_back(trkData.trkPos[0]);
  v.push_back(trkData.trkPos[1]);
  v.push_back(trkData.trkPos[2]);

  // return the vector containing the imu's data.
  return v;
}

// Close the "Buffer of the camera"
bool overClose()
{
  // std::cout << "Try to close sfAcccess" << std::endl;
  sfa.close();
  // std::cout << "sfAcccess is closed" << std::endl;
  return true;
}
