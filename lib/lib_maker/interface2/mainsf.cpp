// Sensor Fusion Development Kit Sample Application
//
// Date: 2016 Mar 29
// Compatibility: sfAccess 1.00.00
//
// Thales Visionix, Inc.
// 700 Technology Park Drive, Suite 102
// Billerica, MA 01821 USA
// www.intersense.com
// techsupport@intersense.com
// +1 781-541-7624
//
// This is an example of how to use the Sensor Fusion Access Library API
// including retrieval of data output from a single tracker.
//
// This sample is released as public domain. It is distributed in the hope
// that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

#ifdef WIN32
#include "windows.h"
#endif
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <unistd.h>
#include "libsfaccess.h"

using namespace std;

// Return status;
#define ERR 0
#define OK 1

// Sleep specified number of ms
#ifdef WIN32
#define SNOOZE_MS(ms) Sleep(ms)
#else // Linux
#define SNOOZE_MS(ms) usleep((ms) * 1000)
#endif

// Convert radians to degrees
#define RAD2DEG(rad) ((rad) * 180.0 / 3.1415927)

// Stop program?
static bool stop = false;

// Signal handler for ctrl-c
void sigHandler(int sig)
{
    cout << endl << "Stop signal received" << endl;
    stop = true;
}

// add function
string testFuncion(bool alpha)
{
  if (alpha == true)
  {
    return "Salut les enfants !";
  }else
  {
    return "On n'est pas content";
  }
}

string giveOutput(bool alpha)
{
  string output = "I am empty";

//
// Start programm
//


SfAccess sfa;
SfAccessStatus status;
SfAccess::TrkData trkData;
bool trkValid = false;
SfAccess::ErrorCode error;
uint64_t timeInitial = 0;
double timeNow;
uint32_t sid = 0;
bool tcpOk = true;
uint64_t watchdog;

// Set up signal handler for ctrl-c
signal(SIGINT, &sigHandler);

cout << "Sensor Fusion Development Kit Sample Application" << endl;

// Show library version
cout << "sfAccess version " << sfa.getVersion() << endl;

// Read sfaccess.ini file and open tracker interface
if (!sfa.open())
{
    cout << "Error: Failed to open - " << sfa.getLastError() << endl;
    return ERR;
}

// Check connection status
if (!sfa.getStatus(&status))
{
    cout << "Error: Failed to get status - " << sfa.getLastError() << endl;
}
// If TCP interface is not open, streaming cannot be started here and
// must be enabled in sfRxCore.ini in order to receive data
else if (!(status.portStatus & SFACCESS_STATUS_TCP_SFRX))
{
    cout << "Warning: TCP interface not open" << endl;
    tcpOk = false;
}

// Start streaming if TCP interface is open
if (tcpOk && !sfa.setStreaming(true))
{
    cout << "Error: Failed to start streaming - " << sfa.getLastError() << endl;
}

// Wait for streaming to start.
// If streaming does not start, check communication status:
// NA - Status not available (sfRxCore not streaming or UDP port incorrect).
// SEARCHING - No sensor module detected (check hardware and sfRxCore I/O settings).
// PAUSED - Streaming is not enabled (TCP needed or sfRxCore must stream by default).
// DISCONNECTED - Connection to sensor module lost (check hardware).
cout << "Waiting for streaming to start (ctrl-c to stop)..." << endl;
while(!stop)
{
    SNOOZE_MS(1000);

    if (!sfa.getStatus(&status))
    {
        cout << "Error: Failed to get status - " << sfa.getLastError() << endl;
        break;
    }

    if (status.commStatus == SFACCESS_SFRX_COMM_STREAMING)
    {
        cout << "Streaming started" << endl;
        break;
    }
    else if (status.commStatus != SFACCESS_SFRX_COMM_NA)
    {
        cout << "Communication status = " << status.commStatus << endl;
    }
}

// Clear any start-up errors
while (sfa.getErrorCode(&error));

// Start watchdog timer
watchdog = sfa.getTimeUs();

if (!stop)
{
cout << "Reading data (ctrl-c to stop)..." << endl;
cout << "  time state   roll  pitch    yaw   posx   posy   posz" << endl;
}

while (!stop)
{
    // Get latest tracker data
    if (!sfa.getTrackingDataLatest(&trkValid, &trkData))
    {
        cout << endl << "Error: Failed to get tracking data - " << sfa.getLastError() << endl;
        break;
    }
    // If new tracker data is available...
    if (trkValid && trkData.sid != sid)
    {
        // Update sequence ID
        sid = trkData.sid;

        // Get relative timestamp in seconds
        if (timeInitial == 0)
        {
            timeInitial = trkData.timeImu;
        }
        timeNow = (trkData.timeImu - timeInitial) / 1000000.0;

        cout << "\r";

        // Timestamp (s)
        cout << setw(6) << timeNow << " ";

        // Tracker state
        // 0 - Reset
        // 1 - Initialize
        // 2 - Acquiring
        // 3 - Optical lock
        // 4 - Free inertial
        // 5 - Error
        cout << setw(5) << trkData.trkState << " ";

        // Orientation Euler angles (deg): roll, pitch, yaw
        cout << setprecision(1) << fixed;
        cout << setw(6) << RAD2DEG(trkData.trkRot[0]) << " ";
        cout << setw(6) << RAD2DEG(trkData.trkRot[1]) << " ";
        cout << setw(6) << RAD2DEG(trkData.trkRot[2]) << " ";

        // Position (m): X, Y, Z
        cout << setprecision(1) << fixed;
        cout << setw(6) << trkData.trkPos[0] << " ";
        cout << setw(6) << trkData.trkPos[1] << " ";
        cout << setw(6) << trkData.trkPos[2] << " ";
    }

    // Check for errors
    if (sfa.getErrorCode(&error))
    {
        cout << endl << "Error " << error.code << ": " << sfa.getErrorMessage(error.code) << endl;
    }
    // Check communication status
    if (!sfa.getStatus(&status))
    {
        cout << endl << "Error: Failed to get status - " << sfa.getLastError() << endl;
        break;
    }
    // Still streaming?
    if (status.commStatus == SFACCESS_SFRX_COMM_STREAMING)
    {
        // Reset watchdog timer
        watchdog = sfa.getTimeUs();
    }
    // If not streaming and watchdog timer expires...
    else if (sfa.getTimeUs() - watchdog > 2000000)
    {
        if (status.commStatus == SFACCESS_SFRX_COMM_NA)
        {
            cout << endl << "Warning: Streaming stopped - Watchdog timer expired" << endl;
        }
        else
        {
            cout << endl << "Warning: Streaming stopped - Communication status = " <<
                status.commStatus << endl;
        }
        // Reset watchdog timer
        watchdog = sfa.getTimeUs();
    }

    // Free processing resource
    SNOOZE_MS(1);
} // end while
cout << endl;

// Close tracker interface
if (!sfa.close())
{
    cout << "Error: Failed to close - " << sfa.getLastError() << endl;
    return ERR;
}

cout << "Disconnected" << endl;


//
// End of programm
//

  return output;
}

bool init()
{

}

int main(void)
{
  // while(true)
  // {
  //     cout << testFuncion(true) << endl;
  // }
  cout << giveOutput(true) << endl;

    return 0;
}
