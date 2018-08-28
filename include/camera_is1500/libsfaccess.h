///////////////////////////////////////////////////////////////////////////////
//
// libsfaccess.h
//
// Copyright (c) InterSense LLC 2012. All rights reserved.
//
// Comments:
//
// Unless otherwise indicated, functions returning boolean status
// return true on success and false on failure
//
// For Windows:
// Character set is not set.
// DLL runtimes are required because some functions return std::string.
//
// To install on Linux:
// export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH in client app context
// cp libsfaccess.so to where it is used
//
///////////////////////////////////////////////////////////////////////////////

#ifndef LIBSFACCESS_H_
#define LIBSFACCESS_H_

#include "sfosiTypes.h"

#if !defined QT_SFACCESS && defined WIN32 && !defined SFA_STATIC
#ifdef SFACCESS_EXPORTS
#define DllItem __declspec( dllexport )
#else
#define DllItem __declspec( dllimport )
#endif
#else // Linux
#define DllItem
#endif

///////////////////////////////////////////////////////////////////////////////
// C API

#ifdef __cplusplus
extern "C"
{
#endif

typedef int bool_t;
struct HSfAccess;
typedef struct HSfAccess HSfAccess;

DllItem HSfAccess *SfAccess_create(void);
DllItem void SfAccess_destroy(HSfAccess *h);

DllItem bool_t SfAccess_open(HSfAccess *h);
DllItem bool_t SfAccess_close(HSfAccess *h);

DllItem void SfAccess_setIniPath(HSfAccess *h, char *buf);

DllItem const char *SfAccess_getVersion(HSfAccess *h, char *buf, size_t bufsize);
DllItem const char *SfAccess_getBuildDate(HSfAccess *h, char *buf, size_t bufsize);
DllItem const char *SfAccess_getLastError(HSfAccess *h, char *buf, size_t bufsize);

// Error code source prefixes
#define SFACCESS_ERR_SFCORE   0x000
#define SFACCESS_ERR_SFRX     0x100
#define SFACCESS_ERR_SFTX     0x200
#define SFACCESS_ERR_SFACCESS 0x300
#define SFACCESS_ERR_SFHUB    0x400

// SfAccess error codes
// Some of these errors will be reported with sequence ID zero since
// the actual value won't be known at the time of the error.
#define SFACCESS_ERRCODE_IMG_XFER_FAILED     (SFACCESS_ERR_SFACCESS | 1)
#define SFACCESS_ERRCODE_BAD_TRK_PKT         (SFACCESS_ERR_SFACCESS | 2)
#define SFACCESS_ERRCODE_BAD_IMG_PKT         (SFACCESS_ERR_SFACCESS | 3)
#define SFACCESS_ERRCODE_BAD_META_PKT        (SFACCESS_ERR_SFACCESS | 4)
#define SFACCESS_ERRCODE_BAD_OPT_SUBPKT      (SFACCESS_ERR_SFACCESS | 5)
#define SFACCESS_ERRCODE_BAD_DIAG_SUBPKT     (SFACCESS_ERR_SFACCESS | 6)
#define SFACCESS_ERRCODE_BAD_TGT_SUBPKT      (SFACCESS_ERR_SFACCESS | 7)
#define SFACCESS_ERRCODE_BAD_PPOSE_SUBPKT    (SFACCESS_ERR_SFACCESS | 8)
#define SFACCESS_ERRCODE_BAD_IMU_SUBPKT      (SFACCESS_ERR_SFACCESS | 9)
#define SFACCESS_ERRCODE_TGT_SUBPKT_OVERRUN  (SFACCESS_ERR_SFACCESS | 10)
#define SFACCESS_ERRCODE_DIAG_SUBPKT_OVERRUN (SFACCESS_ERR_SFACCESS | 11)
#define SFACCESS_ERRCODE_OPT_SUBPKT_OVERRUN  (SFACCESS_ERR_SFACCESS | 12)
#define SFACCESS_ERRCODE_BAD_HUB_PKT         (SFACCESS_ERR_SFACCESS | 13)
#define SFACCESS_ERRCODE_BAD_NFT_PKT         (SFACCESS_ERR_SFACCESS | 14)
#define SFACCESS_ERRCODE_BAD_GPS_SUBPKT      (SFACCESS_ERR_SFACCESS | 15)
#define SFACCESS_ERRCODE_BAD_PRA_PKT         (SFACCESS_ERR_SFACCESS | 16)

// Error code data
typedef struct s_SfAccessErrorCode
{
    uint16_t code; // error code
    uint32_t sid;  // sequence ID
}
SfAccessErrorCode;

DllItem uint16_t SfAccess_getErrorCode(HSfAccess *h, SfAccessErrorCode *data);
DllItem const char *SfAccess_getErrorMessage(HSfAccess *h, char *buf, size_t bufsize,
    uint16_t code);

DllItem bool_t SfAccess_getTargetCount(HSfAccess *h, bool_t *valid, uint32_t *sid, uint32_t *count);

// Diagnostics data
typedef struct s_SfAccessDiagData
{
    uint32_t sid;         // sequence ID
    float procTemp;       // processor chip temp (deg C)
    uint32_t exposure;    // camera exposure (us)
    uint8_t opticalMode;  // optical mode
    float ncTemp;         // NavChip temp (deg C)
    uint8_t ncRtwFlags;   // NavChip run-time warning flags
    uint8_t errorCode[4]; // sfRx error codes (0 = no error)
    uint32_t commStatus;  // communication status
	uint8_t trkType;      // Tracker type (0=invalid, 1=HObIT, 2=InertiaCam, 3=InertiaCubeNC)
	uint8_t trkQuality;   // Tracking quality percentage (0-100)
    uint8_t trkState;     // Tracking state (0=no comm, 1=tracking not locked, 2=tracking locked, 3=lost)
    uint8_t trkMode;      // Tracking mode (1=sfCore 3DOF, 2=sfCore 6DOF, 3=NFT+fid, 4=NFT+GPS, 5=NFT+GPS+fid)
}
SfAccessDiagData;

DllItem bool_t SfAccess_getDiagData(HSfAccess *h, bool_t *valid, SfAccessDiagData *data);
DllItem bool_t SfAccess_getDiagDataLatest(HSfAccess *h, bool_t *valid, SfAccessDiagData *data);

// Returns size of image buffer (bytes)
DllItem uint32_t SfAccess_getImageBufSize(HSfAccess *h);

// Image formats
enum SfAccessImageFormat
{
    VGA_GRAY_8 = 10,
    VGA_GRAY_16,
    VGA_YUYV_8,
    WVGA_GRAY_8 = 20,
    WVGA_GRAY_16,
    SXGA_GRAY_8 = 30,
    SXGA_GRAY_16,
    UVGA_GRAY_8 = 40,
    UVGA_GRAY_8_UND = 41,
    VGA_24 = 50,
    UVGA_24,
};

// Retrieves last received image
// out: progress - 0=waiting, 100=done, 1-99=receiving
// out: format - image format
// out: sid - sequence id
// out: buf - image buffer
// in:  bufsize - size of image buffer
DllItem bool_t SfAccess_getImage(HSfAccess *h, int32_t *progress, enum SfAccessImageFormat *format, uint32_t *sid,
    uint8_t buf[], size_t bufsize);

// Reports image info given format (outputs ignored if null)
// in:  format - image format
// out: size - image size in bytes
// out: w - image width in pixels
// out: h - image height in pixels
// out: bpp - bytes per pixel
DllItem bool_t SfAccess_getImageInfo(HSfAccess *h, enum SfAccessImageFormat format,
    size_t *size, int32_t *width, int32_t *height, int32_t *bpp);

// Gets/sets register byte
// in: tmo - timeout (ms)
// in: addr - register address
// out/in: value - register value
DllItem bool_t SfAccess_getRegister(HSfAccess *h, uint32_t tmo, uint32_t addr, uint8_t *value);
DllItem bool_t SfAccess_setRegister(HSfAccess *h, uint32_t tmo, uint32_t addr, uint8_t value);

// Abridged tracking data
typedef struct s_SfAccessTrkData
{
    uint32_t sid;            // sequence ID
    uint32_t timePlat;       // platform data timestamp (ms)
    float ahrsRot[3];        // AHRS roll/pitch/yaw (rad)
    uint16_t state;          // tracker state
    uint16_t status;         // tracker status flags
    float rot[3];            // tracker roll/pitch/yaw (rad)
    float pos[3];            // tracker position (m)
    float vel[3];            // tracker velocity (m/s)
    float acc[3];            // tracker acceleration (m/s/s)
    float omega[3];          // tracker angular rate (rad/s)
    float rotSig[3];         // tracker roll/pitch/yaw sigma (rad)
    float posSig[3];         // tracker position sigma (m)
	float velSig[3];    // X/Y/Z velocity sigma (m/s)
    float autoHarmRot[3];    // auto-harmonization angles (rad)
}
SfAccessTrkData;

DllItem bool_t SfAccess_getTrackingDataLatest(HSfAccess *h, bool_t *valid,
    SfAccessTrkData *data);

DllItem bool_t SfAccess_getTrackingDataAtTime(HSfAccess *h, uint64_t timeUs,
    bool_t *valid, SfAccessTrkData *data);

// Exports of functions used for parsing raw data
DllItem void SfAccess_processMetaPkt(HSfAccess *h, const uint8_t *buf);
DllItem void SfAccess_processTrackingPkt(HSfAccess *h, const uint8_t *buf);

// Tracking data
// Vectors are X/Y/Z unless otherwise indicated
typedef struct s_SfAccessTrkDataExt
{
    uint32_t sid;            // sequence ID
    uint64_t timeImu;        // IMU data arrival timestamp (us)
    uint32_t timePlat;       // platform data timestamp (ms)
    uint64_t timeMeta;       // sfCore meta packet data arrival timestamp (us)
    uint32_t procTime;       // sfCore processing time (us)
    float ahrsRot[3];        // AHRS roll/pitch/yaw (rad)
    float ahrsGyrBias[3];    // AHRS gyro bias (rad/s)
    float ahrsAccBias[3];    // AHRS accel bias (m/s/s)
    float ahrsRotSig[3];     // AHRS roll/pitch/yaw sigma (rad)
    float ahrsGyrBiasSig[3]; // AHRS gyro bias sigma (rad/s)
    float ahrsAccBiasSig[3]; // AHRS accel bias sigma (m/s/s)
    uint16_t trkState;       // tracker state
    uint16_t trkStatus;      // tracker status flags
    float trkCbn[3][3];      // n-frame tracker rotation matrix, row-major order
    float trkRot[3];         // n-frame tracker roll/pitch/yaw (rad)
    float trkCbi[3][3];      // i-frame tracker rotation matrix, row-major order
    float trkPos[3];         // tracker position (m)
    float trkVel[3];         // tracker velocity (m/s)
    float trkAcc[3];         // tracker acceleration (m/s/s)
    float trkOmega[3];       // tracker angular rate (rad/s)
    float trkGyrBias0[3];    // tracker primary gyro bias (rad/s)
    float trkGyrBias1[3];    // tracker secondary gyro bias (rad/s)
    float trkAccBias0[3];    // tracker primary accel bias (m/s/s)
    float trkAccBias1[3];    // tracker primary accel bias (m/s/s)
    float trkRotSig[3];      // tracker roll/pitch/yaw sigma (rad)
    float trkPosSig[3];      // tracker position sigma (m)
    float trkVelSig[3];      // tracker velocity sigma (m/s)
    float trkGyrBiasSig0[3]; // tracker primary gyro bias sigma (rad/s)
    float trkGyrBiasSig1[3]; // tracker secondary gyro bias sigma (rad/s)
    float trkAccBiasSig0[3]; // tracker primary accel bias sigma (m/s/s)
    float trkAccBiasSig1[3]; // tracker secondary accel bias sigma (m/s/s)
    float autoHarmRot[3];    // auto-harmonization angles (rad)
    float autoHarmRotSig[3]; // auto-harmonization angle sigmas (rad)
    float insDelay;          // platform INS delay estimation (s)
    float insDelaySig;       // platform INS delay estimation sigma (s)
    uint8_t errorCode[4];    // sfCore error codes (0 = no error)
    float platRot[3];        // platform roll/pitch/yaw (rad)
    float platOmega[3];      // platform angular rate (rad/s)
    float globalPose[3];     // tracker lat (deg), long (deg), el (m)
}
SfAccessTrkDataExt;

DllItem bool_t SfAccess_getTrackingDataLatestExt(HSfAccess *h, bool_t *valid,
    SfAccessTrkDataExt *data);

DllItem bool_t SfAccess_getTrackingDataExt(HSfAccess *h, bool_t *valid,
    SfAccessTrkDataExt *data);

DllItem bool_t SfAccess_getTrackingDataAtTimeExt(HSfAccess *h, uint64_t timeUs,
    bool_t *valid, SfAccessTrkDataExt *data);

// sfHub data
typedef struct s_SfAccessHubData
{
	uint32_t sid;       // sequence ID
	uint16_t format;    // reserved
	uint8_t init;       // flag indicating NavChip calibration is done
	uint32_t coreId;    // reserved
	uint32_t seqId;     // reserved
	uint64_t osTime;    // timestamp (us)
	float outCbn[3][3]; // rotation matrix, n-frame, row-major order
	float rot[3];       // orientation euler angles, roll/pitch/yaw, n-frame (rad)
	float pos[3];       // X/Y/Z position, n-frame (m)
	float vel[3];       // X/Y/Z velocity, n-frame (m/s)
	float acc[3];       // X/Y/Z acceleration, n-frame (m/s/s)
	float omega[3];     // X/Y/Z angular rate, body frame (rad/s)
	float rotSig[3];    // roll/pitch/yaw orientation sigma (rad)
	float posSig[3];    // X/Y/Z position sigma (m)
	float velSig[3];    // X/Y/Z velocity sigma (m/s)
	uint8_t footLift;   // event flag indicating when foot leaves floor
}
SfAccessHubData;

// GPS data
typedef struct s_SfAccessGpsData
{
	struct GGA
	{
		uint8_t format;       // GPS sentence format identifier.
		uint32_t timestamp;   // GGA timestamp (HH:MM:SS from GPS converted to sec)
		double latitude;      // GGA latitude
		double longitude;     // GGA longitude
		float altitude;       // Meters
		uint16_t nSatellites; // Satellite count
		uint8_t quality;      // GPS fix quality. 1 = GPS fix
		float geoIdHeight;    // Height of sea level (meters)
		float hdop;           // Horizontal dilution of precision.
	} gga;

	struct RMC
	{
		uint32_t timestamp;   // Minimum Recommended timestamp (HH:MM:SS from GPS converted to sec)
		double latitude;      // Minimum Recommended Latitude
		double longitude;     // Minimum Recommended Longitude
		float speed;          // m/s
		float bearing;        // degrees
		float magVar;		  // Variation from magnetic north (degrees)
		bool_t dataActive;    // Is GPS data good?
	} rmc;
}
SfAccessGpsData;

DllItem bool_t SfAccess_getGpsData(HSfAccess *h, bool_t *valid, SfAccessGpsData *data);
DllItem bool_t SfAccess_setGpsData(HSfAccess *h, SfAccessGpsData *data);

// Platform pose data
typedef struct s_SfAccessPlatPoseData
{
    uint32_t timePlat; // platform data timestamp (ms)
    float rot[3];      // roll/pitch/yaw (rad)
}
SfAccessPlatPoseData;

DllItem bool_t SfAccess_setPlatformPoseData(HSfAccess *h, SfAccessPlatPoseData data);

// Pose recovery statistics
typedef struct s_SfAccessPoseRecStats
{
    uint32_t sid;           // sequence ID
    int32_t nRecognized;    // number of targets recognized
    int32_t nMatched;       // number of targets matched to constellation
    int32_t nUsed;          // number of targets used in pose recovery
    int32_t delay;          // cycles of delay in receiving optical data
    float praMaxFitErr;     // pose recovery max fit error
    bool_t praValid;        // pose recovery valid
    bool_t praMeasAccepted; // pose recovery measurement accepted
    float poseInnov;        // pose measurement normalized innovation squared
    float praCbp[3][3];     // pose recovery rotation matrix, row-major order
    float praPos[3];        // pose recovery position, rNav (m)
    float praCrossAxisSpan; // pose recovery cross-axis span (rad)
    float praGdop;          // pose recovery geometric dilution of precision
    float praPdop;          // pose recovery position dilution of precision
    float praOdop;          // pose recovery orientaiton dilution of precision
	float rotCov[3];        // pose recovery rotation covariances (not valid for sfCore)
	float posCov[3];        // pose recovery position covariances (not valid for sfCore)
	uint16_t pointIDs[60];  // pose recovery point IDs (not valid for sfCore)
}
SfAccessPoseRecStats;

DllItem bool_t SfAccess_getPoseRecoveryStats(HSfAccess *h, bool_t *valid,
    SfAccessPoseRecStats *data);

DllItem bool_t SfAccess_setBoresightRef(HSfAccess *h, float euler[]);
DllItem bool_t SfAccess_setPrediction(HSfAccess *h, float interval);
DllItem bool_t SfAccess_setTipOffset(HSfAccess *h, float offset[]);

DllItem bool_t SfAccess_transactImu(HSfAccess *h, int sel, uint8_t txbuf[],
    size_t txsize, uint8_t rxbuf[], size_t rxsize, uint32_t rxtmo, size_t *rxcount);

DllItem uint64_t SfAccess_getTimeUs(void);

DllItem bool_t SfAccess_sendInputData(HSfAccess *h, uint8_t data);

// Port status flags
#define SFACCESS_STATUS_TCP_SFRX       0x001
#define SFACCESS_STATUS_UDP_SFRX       0x002
#define SFACCESS_STATUS_UDP_SFRX_IMG   0x004
#define SFACCESS_STATUS_UDP_SFRX_INPUT 0x008
#define SFACCESS_STATUS_UDP_SFCORE     0x010
#define SFACCESS_STATUS_UDP_SFHUB      0x020
#define SFACCESS_STATUS_SMEM_IMG       0x040
#define SFACCESS_STATUS_UDP_NFT        0x080
#define SFACCESS_STATUS_UDP_PRA        0x100

// sfRx communication status
#define SFACCESS_SFRX_COMM_NA           0
#define SFACCESS_SFRX_COMM_SEARCHING    1
#define SFACCESS_SFRX_COMM_STREAMING    2
#define SFACCESS_SFRX_COMM_PAUSED       3
#define SFACCESS_SFRX_COMM_DISCONNECTED 4

// receive status flags
#define SFACCESS_RECEIVE_UDP_SFRX     0x02
#define SFACCESS_RECEIVE_UDP_SFRX_IMG 0x04
#define SFACCESS_RECEIVE_UDP_SFCORE   0x10
#define SFACCESS_RECEIVE_UDP_SFHUB    0x20
#define SFACCESS_RECEIVE_SMEM_IMG     0x40
#define SFACCESS_RECEIVE_UDP_NFT      0x80

// Status data
typedef struct s_SfAccessStatus
{
    uint32_t portStatus;     // SFACCESS_STATUS
    uint32_t receiveStatus;  // SFACCESS_RECEIVE
    uint32_t errorStatus;    // error status (0=no new errors, 1=new errors)
    uint32_t commStatus;     // SFACCESS_SFRX_COMM
}
SfAccessStatus;

DllItem bool_t SfAccess_getStatus(HSfAccess *h, SfAccessStatus *data);

// Aux input data from NavChip (I2C)
typedef struct s_SfAccessAuxData
{
    uint8_t data[4];
}
SfAccessAuxData;

DllItem bool_t SfAccess_getAuxData(HSfAccess *h, bool_t *valid, SfAccessAuxData *auxData);
DllItem bool_t SfAccess_getAuxDataLatest(HSfAccess *h, bool_t *valid, SfAccessAuxData *auxData);

// IMU data (vectors are in X-Y-Z order)
typedef struct s_SfAccessImuData
{
    uint64_t time;      // data arrival timestamp (us)
    uint32_t sid;       // sequence ID
    uint32_t pid;       // packet ID
    int32_t flags;      // discrete flags
    float dt;           // delta time (s)
    int16_t dv[3];      // delta V (m/s)
    int16_t dtheta[3];  // delta theta (rad)
    float a[3];         // acceleration (m/s/s)
    float w[3];         // angular rate (rad/s)
}
SfAccessImuData;

// IMU selector
typedef enum e_SfAccessImuSelect
{
    PLATFORM = 0,
    CAMERA = 1,
}
SfAccessImuSelect;

typedef enum e_SfAccessFileLoc
{
	SFRX = 0,
	SFTX = 1,
}
SfAccessFileLoc;

DllItem bool_t SfAccess_getImuData(HSfAccess *h, SfAccessImuSelect imu, bool_t *valid, SfAccessImuData *imuData);

// Retrieve latest IMU data from primary (Camera) IMU
DllItem bool_t SfAccess_getImuDataLatest(HSfAccess *h, bool_t *valid, SfAccessImuData *imuData);

DllItem bool_t SfAccess_getFile(HSfAccess *h, const char* src, SfAccessFileLoc srcsel, const char* dest);


// Sets streaming state
// in : on - streaming on?
DllItem bool_t SfAccess_setStreaming(HSfAccess *h, bool_t on);


// Gets streaming state
// returns : true if streaming
DllItem bool_t SfAccess_getStreaming(HSfAccess *h);


#ifdef __cplusplus
}

///////////////////////////////////////////////////////////////////////////////
// C++ API

#include <string>
#include <queue>

// Forward definitions
class SfosiTcpClient;
class SfosiUdpClient;
class SfosiUdpServer;
class SfosiSharedMem;

class DllItem SfAccess
{
public:

    SfAccess();
    ~SfAccess();

    // Connects to VisTracker2 (ini file is read at this time)
    bool open();

    // Disconnects from VisTracker2 and releases resources
    bool close();

    // Control message verbosity
    // in: control - 0=off
    void setVerbosity(uint32_t control);

    // Control log file output
    // in: control - 0=off
    // in: path - log file path (must end with /)
    void setLogging(uint32_t control, std::string path);

    // in: path - ini file name (with full path)
    void setIniPath(std::string path);

    // Returns sfAccess version (x.yy.zz where x.yy = ICD rev)
    std::string getVersion();

    // Returns sfAccess build date (mmm dd yyyy)
    std::string getBuildDate();

    // Returns error message after a function fails
    std::string getLastError();

    // Clears error message
    void clearLastError();

    // Returns next error code (zero if none)
    // out: data - error code data
    typedef SfAccessErrorCode ErrorCode;
    uint16_t getErrorCode(ErrorCode *data);

    // Returns message translated from error code obtained from getErrorCode()
    // in: code - error code
    std::string getErrorMessage(uint16_t code);

    // Returns size of image buffer (bytes)
    uint32_t getImageBufSize();

    // Image formats
    enum ImageFormat
    {
        VGA_GRAY_8 = 10,
        VGA_GRAY_16,
        VGA_YUYV_8,
        WVGA_GRAY_8 = 20,
        WVGA_GRAY_16,
        SXGA_GRAY_8 = 30,
        SXGA_GRAY_16,
        UVGA_GRAY_8 = 40,
        UVGA_GRAY_8_UND = 41,
        VGA_24 = 50,
        UVGA_24,

    };

    // Retrieves last received image
    // out: progress - 0=waiting, 100=done, 1-99=receiving
    // out: format - image format
    // out: sid - sequence id
    // out: buf - image buffer
    // in:  bufsize - size of image buffer
    bool getImage(int32_t *progress, enum ImageFormat *format, uint32_t *sid,
        uint8_t buf[], size_t bufsize);

    // Reports image info given format (outputs ignored if null)
    // in:  format - image format
    // out: size - image size in bytes
    // out: w - image width in pixels
    // out: h - image height in pixels
    // out: bpp - bytes per pixel
    bool getImageInfo(enum ImageFormat format,
        size_t *size, int32_t *w, int32_t *h, int32_t *bpp);

    // Reads next available IMU data packet
    // in:  sel - source selector (0=platform, 1=camera)
    // out: valid - true if data is valid
    // out: data - IMU data
    typedef SfAccessImuSelect ImuSelect;
    typedef SfAccessImuData ImuData;
    bool getImuData(ImuSelect sel, bool *valid, ImuData *data);
    
    // Retrieve latest data from primary IMU (Camera)
    bool getImuDataLatest(bool *valid, ImuData *data);

    // IMU configuration register set (see NavChip ICD for details)
    typedef struct s_ImuCrs
    {
        uint8_t crs[32];
    }
    ImuCrs;

    // Retrieves last received IMU configuration register set
    // in:  sel - source selector (0=platform, 1=camera)
    // out: valid - true if data is valid
    // out: data - IMU CRS
    bool getImuCrs(ImuSelect sel, bool *valid, ImuCrs *data);

    // Target data
    // Coordinate pairs are (U,V)
    typedef struct s_TgtData
    {
        uint32_t sid;     // sequence ID
        int32_t i;        // target index (0..count)
        int32_t type;     // target type
        uint16_t tid;     // target ID
        int32_t flags;    // output options (0=distorted, 1=undistorted)
        uint16_t roi[2];  // coord. of top-left corner
        uint16_t size[2]; // size (pixels)
        float eyec[2];    // coord. of center eye
        float eyeb1[2];   // coord. of black eye 1
        float eyeb2[2];   // coord. of black eye 2
        int32_t ourate;   // optical update rate (Hz)
    }
    TgtData;

    // Reads next available optical target
    // out: valid - true if data is valid
    // out: data - target data
    bool getTargetData(bool *valid, TgtData *data);

	// Get number of optical targets
    // out: valid - true if data is valid
	// out: sid - sequence ID
	// out: count - number of targets
	bool getTargetCount(bool *valid, uint32_t *sid, uint32_t *count);

    // Reads next available diagnostic data set
    // out: valid - true if data is valid
    // out: data - diag data
    typedef SfAccessDiagData DiagData;
    bool getDiagData(bool *valid, DiagData *data);
    bool getDiagDataLatest(bool *valid, DiagData *data);

    // Reads next or latest available tracking data
    // out: valid - true if data is valid
    // out: data - tracking data
    typedef SfAccessTrkDataExt TrkData;
    bool getTrackingData(bool *valid, TrkData *data);
    bool getTrackingDataLatest(bool *valid, TrkData *data);

    // Reads latest available tracking data and applies prediction
    // in: time - time to which to predict, obtained using getTimeUs()
    // out: valid - true if data is valid
    // out: data - tracking data
    bool getTrackingDataAtTime(uint64_t timeUs, bool *valid, TrkData *data);

    // Target residual data
    typedef struct s_TgtResData
    {
        uint32_t sid;   // sequence ID
        int32_t i;      // target index (0..count)
        int32_t tid;    // target ID
        float meas[2];  // coord. of measurement
        float resid[2]; // residual vector
        float innov;    // normalized innovation squared
    }
    TgtResData;

    // Reads next available target residual data
    // out: valid - true if data is valid
    // out: data - target residual data
    bool getTargetResData(bool *valid, TgtResData *data);

    // Reads next available set of pose recovery statistics
    // out: valid - true if data is valid
    // out: data - pose recovery statistics
    typedef SfAccessPoseRecStats PoseRecStats;
    bool getPoseRecoveryStats(bool *valid, PoseRecStats *data);

    // Platform pose data
    typedef SfAccessPlatPoseData PlatPoseData;

    // Sets platform pose data obtained from external GPS/INS system
    // in: data - platform pose data
    bool setPlatformPoseData(PlatPoseData data);

    // Reads next available platform pose data
    // out: valid - true if data is valid
    // out: data - platform pose data
    bool getPlatformPoseData(bool *valid, PlatPoseData *data);

    // Feature data
    typedef struct s_FeatData
    {
        uint32_t sid;   // sequence ID
        int32_t i;      // target index (0..count)
        int32_t tid;    // target ID
        float pos[3];   // X/Y/Z coordinates in world frame (m)
    }
    FeatData;

    // Reads next available set of feature data
    // out: valid - true if data is valid
    // out: data - feature data
    bool getFeatureData(bool *valid, FeatData *data);

    // Sets prediction interval (seconds)
    bool setPrediction(float interval);

    // Sets boresight reference Euler angles (roll/pitch/yaw in radians)
    bool setBoresightRef(float euler[]);

    // Sets tip offset (X/Y/Z in meters)
    bool setTipOffset(float offset[]);

    // Sends input data to sfRx
    bool sendInputData(uint8_t data);

    // File location selector
	typedef SfAccessFileLoc FileLoc;

    // Transfers file to system
    // in: src - file name (may include path) of source file on host
    // in: dest - name of destination file on system
    // in: destsel - destination selector
    bool putFile(std::string src, std::string dest, FileLoc destsel);

    // Transfers file from system
    // in: src - name of source file on system
    // in: srcsel - source selector
    // in: dest - file name (may include path) of destination file on host
    bool getFile(std::string src, FileLoc srcsel, std::string dest);

    // Sends data to IMU and receives response, if any
    // in: sel - destination IMU
    // in: txbuf - transmit buffer
    // in: txsize - number of bytes to transmit
    // out: rxbuf - receive buffer
    // in: rxsize - number of expected receive bytes (0=none)
    // in: rxtmo - receive timeout (ms)
    // out: rxcount - number of received bytes
    bool transactImu(ImuSelect sel, uint8_t txbuf[], size_t txsize,
        uint8_t rxbuf[], size_t rxsize, uint32_t rxtmo, size_t *rxcount);

    // Gets/sets register byte
    // in: tmo - timeout (ms)
    // in: addr - register address
    // out/in: value - register value
    bool getRegister(uint32_t tmo, uint32_t addr, uint8_t *value);
    bool setRegister(uint32_t tmo, uint32_t addr, uint8_t value);

    // Sets streaming state
    // in : on - streaming on?
    bool setStreaming(bool on);

    // Gets streaming state
    // returns : true if streaming
    bool getStreaming();

    // Gets current status
    // out : data
    bool getStatus(SfAccessStatus *data);

    // Gets system time in microseconds
    static uint64_t getTimeUs();

    // Reads next available sfHub data
    // out: valid - true if data is valid
    // out: data - sfHub data
	typedef SfAccessHubData HubData;
    bool getHubData(bool *valid, HubData *data);

    // Reads next available GPS data
    // out: valid - true if data is valid
    // out: data - GPS data
    typedef SfAccessGpsData GpsData;
    bool getGpsData(bool *valid, GpsData *data);

    // Set data obtained from external GPS system
    // out: data - GPS data
    bool setGpsData(GpsData *data);

    // Get latest aux data
    // out: valid - true if data is valid
    // out: auxData - latest aux data
    typedef SfAccessAuxData AuxData;
    bool getAuxData(bool *valid, AuxData *auxData);
    bool getAuxDataLatest(bool *valid, AuxData *auxData);

    // For off-line data processing by sfStudio
    void processMetaPkt(const uint8_t *buf);
    void processTrackingPkt(const uint8_t *buf);
    void processNftDataPkt(const uint8_t *buf);
	void processPraPkt(const uint8_t *buf);
    void init();
    bool offLine;

private:

// Disable VS2010 warning for std:string in private part of class
#ifdef WIN32
#pragma warning (disable : 4251)
#endif

    uint32_t verbosity;
    uint32_t logging;
    std::string logPath;
    std::string iniPath;
    uint8_t *udpBuf;

    bool opened;
    std::string errorMsg;

    bool tcpSfRxOk;
    SfosiTcpClient *pTcpSfRx;

    bool udpSfRxOk;
    SfosiUdpClient *pUdpSfRx;

    bool udpSfRxImgOk;
    SfosiUdpClient *pUdpSfRxImg;

    bool udpSfRxInputOk;
    SfosiUdpServer *pUdpSfRxInput;

    bool udpSfCoreOk;
    SfosiUdpClient *pUdpSfCore;

    bool udpSfHubOk;
    SfosiUdpClient *pUdpSfHub;

	bool udpNftOk;
	SfosiUdpClient *pUdpNft;

    uint32_t imgBufSize;
    int imgBufWriteIdx;
    int imgBufReadIdx; // -1 if no image
    bool imgWait;
    int imgAddr;
    enum ImageFormat imgFormat;
    uint32_t imgSid[2];
    uint8_t *imgBuf[2];
    int imgProgress;
    SfosiSharedMem *pImgSharedMem;
    bool imgSharedMemOk;

    // IMU CRS buffers
    typedef struct s_ImuCrsBuf
    {
        ImuCrs data[2];
        int writeIdx;
        int readIdx; // -1 if no data
        int pid;
    }
    ImuCrsBuf;
    ImuCrsBuf imuCrsCam;
    ImuCrsBuf imuCrsPlat;
    void imuCrsBufReset(ImuCrsBuf *b);

    void processImuSubPkt(const char *name, const uint8_t *p, int size, int32_t sid,
        float dt, uint64_t time, std::queue<ImuData> *q, uint32_t qMaxSize, ImuCrsBuf *b);
    void processTargetSubPkt(const uint8_t *buf);
    void processDiagSubPkt(const uint8_t *buf);
    void processImagePkt(const uint8_t *buf);
    void processOpticalSubPkt(const uint8_t *buf);
    void processPlatPoseSubPkt(const uint8_t *buf);
    void processHubDataPkt(const uint8_t *buf);
    void processHubErrPkt(const uint8_t *buf);
    void processGpsSubPkt(const uint8_t *buf);
    bool sendPlatformPosePkt(PlatPoseData *data);
	bool sendGpsDataPkt(GpsData *data);
    bool sendInputDataPkt(uint8_t data);

    bool streamingOn; // streaming state

    bool serviceUdp(bool enableStreaming = true);

    void flushUdp();
    void flushTcp();

    bool writeSfRx(uint8_t *cmd, size_t len);
    bool readSfRx(uint8_t *reply, size_t len, int tmo, size_t *count);

    void initVsData();

    void addErrorCode(uint16_t code, uint32_t sid);

    // Data queues
    std::queue<ImuData> *pqImuPlat; // platform IMU
    std::queue<ImuData> *pqImuCam; // camera IMU
    std::queue<TgtData> *pqTarget; // target
    std::queue<DiagData> *pqDiag; // diagnostics
    std::queue<TrkData> *pqTracking; // tracking
    std::queue<TgtResData> *pqTargetRes; // target residuals
    std::queue<PoseRecStats> *pqPoseRecStats; // pose recovery statistics
    std::queue<PlatPoseData> *pqPlatPose; // platform pose
    std::queue<HubData> *pqHub; // hub
    std::queue<GpsData> *pqGps; // GPS data
    std::queue<ErrorCode> *pqErrorCode; // error code
    std::queue<AuxData> *pqAuxData; // aux input data queue
    std::queue<FeatData> *pqFeature; // feature

    // Maximum queue sizes (elements)
    uint32_t imuQueueSize; // for all IMU data, platform pose, diag data queues
    uint32_t targetQueueSize; // for both target and residuals queues
    uint32_t trackingQueueSize;
    uint32_t hubQueueSize;
    uint32_t gpsQueueSize;
    uint32_t errorCodeQueueSize;

    // Latest tracking data
    bool trkLatestValid;
    TrkData trkLatest;

    // Latest imu data
    bool imuLatestValid;
    ImuData imuLatest;

    // Latest AUX data
    bool auxLatestValid;
    AuxData auxLatest;

    // Latest Diagnostic Data
    bool diagLatestValid;
    DiagData diagLatest;

    // Parameters used for prediction
    long virtualSyncCtrl;
    float deltaTimeS;        // IMU time step (s)
    uint64_t tBiasImuNs;     // IMU clock bias (ns)
    uint64_t tBiasDispNs;    // display clock bias (ns)
    uint64_t tLocDispPrevUs; // previous display time (us)
    uint64_t tNatDispUs;     // native display time (us) 
    uint64_t dispPeriodNs;   // display period (ns)
    double predictionS;      // prediction interval (s)

    // Other exports
    double boresightRef[3][3]; // Boresight reference matrix
    double tipOffset[3];       // Tip offset (m)

    void updateExports(double pred, TrkData *data);

    bool tcpSfRxEnabled;
    std::string tcpSfRxAddr;
    std::string tcpSfRxPort;
    std::string udpSfRxPort;
    std::string udpSfRxImgPort;
    std::string udpSfRxInputPort;
    std::string udpSfCorePort;
    std::string udpSfHubPort;
	std::string udpNftPort;
    std::string sMemImgName;
    int udpSfRxRcvBufSize;
    int udpSfRxImgRcvBufSize;
    int udpSfCoreRcvBufSize;
    int udpSfHubRcvBufSize;
	int udpNftRcvBufSize;

    // Status
    uint32_t receiveStatus;
    uint32_t errorStatus;
    uint32_t commStatus;

};

#endif

#endif // LIBSFACCESS_H_
