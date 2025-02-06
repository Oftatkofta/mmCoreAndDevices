#pragma once
#include "MMDevice.h"
#include "DeviceBase.h"
#include "DeviceThreads.h"
#include "ModuleInterface.h"
#include <string>

// Error codes should be in range 10000-10999 for device adapters
#define ERR_PORT_CHANGE_FORBIDDEN    10004
#define ERR_UNRECOGNIZED_ANSWER      10009
#define ERR_UNSPECIFIED_ERROR        10010
#define ERR_HOME_REQUIRED            10011
#define ERR_INVALID_PACKET_LENGTH    10012
#define ERR_RESPONSE_TIMEOUT         10013
#define ERR_BUSY                     10014
#define ERR_STEPS_OUT_OF_RANGE       10015
#define ERR_STAGE_NOT_ZEROED         10016

// Command lengths - make const for type safety
static const int SET_POS_LENGTH = 12;
static const int QUERY_POS_LENGTH = 6;
static const int STATUS_LENGTH = 6;

// Device specific constants - for external use
static const char* const g_DeviceName = "MCM3000";
static const char* const g_Description = "MCM3000 Focus Controller";

// Add standard error messages
static const char* const g_Msg_PORT_CHANGE_FORBIDDEN = "Port change is not allowed after device has been initialized.";
static const char* const g_Msg_INVALID_STEP_SIZE = "Invalid step size";
static const char* const g_Msg_DEVICE_BUSY = "Device is busy";
static const char* const g_Msg_STEPS_OUT_OF_RANGE = "Position out of range";
static const char* const g_Msg_NOT_INITIALIZED = "Device not initialized";

class myFocusController : public CStageBase<myFocusController>
{
public:
    myFocusController();
    ~myFocusController();

    // MMDevice API
    int Initialize();
    int Shutdown();
    void GetName(char* name) const;
    bool Busy();

    // Stage API
    int SetPositionUm(double pos);
    int GetPositionUm(double& pos);
    int SetPositionSteps(long steps);
    int GetPositionSteps(long& steps);
    int SetRelativePositionUm(double d);
    int SetRelativePositionSteps(long steps);
    int GetLimits(double& lower, double& upper);
    int Home();
    int Stop();
    int SetOrigin();
    int Move(double velocity);
    int SetAdapterOriginUm(double d);

    // Focus-specific functions
    int GetFocusDirection(MM::FocusDirection& direction);
    bool IsContinuousFocusDrive() const;
    int IsStageSequenceable(bool& isSequenceable) const;
    int IsStageLinearSequenceable(bool& isSequenceable) const;

    // Action interface
    int OnPort(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnStepSizeUm(MM::PropertyBase* pProp, MM::ActionType eAct);

    // Declare SendCommand in header
    int SendCommand(const unsigned char* command, unsigned length);

    // Device specific constants - for internal use
    static const char* DeviceName() { return g_DeviceName; }
    static const char* Description() { return g_Description; }

private:
    int GetResponse(unsigned char* response, unsigned length);
    int ClearPort();
    int MoveBlocking(long steps, bool relative);

    // Timing constants
    static const int MOTOR_STATUS_POLL_MS = 50;  // Poll interval for motor status
    static const int ENCODER_COUNT_TOLERANCE = 1; // Minimum 1 count tolerance to avoid hanging
    static const long INVALID_POSITION = 0x80000000;  // Invalid position marker
    
    // Command codes
    static const unsigned char AXIS_ID_BYTE = 0x01;     // For Stop, Query Position, Query Status
    static const uint16_t AXIS_ID_WORD = 0x0001;       // For Set encoder, Go to Position
    static const unsigned char CMD_STOP = 0x01;         // 1 byte ID
    static const unsigned char CMD_QUERY_POS = 0x0A;    // 1 byte ID
    static const unsigned char CMD_QUERY_STATUS = 0x80; // 1 byte ID
    static const unsigned char CMD_SET_ENCODER = 0x09;  // 2 byte ID
    static const unsigned char CMD_GOTO_POS = 0x53;     // 2 byte ID
    
    bool initialized_;
    std::string port_;
    double stepSizeUm_;
    double answerTimeoutMs_;
    bool home_;
    long curSteps_;          // Cached position
    bool positionValid_;     // Cache validity flag
    MM::MMTime lastMoveTime_;
    unsigned char lastCommand_;  // Tracks last command sent for response validation
    // Device limits
    static const double POSITION_LIMIT_UM;  // ±12.7mm for ZFM2020/2030
    static const double DEFAULT_STEP_SIZE_UM;  // From Python code
};
