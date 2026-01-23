// FILE:          MCM3000Controller.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Device adapter for the Thorlabs MCM3000/3001 Focus Controller.
//                Implements direct serial control of the MCM3000 Focus Controller
//                with the following key features:
//                - Precise positioning in steps and microns
//                - Configurable step size for different stages
//                - Support for multiple axes (0-2)
//                - Status monitoring and position feedback
//                - Home, Stop and Set Origin functionality
//
// AUTHOR:        Jens Eriksson, firs.lastname@imbim.uu.se
// COPYRIGHT:     Jens Eriksson, 2025
// LICENSE:       MIT
//

#pragma once
#include "MMDevice.h"
#include "DeviceBase.h"
#include "DeviceThreads.h"
#include "ModuleInterface.h"
#include <string>
#include <vector>
#include <map>

// Error codes for the MCM3000 device adapter (10000-10999 range reserved for device adapters)
#define ERR_PORT_CHANGE_FORBIDDEN    10004  // Cannot change port after initialization
#define ERR_UNRECOGNIZED_ANSWER      10009  // Invalid response from device
#define ERR_UNSPECIFIED_ERROR        10010  // Generic error
#define ERR_HOME_REQUIRED            10011  // Device must be homed before operation
#define ERR_INVALID_PACKET_LENGTH    10012  // Serial packet has wrong length
#define ERR_RESPONSE_TIMEOUT         10013  // Device did not respond in time
#define ERR_BUSY                     10014  // Device is busy with another operation
#define ERR_STEPS_OUT_OF_RANGE       10015  // Requested position exceeds limits
#define ERR_STAGE_NOT_ZEROED         10016  // Stage must be zeroed before use
#define ERR_INVALID_VALUE            10017  // Invalid parameter value

// Command packet lengths for serial communication
static const int SET_POS_LENGTH = 12;     // Length of position setting command
static const int QUERY_POS_LENGTH = 6;    // Length of position query command  
static const int STATUS_LENGTH = 6;       // Length of status query command

// Device identification constants
static const char* const g_DeviceName = "MCM3000";
static const char* const g_Description = "MCM3000 Focus Controller";

// Standard error messages
static const char* const g_Msg_PORT_CHANGE_FORBIDDEN = "Port change is not allowed after device has been initialized.";
static const char* const g_Msg_INVALID_STEP_SIZE = "Invalid step size";
static const char* const g_Msg_DEVICE_BUSY = "Device is busy";
static const char* const g_Msg_STEPS_OUT_OF_RANGE = "Position out of range";
static const char* const g_Msg_NOT_INITIALIZED = "Device not initialized";

/**
 * myFocusController class
 * 
 * Main device adapter class for the MCM3000 Focus Controller.
 * Implements the CStageBase interface for single-axis stage control.
 * 
 * Key capabilities:
 * - Absolute and relative positioning
 * - Position tracking in steps and microns
 * - Multiple stage support via configurable step sizes
 * - Hardware status monitoring
 * - Origin setting and homing functions
 */
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

    // Action interface - Property action handlers
    int OnPort(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnPosition(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnSetOrigin(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnStepSize(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnAxisID(MM::PropertyBase* pProp, MM::ActionType eAct);

    // Serial communication
    int SendCommand(const unsigned char* command, unsigned length);

    // Device identification
    static const char* DeviceName() { return g_DeviceName; }
    static const char* Description() { return g_Description; }

private:
    // Private helper functions
    int GetResponse(unsigned char* response, unsigned length);
    int ClearPort();
    int MoveBlocking(long steps, bool relative);

    // Timing and tolerance constants
    static const int MOTOR_STATUS_POLL_MS = 50;      // Poll interval for motor status
    static const int ENCODER_COUNT_TOLERANCE = 1;     // Minimum tolerance for position verification
    static const long INVALID_POSITION = 0x80000000;  // Invalid position marker

    // Command codes for serial protocol
    static const unsigned char AXIS_ID_BYTE = 0x01;     // For Stop, Query Position, Query Status
    static const uint16_t AXIS_ID_WORD = 0x0001;       // For Set encoder, Go to Position
    static const unsigned char CMD_STOP = 0x01;         // Stop command
    static const unsigned char CMD_QUERY_POS = 0x0A;    // Query position command
    static const unsigned char CMD_QUERY_STATUS = 0x80;  // Query status command
    static const unsigned char CMD_SET_ENCODER = 0x09;   // Set encoder command
    static const unsigned char CMD_GOTO_POS = 0x53;      // Go to position command

    // Device constants
    static constexpr double DEFAULT_STEP_SIZE_UM = 0.2116667;  // Default step size (ZFM2020/2030)
    static constexpr double POSITION_LIMIT_UM = 12700.0;       // ±12.7mm travel range

    // Step size lookup table (UI string -> exact value)
    static const std::map<std::string, double> STEP_SIZE_MAP;

    // Member variables
    bool initialized_;          // Initialization state
    std::string port_;         // Serial port name
    double stepSizeUm_;        // Current step size in microns
    double answerTimeoutMs_;   // Serial timeout
    bool home_;                // Home state
    long curSteps_;            // Current position in steps
    bool positionValid_;       // Position validity flag
    MM::MMTime lastMoveTime_;  // Timestamp of last move
    unsigned char lastCommand_; // Last command sent
    unsigned char axisID_;     // Current axis ID (0-2)
};
