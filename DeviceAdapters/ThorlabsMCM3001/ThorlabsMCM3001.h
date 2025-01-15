#pragma once

#include "MMDevice.h"
#include "DeviceBase.h"
#include "ModuleInterface.h"
#include <string>

// Device specific constants
const double ENCODER_RESOLUTION_UM = 0.2116667; // μm per count

// Command codes
const uint8_t CMD_SET_POSITION = 0x09;
const uint8_t CMD_STOP = 0x65;
const uint8_t CMD_QUERY_POS = 0x0A;
const uint8_t CMD_GOTO_POS = 0x53;
const uint8_t CMD_REQUEST_STATUS = 0x80;

// Error codes
const int ERR_PORT_CHANGE_FORBIDDEN = 10002;
const int ERR_INVALID_SERIAL_PARAMS = 10003;
const int ERR_COMMAND_FAILED = 10004;

class ThorlabsMCM3001 : public CStageBase<ThorlabsMCM3001>
{
public:
    ThorlabsMCM3001();
    ~ThorlabsMCM3001();

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
    int SetOrigin();
    int GetLimits(double& lower, double& upper);
    int IsStageSequenceable(bool& isSequenceable) const { isSequenceable = false; return DEVICE_OK; }
    bool IsContinuousFocusDrive() const { return false; }

    // Action interface
    int OnPort(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnStepSize(MM::PropertyBase* pProp, MM::ActionType eAct);

    int Home();

private:
    bool initialized_;
    bool busy_;
    double stepSizeUm_;
    double posUm_;
    std::string port_;
    MMThreadLock lock_;

    // Utility functions
    int SendCommand(const unsigned char* command, unsigned length);
    int ReadResponse(unsigned char* response, unsigned length);
    int WaitForResponse(unsigned timeoutMs = 500);
    int ClearPort();
    int SetupSerialPort();
    
    // Conversion functions
    long UmToSteps(double um) { return static_cast<long>(um / ENCODER_RESOLUTION_UM); }
    double StepsToUm(long steps) { return steps * ENCODER_RESOLUTION_UM; }
};

