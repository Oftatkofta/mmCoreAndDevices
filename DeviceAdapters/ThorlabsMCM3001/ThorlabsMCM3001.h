#pragma once

#include "MMDevice.h"
#include "DeviceBase.h"
#include "ModuleInterface.h"
#include <string>

// Device specific constants
const double ENCODER_RESOLUTION_UM = 0.2116667; // μm per count

// Error codes
const int ERR_PORT_CHANGE_FORBIDDEN = 10002;
const int ERR_INVALID_SERIAL_PARAMS = 10003;
const int ERR_COMMAND_FAILED = 10004;
const int ERR_INVALID_AXIS = 10005;

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
    int IsStageSequenceable(bool& isSequenceable) const;
    bool IsContinuousFocusDrive() const;
    int Home();

    // Action interface
    int OnPort(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnStepSize(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnAxis(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnStatus(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
    bool initialized_;
    bool busy_;
    double stepSizeUm_;
    double posUm_;
    std::string port_;
    uint16_t currentAxis_;
    MMThreadLock lock_;

    // Command structures
    struct CmdPacket6 {
        uint8_t cmd;
        uint8_t length;
        uint8_t channelId;
        uint8_t param1;
        uint8_t param2;
        uint8_t param3;
    };

    struct CmdPacket12 {
        uint8_t cmd;
        uint8_t length;
        uint8_t param1;
        uint8_t param2;
        uint8_t param3;
        uint8_t param4;
        uint16_t channelId;
        int32_t value;
    };

    // Utility functions
    int SendCommand(const CmdPacket6& cmd);
    int SendCommand(const CmdPacket12& cmd);
    int ReadResponse(unsigned char* response, unsigned length);
    int WaitForResponse(unsigned timeoutMs = 500);
    int ClearPort();
    int SetupSerialPort();
    void LogError(const char* message);
    
    // Conversion functions
    long UmToSteps(double um) const { return static_cast<long>(um / ENCODER_RESOLUTION_UM); }
    double StepsToUm(long steps) const { return steps * ENCODER_RESOLUTION_UM; }
};

