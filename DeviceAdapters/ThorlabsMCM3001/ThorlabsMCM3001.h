#ifndef _THORLABS_MCM3001_H_
#define _THORLABS_MCM3001_H_

#include "MMDevice.h"
#include "DeviceBase.h"
#include <string>

// Error codes
const int ERR_PORT_CHANGE_FORBIDDEN = 101;
const int ERR_INVALID_AXIS = 102;
const int ERR_COMMAND_FAILED = 103;

// Command codes
const uint8_t CMD_SET_ENCODER = 0x09;    // Set encoder counter
const uint8_t CMD_STOP = 0x65;           // Stop any motor move
const uint8_t CMD_QUERY_POS = 0x0A;      // Query position
const uint8_t CMD_GOTO_POS = 0x53;       // Go to position
const uint8_t CMD_REQUEST_STATUS = 0x80;  // Request motor status

// Command packet structures (match hardware protocol)
struct CmdPacket6 {
    uint8_t cmd;        // Command byte
    uint8_t length;     // Always 0x04
    uint8_t channelId;  // Single byte for channel
    uint8_t param1;     // e.g., Stop Mode
    uint8_t param2;     // Always 0x00
    uint8_t param3;     // Always 0x00
};

struct CmdPacket12 {
    uint8_t cmd;        // Command byte
    uint8_t length;     // Always 0x04
    uint8_t param1;     // Always 0x06
    uint8_t param2;     // Always 0x00
    uint8_t param3;     // Always 0x00
    uint8_t param4;     // Always 0x00
    uint16_t channelId; // Two bytes for channel
    int32_t value;      // Four bytes, little endian
};

// Response packet structures
struct PosResponse {
    uint8_t header[6];  // Response header
    uint8_t data[6];    // Data packet containing position
};

struct StatusResponse {
    uint8_t header[6];  // Response header
    uint8_t data[28];   // Busy status in byte 16: true if (byte16 & 0x30)
};

class ThorlabsMCM3001 : public CStageBase<ThorlabsMCM3001>
{
public:
    // For ZFM2020/ZFM2030
    static const double DEFAULT_ENCODER_RESOLUTION_UM;  // Defined in cpp file

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
    int OnAxis(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnEncoderResolution(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
    // Internal state
    bool initialized_;
    bool busy_;
    double stepSizeUm_;
    double posUm_;
    std::string port_;
    uint16_t currentAxis_;
    double encoderResolutionUm_;
    MMThreadLock lock_;

    // Utility functions
    int SendCommand(const CmdPacket6& cmd);
    int SendCommand(const CmdPacket12& cmd);
    int ReadResponse(unsigned char* response, unsigned length);
    int WaitForResponse(unsigned timeoutMs = 500);
    int ClearPort();
    int SetupSerialPort();
    
    // Conversion functions
    long UmToSteps(double um) const;
    double StepsToUm(long steps) const;
};

#endif  // _THORLABS_MCM3001_H_

