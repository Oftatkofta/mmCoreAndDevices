#include "ThorlabsMCM3001.h"
#include "ModuleInterface.h"
#include <cstdio>
#include <sstream>

MODULE_API void InitializeModuleData()
{
    RegisterDevice("ThorlabsMCM3001", MM::StageDevice, "Thorlabs MCM3001 Stage");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    if (deviceName == 0)
        return 0;

    if (strcmp(deviceName, "ThorlabsMCM3001") == 0)
        return new ThorlabsMCM3001();

    return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
    delete pDevice;
}

ThorlabsMCM3001::ThorlabsMCM3001() :
    initialized_(false),
    busy_(false),
    stepSizeUm_(ENCODER_RESOLUTION_UM),
    posUm_(0.0),
    port_(""),
    currentAxis_(0)
{
    InitializeDefaultErrorMessages();

    SetErrorText(ERR_PORT_CHANGE_FORBIDDEN, "Port change is not allowed while the device is connected.");
    SetErrorText(ERR_INVALID_SERIAL_PARAMS, "Invalid serial port parameters.");
    SetErrorText(ERR_COMMAND_FAILED, "Command failed or no response from device.");
    SetErrorText(ERR_INVALID_AXIS, "Invalid axis specified (valid: 0-2).");
    
    CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false);

    CPropertyAction* pAct = new CPropertyAction(this, &ThorlabsMCM3001::OnAxis);
    CreateProperty("Axis", "0", MM::Integer, false, pAct);
    SetPropertyLimits("Axis", 0, 2);
}

ThorlabsMCM3001::~ThorlabsMCM3001()
{
    Shutdown();
}

void ThorlabsMCM3001::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(name, "ThorlabsMCM3001");
}

int ThorlabsMCM3001::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    if (port_.empty())
        return ERR_INVALID_SERIAL_PARAMS;

    int ret = SetupSerialPort();
    if (ret != DEVICE_OK)
        return ret;

    ret = ClearPort();
    if (ret != DEVICE_OK)
        return ret;

    // Add status property
    CPropertyAction* pAct = new CPropertyAction(this, &ThorlabsMCM3001::OnStatus);
    ret = CreateProperty("Status", "Idle", MM::String, true, pAct);
    if (ret != DEVICE_OK)
        return ret;

    initialized_ = true;
    return DEVICE_OK;
}

int ThorlabsMCM3001::Shutdown()
{
    if (initialized_)
    {
        initialized_ = false;
    }
    return DEVICE_OK;
}

bool ThorlabsMCM3001::Busy()
{
    MMThreadGuard guard(lock_);
    if (!initialized_)
        return false;

    CmdPacket6 cmd = {
        0x80,           // CMD_REQUEST_STATUS
        0x04,           // Length
        currentAxis_,   // Channel ID
        0x00,          // Param1
        0x00,          // Param2
        0x00           // Param3
    };

    if (SendCommand(cmd) != DEVICE_OK)
        return false;

    unsigned char response[34];
    if (ReadResponse(response, sizeof(response)) != DEVICE_OK)
        return false;

    return (response[16] & 0x30) != 0;
}

int ThorlabsMCM3001::SetPositionUm(double pos)
{
    MMThreadGuard guard(lock_);
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    long steps = UmToSteps(pos);
    
    CmdPacket12 cmd = {
        0x53,           // CMD_GOTO_POS
        0x04,           // Length
        0x06,           // Param1
        0x00,           // Param2
        0x00,           // Param3
        0x00,           // Param4
        currentAxis_,   // Channel ID
        steps          // Position value
    };

    int ret = SendCommand(cmd);
    if (ret != DEVICE_OK)
    {
        LogError("Failed to set position");
        return ret;
    }

    busy_ = true;
    return DEVICE_OK;
}

int ThorlabsMCM3001::GetPositionUm(double& pos)
{
    MMThreadGuard guard(lock_);
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    CmdPacket6 cmd = {
        0x0A,           // CMD_QUERY_POS
        0x04,           // Length
        currentAxis_,   // Channel ID
        0x00,          // Param1
        0x00,          // Param2
        0x00           // Param3
    };

    int ret = SendCommand(cmd);
    if (ret != DEVICE_OK)
        return ret;

    unsigned char response[12];
    ret = ReadResponse(response, sizeof(response));
    if (ret != DEVICE_OK)
        return ret;

    // Extract position value (bytes 8-11, little endian)
    int32_t steps = 
        (response[11] << 24) | 
        (response[10] << 16) | 
        (response[9] << 8) | 
        response[8];

    pos = StepsToUm(steps);
    return DEVICE_OK;
}

int ThorlabsMCM3001::SetPositionSteps(long steps)
{
    MMThreadGuard guard(lock_);
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    CmdPacket12 cmd = {
        0x53,           // CMD_GOTO_POS
        0x04,           // Length
        0x06,           // Param1
        0x00,           // Param2
        0x00,           // Param3
        0x00,           // Param4
        currentAxis_,   // Channel ID
        steps          // Position value
    };

    int ret = SendCommand(cmd);
    if (ret != DEVICE_OK)
        return ret;

    busy_ = true;
    return DEVICE_OK;
}

int ThorlabsMCM3001::GetPositionSteps(long& steps)
{
    MMThreadGuard guard(lock_);
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    CmdPacket6 cmd = {
        0x0A,           // CMD_QUERY_POS
        0x04,           // Length
        currentAxis_,   // Channel ID
        0x00,          // Param1
        0x00,          // Param2
        0x00           // Param3
    };

    int ret = SendCommand(cmd);
    if (ret != DEVICE_OK)
        return ret;

    unsigned char response[12];
    ret = ReadResponse(response, sizeof(response));
    if (ret != DEVICE_OK)
        return ret;

    steps = (response[11] << 24) | 
            (response[10] << 16) | 
            (response[9] << 8) | 
            response[8];

    return DEVICE_OK;
}

int ThorlabsMCM3001::SetOrigin()
{
    MMThreadGuard guard(lock_);
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    CmdPacket12 cmd = {
        0x09,           // CMD_SET_POSITION
        0x04,           // Length
        0x06,           // Param1
        0x00,           // Param2
        0x00,           // Param3
        0x00,           // Param4
        currentAxis_,   // Channel ID
        0              // Set current position as zero
    };

    return SendCommand(cmd);
}

int ThorlabsMCM3001::GetLimits(double& lower, double& upper)
{
    lower = -25000.0; // These values should be adjusted based on your stage
    upper = 25000.0;  // These values should be adjusted based on your stage
    return DEVICE_OK;
}

int ThorlabsMCM3001::Home()
{
    MMThreadGuard guard(lock_);
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    // For MCM3001, homing is setting position to 0
    return SetPositionUm(0.0);
}

int ThorlabsMCM3001::IsStageSequenceable(bool& isSequenceable) const
{
    isSequenceable = false;
    return DEVICE_OK;
}

bool ThorlabsMCM3001::IsContinuousFocusDrive() const
{
    return false;
}

// Property handlers
int ThorlabsMCM3001::OnPort(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(port_.c_str());
    }
    else if (eAct == MM::AfterSet)
    {
        if (initialized_)
        {
            pProp->Set(port_.c_str());
            return ERR_PORT_CHANGE_FORBIDDEN;
        }
        pProp->Get(port_);
    }
    return DEVICE_OK;
}

int ThorlabsMCM3001::OnStepSize(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(CDeviceUtils::ConvertToString(stepSizeUm_));
    }
    return DEVICE_OK;
}

int ThorlabsMCM3001::OnAxis(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set((long)currentAxis_);
    }
    else if (eAct == MM::AfterSet)
    {
        long axis;
        pProp->Get(axis);
        if (axis < 0 || axis > 2)
            return ERR_INVALID_AXIS;
        currentAxis_ = (uint16_t)axis;
    }
    return DEVICE_OK;
}

int ThorlabsMCM3001::OnStatus(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(busy_ ? "Busy" : "Idle");
    }
    return DEVICE_OK;
}

// Utility functions
int ThorlabsMCM3001::SendCommand(const CmdPacket6& cmd)
{
    return WriteToComPort(port_.c_str(), reinterpret_cast<const unsigned char*>(&cmd), sizeof(cmd));
}

int ThorlabsMCM3001::SendCommand(const CmdPacket12& cmd)
{
    return WriteToComPort(port_.c_str(), reinterpret_cast<const unsigned char*>(&cmd), sizeof(cmd));
}

int ThorlabsMCM3001::ReadResponse(unsigned char* response, unsigned length)
{
    const unsigned long timeoutMs = 1000;
    MM::MMTime startTime = GetCurrentMMTime();
    unsigned long bytesRead = 0;
    unsigned long totalBytesRead = 0;

    while (totalBytesRead < length)
    {
        if ((GetCurrentMMTime() - startTime).getMsec() > timeoutMs)
            return DEVICE_SERIAL_TIMEOUT;

        int ret = ReadFromComPort(port_.c_str(), response + totalBytesRead, 
                                length - totalBytesRead, bytesRead);
        if (ret != DEVICE_OK)
            return ret;

        if (bytesRead == 0)
            CDeviceUtils::SleepMs(1);
        else
            totalBytesRead += bytesRead;
    }

    return DEVICE_OK;
}

int ThorlabsMCM3001::WaitForResponse(unsigned timeoutMs)
{
    MM::MMTime startTime = GetCurrentMMTime();
    unsigned long bytesRead = 0;
    unsigned char dummy;
    
    do {
        if (ReadFromComPort(port_.c_str(), &dummy, 1, bytesRead) == DEVICE_OK && bytesRead > 0)
            return DEVICE_OK;
        CDeviceUtils::SleepMs(1);
    } 
    while ((GetCurrentMMTime() - startTime).getMsec() < timeoutMs);
    
    return DEVICE_SERIAL_TIMEOUT;
}

int ThorlabsMCM3001::ClearPort()
{
    return PurgeComPort(port_.c_str());
}

int ThorlabsMCM3001::SetupSerialPort()
{
    MM::Device* pSerialDevice = GetDevice(port_.c_str());
    if (!pSerialDevice)
        return -1; // DEVICE_SERIAL_INVALID_DEVICE is not defined, replaced with a placeholder return value

    // Configure port settings
    pSerialDevice->SetProperty(MM::g_Keyword_BaudRate, CDeviceUtils::ConvertToString(460800));
    pSerialDevice->SetProperty(MM::g_Keyword_DataBits, CDeviceUtils::ConvertToString(8));
    pSerialDevice->SetProperty(MM::g_Keyword_StopBits, CDeviceUtils::ConvertToString(1));
    pSerialDevice->SetProperty(MM::g_Keyword_Parity, "None");
    pSerialDevice->SetProperty(MM::g_Keyword_Handshaking, "Off");

    return DEVICE_OK;
}

void ThorlabsMCM3001::LogError(const char* message)
{
    char buf[MM::MaxStrLength];
    snprintf(buf, MM::MaxStrLength, "ThorlabsMCM3001: %s", message);
    LogMessage(buf, false);
}




