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
    port_("")
{
    InitializeDefaultErrorMessages();

    // Add custom error messages
    SetErrorText(ERR_PORT_CHANGE_FORBIDDEN, "Port change is not allowed while the device is connected.");
    
    // Create pre-initialization properties
    CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false);
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

    // Set up serial port
    if (port_.empty())
        return DEVICE_ERR;

    int ret = SetSerialProperties(port_.c_str(),
        "500",    // timeout
        "460800", // baud
        "0",      // delayBetweenChars
        "None",   // handshaking
        "None",   // parity
        "1");     // stopBits
    if (ret != DEVICE_OK)
        return ret;

    // Clear communication
    ret = ClearPort();
    if (ret != DEVICE_OK)
        return ret;

    // Create post-initialization properties
    CreateProperty("StepSize", CDeviceUtils::ConvertToString(stepSizeUm_), 
        MM::Float, true);

    initialized_ = true;
    return DEVICE_OK;
}

int ThorlabsMCM3001::Shutdown()
{
    if (initialized_)
    {
        Stop();
        initialized_ = false;
    }
    return DEVICE_OK;
}

bool ThorlabsMCM3001::Busy()
{
    MMThreadGuard guard(lock_);
    return IsControllerBusy();
}

int ThorlabsMCM3001::SetPositionUm(double pos)
{
    MMThreadGuard guard(lock_);
    
    long steps = UmToSteps(pos);
    uint8_t cmd[] = {
        CMD_GOTO_POS, 0x04, 0x06, 0x00, 0x00, 0x00,
        0x00, 0x00,  // Channel ID
        static_cast<uint8_t>(steps & 0xFF),
        static_cast<uint8_t>((steps >> 8) & 0xFF),
        static_cast<uint8_t>((steps >> 16) & 0xFF),
        static_cast<uint8_t>((steps >> 24) & 0xFF)
    };

    int ret = SendCommand(cmd, sizeof(cmd));
    if (ret != DEVICE_OK)
        return ret;

    busy_ = true;
    return DEVICE_OK;
}

int ThorlabsMCM3001::GetPositionUm(double& pos)
{
    MMThreadGuard guard(lock_);
    
    uint8_t cmd[] = {CMD_QUERY_POS, 0x04, 0x00, 0x00, 0x00, 0x00};
    int ret = SendCommand(cmd, sizeof(cmd));
    if (ret != DEVICE_OK)
        return ret;

    uint8_t response[12];
    ret = ReadResponse(response, sizeof(response));
    if (ret != DEVICE_OK)
        return ret;

    int32_t steps = 
        (response[11] << 24) | 
        (response[10] << 16) | 
        (response[9] << 8) | 
        response[8];

    pos = StepsToUm(steps);
    posUm_ = pos;
    return DEVICE_OK;
}

int ThorlabsMCM3001::SetPositionSteps(long steps)
{
    return SetPositionUm(StepsToUm(steps));
}

int ThorlabsMCM3001::GetPositionSteps(long& steps)
{
    double pos;
    int ret = GetPositionUm(pos);
    if (ret != DEVICE_OK)
        return ret;
    
    steps = UmToSteps(pos);
    return DEVICE_OK;
}

int ThorlabsMCM3001::SetOrigin()
{
    MMThreadGuard guard(lock_);
    
    uint8_t cmd[] = {
        CMD_SET_POSITION, 0x04, 0x06, 0x00, 0x00, 0x00,
        0x00, 0x00,  // Channel ID
        0x00, 0x00, 0x00, 0x00  // Zero position
    };

    return SendCommand(cmd, sizeof(cmd));
}

int ThorlabsMCM3001::GetLimits(double& lower, double& upper)
{
    // These should be determined from the device specifications
    lower = -25000.0; // μm
    upper = 25000.0;  // μm
    return DEVICE_OK;
}

int ThorlabsMCM3001::Stop()
{
    MMThreadGuard guard(lock_);
    
    uint8_t cmd[] = {CMD_STOP, 0x04, 0x00, 0x01, 0x00, 0x00};
    return SendCommand(cmd, sizeof(cmd));
}

bool ThorlabsMCM3001::IsControllerBusy()
{
    uint8_t cmd[] = {CMD_REQUEST_STATUS, 0x04, 0x00, 0x00, 0x00, 0x00};
    if (SendCommand(cmd, sizeof(cmd)) != DEVICE_OK)
        return false;

    uint8_t response[34];
    if (ReadResponse(response, sizeof(response)) != DEVICE_OK)
        return false;

    return (response[16] & 0x30) != 0;
}

int ThorlabsMCM3001::SendCommand(const unsigned char* command, unsigned length)
{
    int ret = WriteToComPort(port_.c_str(), command, length);
    if (ret != DEVICE_OK)
        return ret;

    return WaitForResponse();
}

int ThorlabsMCM3001::ReadResponse(unsigned char* response, unsigned length)
{
    unsigned long bytesRead = 0;
    int ret = ReadFromComPort(port_.c_str(), response, length, bytesRead);
    if (ret != DEVICE_OK)
        return ret;

    if (bytesRead != length)
        return DEVICE_SERIAL_INVALID_RESPONSE;

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
    } 
    while ((GetCurrentMMTime() - startTime).getMsec() < timeoutMs);
    
    return DEVICE_SERIAL_TIMEOUT;
}

int ThorlabsMCM3001::ClearPort()
{
    return PurgeComPort(port_.c_str());
}

bool ThorlabsMCM3001::IsStageSequenceable(bool& isSequenceable) const
{
    isSequenceable = false;
    return true;
}

bool ThorlabsMCM3001::IsContinuousFocusDrive() const
{
    return false;
}

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
        pProp->Set(stepSizeUm_);
    }
    return DEVICE_OK;
}




