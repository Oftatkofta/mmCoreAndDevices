#include "ThorlabsMCM3001.h"
#include "ModuleInterface.h"
#include "DeviceUtils.h"  // For CDeviceUtils
#include <sstream>

// Default encoder resolution for MCM3001 with ZFM2020/ZFM2030 stages (in micrometers per count)
const double ThorlabsMCM3001::DEFAULT_ENCODER_RESOLUTION_UM = 0.2116667;

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
    stepSizeUm_(0.1),
    posUm_(0.0),
    port_(""),
    currentAxis_(0),
    encoderResolutionUm_(DEFAULT_ENCODER_RESOLUTION_UM)
{
    InitializeDefaultErrorMessages();

    // Add custom error messages using MM's message system
    SetErrorText(ERR_PORT_CHANGE_FORBIDDEN, "Port cannot be changed while device is in use.");
    SetErrorText(ERR_INVALID_AXIS, "Invalid axis specified (valid: 0-2).");
    SetErrorText(ERR_COMMAND_FAILED, "Command failed or no response from device.");
    SetErrorText(DEVICE_SERIAL_COMMAND_FAILED, "Command failed or no response from device.");
    SetErrorText(DEVICE_NOT_CONNECTED, "Invalid serial port configuration.");
    SetErrorText(DEVICE_INVALID_PROPERTY_VALUE, "Invalid axis specified (valid: 0-2).");
    
    // Create pre-initialization properties
    // Serial port
    CPropertyAction* pAct = new CPropertyAction (this, &ThorlabsMCM3001::OnPort);
    CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);

    // Create axis selection property
    pAct = new CPropertyAction(this, &ThorlabsMCM3001::OnAxis);
    CreateProperty("Axis", "0", MM::Integer, false, pAct);
    SetPropertyLimits("Axis", 0, 2);

    // Add encoder resolution property with detailed description
    pAct = new CPropertyAction(this, &ThorlabsMCM3001::OnEncoderResolution);
    std::ostringstream defaultValue;
    defaultValue.precision(7);
    defaultValue << DEFAULT_ENCODER_RESOLUTION_UM;
    
    // Create property with description
    CreateProperty("EncoderResolution(um/count)", 
                  defaultValue.str().c_str(), 
                  MM::Float, 
                  false, 
                  pAct, 
                  "Stage-specific conversion factor (micrometers per encoder count). "
                  "Default value 0.2116667 is calibrated for Thorlabs ZFM2020 and ZFM2030 stages. "
                  "Change only if using a different stage model.");
                  
    SetPropertyLimits("EncoderResolution(um/count)", 0.0001, 10.0);  // Reasonable limits
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
    // Check if already initialized
    if (initialized_)
        return DEVICE_OK;

    // Get the port from MM's device manager
    MM::Device* pDevice = GetCoreCallback()->GetDevice(this, port_.c_str());
    if (pDevice == NULL)
    {
        LogMessage("Invalid serial port for MCM3001");
        return DEVICE_INVALID_PROPERTY_VALUE;
    }

    // Configure the serial port using MM's property interface
    // Default settings for MCM3001:
    // Baud Rate = 460800, Data bits = 8, Parity = None, Stop bits = 1, Flow control = None
    MM::Serial* pSerial = static_cast<MM::Serial*>(pDevice);
    
    int ret = pSerial->SetProperty(MM::g_Keyword_BaudRate, "460800");
    if (ret != DEVICE_OK) return ret;
    
    ret = pSerial->SetProperty(MM::g_Keyword_DataBits, "8");
    if (ret != DEVICE_OK) return ret;
    
    ret = pSerial->SetProperty(MM::g_Keyword_StopBits, "1");
    if (ret != DEVICE_OK) return ret;
    
    ret = pSerial->SetProperty(MM::g_Keyword_Parity, "None");
    if (ret != DEVICE_OK) return ret;
    
    ret = pSerial->SetProperty(MM::g_Keyword_Handshaking, "Off");
    if (ret != DEVICE_OK) return ret;
    
    // Clear communication
    ret = ClearPort();
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
        CMD_REQUEST_STATUS,  // Command byte
        0x04,               // Length
        static_cast<uint8_t>(currentAxis_ & 0xFF), // Channel ID
        0x00,               // Param1
        0x00,               // Param2
        0x00                // Param3
    };

    if (SendCommand(cmd) != DEVICE_OK)
        return false;

    StatusResponse response;
    if (ReadResponse(reinterpret_cast<unsigned char*>(&response), sizeof(response)) != DEVICE_OK)
        return false;

    busy_ = (response.data[16] & 0x30) != 0;
    return busy_;
}

int ThorlabsMCM3001::SetPositionUm(double pos)
{
    MMThreadGuard guard(lock_);
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    long steps = UmToSteps(pos);
    
    CmdPacket12 cmd = {
        CMD_GOTO_POS,       // Command byte
        0x04,               // Length
        0x06,               // Param1
        0x00,               // Param2
        0x00,               // Param3
        0x00,               // Param4
        currentAxis_,       // Channel ID
        steps              // Position value
    };

    int ret = SendCommand(cmd);
    if (ret != DEVICE_OK)
        return ret;

    busy_ = true;
    return DEVICE_OK;
}

int ThorlabsMCM3001::GetPositionUm(double& pos)
{
    MMThreadGuard guard(lock_);
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    CmdPacket6 cmd = {
        CMD_QUERY_POS,      // Command byte
        0x04,               // Length
        static_cast<uint8_t>(currentAxis_ & 0xFF), // Channel ID
        0x00,               // Param1
        0x00,               // Param2
        0x00                // Param3
    };

    int ret = SendCommand(cmd);
    if (ret != DEVICE_OK)
        return ret;

    PosResponse response;
    ret = ReadResponse(reinterpret_cast<unsigned char*>(&response), sizeof(response));
    if (ret != DEVICE_OK)
        return ret;

    // Extract position from data packet
    int32_t steps = 
        (response.data[5] << 24) | 
        (response.data[4] << 16) | 
        (response.data[3] << 8) | 
        response.data[2];

    pos = StepsToUm(steps);
    return DEVICE_OK;
}

int ThorlabsMCM3001::SetPositionSteps(long steps)
{
    MMThreadGuard guard(lock_);
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    CmdPacket12 cmd = {
        CMD_GOTO_POS,       // Command byte
        0x04,               // Length
        0x06,               // Param1
        0x00,               // Param2
        0x00,               // Param3
        0x00,               // Param4
        currentAxis_,       // Channel ID
        steps              // Position value
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
        CMD_QUERY_POS,      // Command byte
        0x04,               // Length
        static_cast<uint8_t>(currentAxis_ & 0xFF), // Channel ID
        0x00,               // Param1
        0x00,               // Param2
        0x00                // Param3
    };

    int ret = SendCommand(cmd);
    if (ret != DEVICE_OK)
        return ret;

    PosResponse response;
    ret = ReadResponse(reinterpret_cast<unsigned char*>(&response), sizeof(response));
    if (ret != DEVICE_OK)
        return ret;

    // Extract position from data packet
    steps = 
        (response.data[5] << 24) | 
        (response.data[4] << 16) | 
        (response.data[3] << 8) | 
        response.data[2];

    return DEVICE_OK;
}

int ThorlabsMCM3001::SetOrigin()
{
    MMThreadGuard guard(lock_);
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    CmdPacket6 cmd = {
        CMD_SET_ENCODER,    // Command byte
        0x04,               // Length
        static_cast<uint8_t>(currentAxis_ & 0xFF), // Channel ID
        0x00,               // Param1
        0x00,               // Param2
        0x00                // Param3
    };

    return SendCommand(cmd);
}

int ThorlabsMCM3001::GetLimits(double& lower, double& upper)
{
    lower = 0;
    upper = 25000;  // 25mm travel range
    return DEVICE_OK;
}

bool ThorlabsMCM3001::IsContinuousFocusDrive() const
{
    return false;
}

int ThorlabsMCM3001::IsStageSequenceable(bool& isSequenceable) const
{
    isSequenceable = false;
    return DEVICE_OK;
}

int ThorlabsMCM3001::Home()
{
    return DEVICE_UNSUPPORTED_COMMAND;
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
            return DEVICE_INVALID_PROPERTY_VALUE;
        }
        pProp->Get(port_);
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
            return DEVICE_INVALID_PROPERTY_VALUE;
        currentAxis_ = (uint16_t)axis;
    }
    return DEVICE_OK;
}

int ThorlabsMCM3001::OnEncoderResolution(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(encoderResolutionUm_);
    }
    else if (eAct == MM::AfterSet)
    {
        if (initialized_)
        {
            pProp->Set(encoderResolutionUm_); // Revert to previous value
            return DEVICE_INVALID_PROPERTY_VALUE;  // Can't change after initialization
        }
            
        double resolution;
        pProp->Get(resolution);
        if (resolution <= 0.0)
        {
            // If invalid, revert to default and explain
            encoderResolutionUm_ = DEFAULT_ENCODER_RESOLUTION_UM;
            pProp->Set(DEFAULT_ENCODER_RESOLUTION_UM);
            LogMessage("Invalid encoder resolution. Reverting to default value (0.2116667 um/count for ZFM2020/ZFM2030).");
            return DEVICE_INVALID_PROPERTY_VALUE;
        }
            
        encoderResolutionUm_ = resolution;
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

long ThorlabsMCM3001::UmToSteps(double um) const
{
    return static_cast<long>(um / encoderResolutionUm_);
}

double ThorlabsMCM3001::StepsToUm(long steps) const
{
    return steps * encoderResolutionUm_;
}




