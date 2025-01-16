#include "ThorlabsMCM3001.h"
#include "ModuleInterface.h"
#include "DeviceUtils.h"
#include <sstream>

// Define the static member
const double ThorlabsMCM3001::DEFAULT_ENCODER_RESOLUTION_UM = 0.2116667;  // For ZFM2020/ZFM2030

MODULE_API void InitializeModuleData()
{
    RegisterDevice("ThorlabsMCM3001", 
                  MM::StageDevice, 
                  "Thorlabs MCM3001 3-Channel Controller for Motorized Stages (ZFM2020/ZFM2030)");
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
    port_("Undefined"),
    currentAxis_(0),
    encoderResolutionUm_(DEFAULT_ENCODER_RESOLUTION_UM)
{
    InitializeDefaultErrorMessages();

    // Add custom error messages
    SetErrorText(ERR_PORT_CHANGE_FORBIDDEN, "Port cannot be changed while device is in use.");
    SetErrorText(ERR_INVALID_AXIS, "Invalid axis specified (valid: 0-2).");
    SetErrorText(ERR_COMMAND_FAILED, "Command failed or no response from device.");
    SetErrorText(DEVICE_SERIAL_COMMAND_FAILED, "Command failed or no response from device.");
    SetErrorText(DEVICE_NOT_CONNECTED, "Invalid serial port configuration.");
    SetErrorText(DEVICE_INVALID_PROPERTY_VALUE, "Invalid axis specified (valid: 0-2).");
    
    // Create pre-initialization properties
    CPropertyAction* pAct = new CPropertyAction (this, &ThorlabsMCM3001::OnPort);
    CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);

    // Create axis selection property
    pAct = new CPropertyAction(this, &ThorlabsMCM3001::OnAxis);
    CreateProperty("Axis", "0", MM::Integer, false, pAct);
    SetPropertyLimits("Axis", 0, 2);

    // Add encoder resolution property
    pAct = new CPropertyAction(this, &ThorlabsMCM3001::OnEncoderResolution);
    std::ostringstream defaultValue;
    defaultValue.precision(7);
    defaultValue << DEFAULT_ENCODER_RESOLUTION_UM;
    
    CreateProperty("EncoderResolution(um/count)", 
                  defaultValue.str().c_str(), 
                  MM::Float, 
                  false, 
                  pAct, 
                  "Stage-specific conversion factor (micrometers per encoder count). "
                  "Default value 0.2116667 is calibrated for Thorlabs ZFM2020 and ZFM2030 stages. "
                  "Change only if using a different stage model.");
                  
    SetPropertyLimits("EncoderResolution(um/count)", 0.0001, 10.0);
}

int ThorlabsMCM3001::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    // Log port information
    LogMessage("Initializing MCM3001 on port " + port_);

    // Get the port from MM's device manager
    MM::Device* pDevice = GetCoreCallback()->GetDevice(this, port_.c_str());
    if (pDevice == NULL)
    {
        LogMessage("Failed to get port device " + port_);
        return DEVICE_INVALID_PROPERTY_VALUE;
    }

    MM::Serial* pSerial = static_cast<MM::Serial*>(pDevice);
    
    // Configure serial port with logging
    LogMessage("Setting serial parameters...");
    int ret = pSerial->SetProperty(MM::g_Keyword_BaudRate, "460800");
    if (ret != DEVICE_OK) {
        LogMessage("Failed to set baud rate");
        return ret;
    }
    
    ret = pSerial->SetProperty(MM::g_Keyword_DataBits, "8");
    if (ret != DEVICE_OK) {
        LogMessage("Failed to set data bits");
        return ret;
    }
    
    ret = pSerial->SetProperty(MM::g_Keyword_StopBits, "1");
    if (ret != DEVICE_OK) {
        LogMessage("Failed to set stop bits");
        return ret;
    }
    
    ret = pSerial->SetProperty(MM::g_Keyword_Parity, "None");
    if (ret != DEVICE_OK) {
        LogMessage("Failed to set parity");
        return ret;
    }
    
    ret = pSerial->SetProperty(MM::g_Keyword_Handshaking, "Off");
    if (ret != DEVICE_OK) {
        LogMessage("Failed to set handshaking");
        return ret;
    }

    LogMessage("Attempting to clear port...");
    ret = ClearPort();
    if (ret != DEVICE_OK) {
        LogMessage("Failed to clear port");
        return ret;
    }

    // Try to communicate with the device
    LogMessage("Testing device communication...");
    CmdPacket6 cmd = {
        CMD_REQUEST_STATUS,
        0x04,
        static_cast<uint8_t>(currentAxis_ & 0xFF),
        0x00,
        0x00,
        0x00
    };

    ret = SendCommand(cmd);
    if (ret != DEVICE_OK) {
        LogMessage("Failed to send initial status request");
        return ret;
    }

    StatusResponse response;
    ret = ReadResponse(reinterpret_cast<unsigned char*>(&response), sizeof(response));
    if (ret != DEVICE_OK) {
        LogMessage("No response from device");
        return ret;
    }

    LogMessage("Device initialized successfully");
    initialized_ = true;
    return DEVICE_OK;
}

int ThorlabsMCM3001::SendCommand(const CmdPacket6& cmd)
{
    MMThreadGuard guard(lock_);
    
    std::stringstream ss;
    ss << "Sending 6-byte command: 0x" << std::hex << (int)cmd.cmd 
       << " to axis " << std::dec << (int)cmd.channelId;
    LogMessage(ss.str(), true);
    
    MM::Device* pDevice = GetCoreCallback()->GetDevice(this, port_.c_str());
    if (pDevice == NULL)
    {
        LogMessage("Failed to get serial port device", false);
        return DEVICE_INVALID_PROPERTY_VALUE;
    }
    
    MM::Serial* pSerial = static_cast<MM::Serial*>(pDevice);
    unsigned char* buf = (unsigned char*)&cmd;
    int ret = pSerial->Write(buf, sizeof(CmdPacket6));
    
    if (ret != DEVICE_OK)
        LogMessage("Command send failed", false);
    else
        LogMessage("Command sent successfully", true);
        
    return ret;
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
            // Prevent changing the port while initialized
            pProp->Set(port_.c_str());
            return ERR_PORT_CHANGE_FORBIDDEN;
        }
        
        pProp->Get(port_);
        LogMessage("Port changed to: " + port_);
    }
    
    return DEVICE_OK;
}

int ThorlabsMCM3001::SetPositionSteps(long steps)
{
    MMThreadGuard guard(lock_);
    
    // Create goto position command
    CmdPacket12 cmd = {
        CMD_GOTO_POS,    // Command byte
        0x04,            // Length
        0x06,            // Param1 (always 0x06)
        0x00,            // Param2
        0x00,            // Param3
        0x00,            // Param4
        currentAxis_,    // Channel ID
        steps           // Target position in encoder counts
    };

    LogMessage("Setting position to " + std::to_string(steps) + " steps");
    
    int ret = SendCommand(cmd);
    if (ret != DEVICE_OK)
    {
        LogMessage("Failed to send goto position command");
        return ret;
    }

    // Wait for move to complete
    do {
        CmdPacket6 statusCmd = {
            CMD_REQUEST_STATUS,
            0x04,
            static_cast<uint8_t>(currentAxis_ & 0xFF),
            0x00,
            0x00,
            0x00
        };

        ret = SendCommand(statusCmd);
        if (ret != DEVICE_OK)
            return ret;

        StatusResponse response;
        ret = ReadResponse(reinterpret_cast<unsigned char*>(&response), sizeof(response));
        if (ret != DEVICE_OK)
            return ret;

        busy_ = (response.data[16] & 0x30) != 0;
        
        if (busy_)
            CDeviceUtils::SleepMs(50);  // Wait before checking again
            
    } while (busy_);

    return DEVICE_OK;
}

int ThorlabsMCM3001::GetPositionSteps(long& steps)
{
    MMThreadGuard guard(lock_);
    
    // Create query position command
    CmdPacket6 cmd = {
        CMD_QUERY_POS,   // Command byte
        0x04,            // Length
        static_cast<uint8_t>(currentAxis_ & 0xFF),  // Channel ID
        0x00,            // Param1
        0x00,            // Param2
        0x00             // Param3
    };

    int ret = SendCommand(cmd);
    if (ret != DEVICE_OK)
    {
        LogMessage("Failed to send position query command");
        return ret;
    }

    // Read response
    PosResponse response;
    ret = ReadResponse(reinterpret_cast<unsigned char*>(&response), sizeof(response));
    if (ret != DEVICE_OK)
    {
        LogMessage("Failed to read position response");
        return ret;
    }

    // Position is in the last 4 bytes of data
    steps = *reinterpret_cast<int32_t*>(&response.data[2]);
    LogMessage("Current position: " + std::to_string(steps) + " steps");
    
    return DEVICE_OK;
}

int ThorlabsMCM3001::Home()
{
    MMThreadGuard guard(lock_);
    
    // Create home command (specific to MCM3001)
    CmdPacket6 cmd = {
        0x43,            // Home command
        0x04,            // Length
        static_cast<uint8_t>(currentAxis_ & 0xFF),  // Channel ID
        0x00,            // Param1
        0x00,            // Param2
        0x00             // Param3
    };

    LogMessage("Starting homing sequence");
    
    int ret = SendCommand(cmd);
    if (ret != DEVICE_OK)
    {
        LogMessage("Failed to send home command");
        return ret;
    }

    // Wait for homing to complete
    do {
        CmdPacket6 statusCmd = {
            CMD_REQUEST_STATUS,
            0x04,
            static_cast<uint8_t>(currentAxis_ & 0xFF),
            0x00,
            0x00,
            0x00
        };

        ret = SendCommand(statusCmd);
        if (ret != DEVICE_OK)
            return ret;

        StatusResponse response;
        ret = ReadResponse(reinterpret_cast<unsigned char*>(&response), sizeof(response));
        if (ret != DEVICE_OK)
            return ret;

        busy_ = (response.data[16] & 0x30) != 0;
        
        if (busy_)
            CDeviceUtils::SleepMs(100);  // Longer wait during homing
            
    } while (busy_);

    LogMessage("Homing completed");
    return DEVICE_OK;
}

int ThorlabsMCM3001::SetOrigin()
{
    MMThreadGuard guard(lock_);
    
    // Create set encoder command
    CmdPacket12 cmd = {
        CMD_SET_ENCODER, // Command byte
        0x04,           // Length
        0x06,           // Param1 (always 0x06)
        0x00,           // Param2
        0x00,           // Param3
        0x00,           // Param4
        currentAxis_,   // Channel ID
        0              // Set current position to 0
    };

    LogMessage("Setting current position as origin");
    
    int ret = SendCommand(cmd);
    if (ret != DEVICE_OK)
    {
        LogMessage("Failed to set origin");
        return ret;
    }

    return DEVICE_OK;
}

int ThorlabsMCM3001::GetLimits(double& lower, double& upper)
{
    // MCM3001 has fixed travel range of ±12.7mm for ZFM2020/ZFM2030
    lower = -12700.0;  // micrometers
    upper = 12700.0;   // micrometers
    return DEVICE_OK;
}

bool ThorlabsMCM3001::Busy()
{
    LogMessage("Busy check: " + std::string(busy_ ? "true" : "false"), true);
    return busy_;
}

void ThorlabsMCM3001::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(name, "ThorlabsMCM3001");
    LogMessage("GetName called", true);
}

int ThorlabsMCM3001::Shutdown()
{
    LogMessage("Shutting down device", true);
    initialized_ = false;
    return DEVICE_OK;
}

int ThorlabsMCM3001::SetPositionUm(double pos)
{
    LogMessage("SetPositionUm: " + std::to_string(pos) + " um", true);
    long steps = UmToSteps(pos);
    LogMessage("Converting to steps: " + std::to_string(steps), true);
    return SetPositionSteps(steps);
}

int ThorlabsMCM3001::GetPositionUm(double& pos)
{
    long steps;
    int ret = GetPositionSteps(steps);
    if (ret != DEVICE_OK)
    {
        LogMessage("GetPositionSteps failed", false);
        return ret;
    }
    pos = StepsToUm(steps);
    LogMessage("GetPositionUm: " + std::to_string(pos) + " um", true);
    return DEVICE_OK;
}

bool ThorlabsMCM3001::IsContinuousFocusDrive() const
{
    LogMessage("IsContinuousFocusDrive: false", true);
    return false;
}

int ThorlabsMCM3001::IsStageSequenceable(bool& isSequenceable) const
{
    isSequenceable = false;
    LogMessage("IsStageSequenceable: false", true);
    return DEVICE_OK;
}

int ThorlabsMCM3001::OnAxis(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        LogMessage("OnAxis BeforeGet: " + std::to_string(currentAxis_), true);
        pProp->Set((long)currentAxis_);
    }
    else if (eAct == MM::AfterSet)
    {
        long axis;
        pProp->Get(axis);
        LogMessage("OnAxis AfterSet requested axis: " + std::to_string(axis), true);
        if (axis < 0 || axis > 2)
        {
            LogMessage("Invalid axis specified", false);
            return ERR_INVALID_AXIS;
        }
        currentAxis_ = (uint16_t)axis;
        LogMessage("Axis set to: " + std::to_string(currentAxis_), true);
    }
    return DEVICE_OK;
}

int ThorlabsMCM3001::OnEncoderResolution(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        LogMessage("OnEncoderResolution BeforeGet: " + std::to_string(encoderResolutionUm_), true);
        pProp->Set(encoderResolutionUm_);
    }
    else if (eAct == MM::AfterSet)
    {
        double oldResolution = encoderResolutionUm_;
        pProp->Get(encoderResolutionUm_);
        LogMessage("Encoder resolution changed from " + std::to_string(oldResolution) + 
                  " to " + std::to_string(encoderResolutionUm_) + " um/count", true);
    }
    return DEVICE_OK;
}

int ThorlabsMCM3001::SendCommand(const CmdPacket12& cmd)
{
    MMThreadGuard guard(lock_);
    
    std::stringstream ss;
    ss << "Sending 12-byte command: 0x" << std::hex << (int)cmd.cmd 
       << " to axis " << std::dec << (int)cmd.channelId 
       << " with value " << cmd.value;
    LogMessage(ss.str(), true);
    
    MM::Device* pDevice = GetCoreCallback()->GetDevice(this, port_.c_str());
    if (pDevice == NULL)
    {
        LogMessage("Failed to get serial port device", false);
        return DEVICE_INVALID_PROPERTY_VALUE;
    }
    
    MM::Serial* pSerial = static_cast<MM::Serial*>(pDevice);
    unsigned char* buf = (unsigned char*)&cmd;
    int ret = pSerial->Write(buf, sizeof(CmdPacket12));
    
    if (ret != DEVICE_OK)
        LogMessage("Command send failed", false);
    else
        LogMessage("Command sent successfully", true);
        
    return ret;
}

int ThorlabsMCM3001::ReadResponse(unsigned char* response, unsigned length)
{
    MMThreadGuard guard(lock_);
    
    LogMessage("Attempting to read " + std::to_string(length) + " bytes", true);
    
    MM::Device* pDevice = GetCoreCallback()->GetDevice(this, port_.c_str());
    if (pDevice == NULL)
    {
        LogMessage("Failed to get serial port device", false);
        return DEVICE_INVALID_PROPERTY_VALUE;
    }
    
    MM::Serial* pSerial = static_cast<MM::Serial*>(pDevice);
    unsigned long bytesRead = 0;
    unsigned long totalRead = 0;
    unsigned long timeoutMs = 500;
    
    while (totalRead < length) {
        int ret = pSerial->Read(response + totalRead, length - totalRead, bytesRead);
        if (ret != DEVICE_OK)
        {
            LogMessage("Read failed with error: " + std::to_string(ret), false);
            return ret;
        }
        if (bytesRead == 0) {
            CDeviceUtils::SleepMs(2);
            timeoutMs -= 2;
            if (timeoutMs == 0)
            {
                LogMessage("Read timeout after 500ms", false);
                return ERR_COMMAND_FAILED;
            }
            continue;
        }
        totalRead += bytesRead;
        LogMessage("Read " + std::to_string(bytesRead) + " bytes", true);
    }
    
    LogMessage("Successfully read " + std::to_string(totalRead) + " bytes", true);
    return DEVICE_OK;
}

int ThorlabsMCM3001::ClearPort()
{
    MMThreadGuard guard(lock_);
    
    LogMessage("Clearing port " + port_, true);
    
    MM::Device* pDevice = GetCoreCallback()->GetDevice(this, port_.c_str());
    if (pDevice == NULL)
    {
        LogMessage("Failed to get serial port device", false);
        return DEVICE_INVALID_PROPERTY_VALUE;
    }
    
    MM::Serial* pSerial = static_cast<MM::Serial*>(pDevice);
    
    // Read any remaining bytes
    unsigned char buf[128];
    unsigned long read = 1;
    int ret = DEVICE_OK;
    unsigned long totalCleared = 0;
    
    while (read > 0 && ret == DEVICE_OK) {
        ret = pSerial->Read(buf, 128, read);
        totalCleared += read;
    }
    
    LogMessage("Cleared " + std::to_string(totalCleared) + " bytes from port", true);
    return DEVICE_OK;
}

ThorlabsMCM3001::~ThorlabsMCM3001()
{
    LogMessage("Destructor called", true);
    Shutdown();
}




