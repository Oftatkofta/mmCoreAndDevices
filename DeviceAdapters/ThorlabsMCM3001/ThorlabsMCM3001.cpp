#include "ThorlabsMCM3001.h"
#include "ModuleInterface.h"
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
    LogMessage("Sending 6-byte command: " + std::to_string(cmd.cmd));
    
    MM::Device* pDevice = GetCoreCallback()->GetDevice(this, port_.c_str());
    if (pDevice == NULL) {
        LogMessage("Invalid serial port for command");
        return DEVICE_INVALID_PROPERTY_VALUE;
    }
    
    MM::Serial* pSerial = static_cast<MM::Serial*>(pDevice);
    unsigned char* buf = (unsigned char*)&cmd;
    int ret = pSerial->Write(buf, sizeof(CmdPacket6));
    if (ret != DEVICE_OK) {
        LogMessage("Failed to write command");
        return ret;
    }
    
    return DEVICE_OK;
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




