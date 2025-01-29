#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include "myFocusController.h"
#include "ModuleInterface.h"
#include "DeviceUtils.h"
#include <cstdio>
#include <string>
#include <sstream>

const char* myFocusController::DeviceName = "MCM3000";
const char* myFocusController::Description = "MCM3000 Focus Controller";

// Module interface
MODULE_API void InitializeModuleData()
{
    RegisterDevice(myFocusController::DeviceName, MM::StageDevice, "MCM3000 Focus Controller");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    if (deviceName == 0)
        return 0;

    if (strcmp(deviceName, myFocusController::DeviceName) == 0)
    {
        return new myFocusController();
    }
    return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
    delete pDevice;
}

myFocusController::myFocusController() :
    initialized_(false),
    port_("Undefined"),
    stepSizeUm_(0.2116667), // um per count from documentation
    answerTimeoutMs_(1000.0),
    home_(false),
    curSteps_(0),
    lastMoveTime_(0.0)
{
    InitializeDefaultErrorMessages();

    // Add custom error messages
    SetErrorText(ERR_PORT_CHANGE_FORBIDDEN, "Port change is not allowed after device has been initialized.");
    SetErrorText(ERR_UNRECOGNIZED_ANSWER, "Unrecognized answer received from the device.");
    SetErrorText(ERR_HOME_REQUIRED, "Home device before moving.");
    SetErrorText(ERR_INVALID_PACKET_LENGTH, "Invalid packet length.");
    SetErrorText(ERR_RESPONSE_TIMEOUT, "Device response timeout.");
    SetErrorText(ERR_BUSY, "Device is busy.");
    SetErrorText(ERR_STEPS_OUT_OF_RANGE, "Position out of range.");
    SetErrorText(ERR_STAGE_NOT_ZEROED, "Stage must be zeroed before use.");

    // Create pre-initialization properties
    CreateProperty(MM::g_Keyword_Name, DeviceName, MM::String, true);
    
    std::string description = Description;
    description += "\n\nSerial port settings:\n";
    description += "  Baud Rate: 460800\n";
    description += "  Data Bits: 8\n";
    description += "  Stop Bits: 1\n";
    description += "  Parity: None\n";
    description += "  Flow Control: None";
    CreateProperty(MM::g_Keyword_Description, description.c_str(), MM::String, true);

    // Create pre-initialization property for port
    CPropertyAction* pAct = new CPropertyAction(this, &myFocusController::OnPort);
    CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);
}

myFocusController::~myFocusController()
{
    Shutdown();
}

void myFocusController::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(name, DeviceName);
}

int myFocusController::Initialize()
{
    // Log initialization start
    LogMessage("MCM3000 initialization started...");

    if (initialized_)
        return DEVICE_OK;

    // Check if port is set
    if (port_ == "Undefined") {
        LogMessage("Port not set");
        return DEVICE_ERR;
    }

    // Clear port before starting
    int ret = ClearPort();
    if (ret != DEVICE_OK) {
        LogMessage("Failed to clear port");
        return ret;
    }

    // Add step size property before attempting communication
    CPropertyAction* pAct = new CPropertyAction(this, &myFocusController::OnStepSizeUm);
    ret = CreateProperty("StepSizeUm", CDeviceUtils::ConvertToString(stepSizeUm_), MM::Float, false, pAct);
    if (ret != DEVICE_OK) {
        LogMessage("Failed to create StepSizeUm property");
        return ret;
    }

    // Test communication with simple status query using channel 1
    LogMessage("Testing communication...");
    unsigned char cmd[] = {0x80, 0x04, 0x01, 0x00, 0x00, 0x00};  // Changed channel to 0x01
    ret = SendCommand(cmd, STATUS_LENGTH);
    if (ret != DEVICE_OK) {
        LogMessage("Failed to send status command");
        return ret;
    }

    // Get response (6 bytes)
    unsigned char response[6];
    ret = GetResponse(response, 6);
    if (ret != DEVICE_OK) {
        LogMessage("Failed to get status response");
        return ret;
    }

    // Set initialized flag before setting origin
    initialized_ = true;

    // Set origin with channel 1
    ret = SetOrigin();
    if (ret != DEVICE_OK) {
        initialized_ = false;
        LogMessage("Failed to set origin");
        return ret;
    }

    home_ = true;
    LogMessage("MCM3000 initialization completed successfully");
    return DEVICE_OK;
}

int myFocusController::Shutdown()
{
    initialized_ = false;
    return DEVICE_OK;
}

bool myFocusController::Busy()
{
    if (!initialized_)
        return false;

    // Send status request command with channel 1
    unsigned char cmd[] = {0x80, 0x04, 0x01, 0x00, 0x00, 0x00};
    int ret = SendCommand(cmd, STATUS_LENGTH);
    if (ret != DEVICE_OK)
    {
        LogMessage("Failed to send status command", true);
        return false;
    }

    // Get 6-byte header
    unsigned char header[6];
    ret = GetResponse(header, 6);
    if (ret != DEVICE_OK)
    {
        LogMessage("Failed to get status header", true);
        return false;
    }

    // Log the header
    std::ostringstream headerMsg;
    headerMsg << "Status header: ";
    for (int i = 0; i < 6; i++)
        headerMsg << std::hex << (int)header[i] << " ";
    LogMessage(headerMsg.str().c_str(), true);

    // Get 28-byte data packet
    unsigned char data[28];
    ret = GetResponse(data, 28);
    if (ret != DEVICE_OK)
    {
        LogMessage("Failed to get status data", true);
        return false;
    }

    // Log the data packet
    std::ostringstream dataMsg;
    dataMsg << "Status data: ";
    for (int i = 0; i < 28; i++)
        dataMsg << std::hex << (int)data[i] << " ";
    LogMessage(dataMsg.str().c_str(), true);

    // Check byte 16 for busy status
    bool isMoving = (data[16] & 0x30) != 0;
    LogMessage(isMoving ? "Device reports busy" : "Device reports not busy", true);
    return isMoving;
}

int myFocusController::GetPositionSteps(long& steps)
{
    // Query Position command with channel 1
    unsigned char cmd[] = {0x0A, 0x04, 0x01, 0x00, 0x00, 0x00};
    int ret = SendCommand(cmd, QUERY_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    // Get response (12 bytes total)
    unsigned char response[12];
    memset(response, 0, sizeof(response));
    ret = GetResponse(response, 12);
    if (ret != DEVICE_OK)
        return ret;

    // Log the full response for debugging
    std::ostringstream msg;
    msg << "Position response: ";
    for (int i = 0; i < 12; i++)
        msg << std::hex << (int)response[i] << " ";
    LogMessage(msg.str().c_str(), true);

    // Position data is in the first 4 bytes (little-endian)
    int32_t position;
    memcpy(&position, response, 4);
    steps = position;
    curSteps_ = steps;

    std::ostringstream posMsg;
    posMsg << "Current position: " << steps << " steps (0x" << std::hex << steps << ")";
    LogMessage(posMsg.str().c_str(), true);
    return DEVICE_OK;
}

int myFocusController::GetPositionUm(double& pos)
{
    long steps;
    int ret = GetPositionSteps(steps);
    if (ret != DEVICE_OK)
        return ret;
    
    pos = steps * stepSizeUm_;
    return DEVICE_OK;
}

int myFocusController::SetPositionSteps(long steps)
{
    if (!initialized_)
        return DEVICE_ERR;

    if (Busy())
        return ERR_BUSY;

    // Format move command with channel 1
    unsigned char cmd[SET_POS_LENGTH];
    cmd[0] = 0x53;  // Go to absolute position command
    cmd[1] = 0x04;
    cmd[2] = 0x06;
    cmd[3] = 0x00;
    cmd[4] = 0x00;
    cmd[5] = 0x00;
    cmd[6] = 0x01;  // Channel 1
    cmd[7] = 0x00;  // Channel high byte
    
    // Convert steps to little-endian bytes
    memcpy(cmd + 8, &steps, 4);

    std::ostringstream msg;
    msg << "Moving to position: " << steps << " steps";
    LogMessage(msg.str().c_str(), true);

    int ret = SendCommand(cmd, SET_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    lastMoveTime_ = GetCurrentMMTime();
    return DEVICE_OK;
}

int myFocusController::SetRelativePositionSteps(long steps)
{
    // Get current position
    long currentPos;
    int ret = GetPositionSteps(currentPos);
    if (ret != DEVICE_OK)
        return ret;

    // Calculate target position
    long targetPos = currentPos + steps;

    // Move to new absolute position
    return MoveBlocking(targetPos, false);
}

int myFocusController::SetPositionUm(double pos)
{
    // Convert microns to steps
    long steps = (long)(pos / stepSizeUm_);
    return SetPositionSteps(steps);
}

int myFocusController::SetRelativePositionUm(double d)
{
    // Convert microns to steps
    long steps = (long)(d / stepSizeUm_);
    return SetRelativePositionSteps(steps);
}

int myFocusController::GetLimits(double& lower, double& upper)
{
    // MCM3000 has a travel range of ±12.5mm
    lower = -12500.0;  // μm
    upper = 12500.0;   // μm
    return DEVICE_OK;
}

int myFocusController::SetOrigin()
{
    if (!initialized_)
        return DEVICE_ERR;

    // Set encoder counter to 0 with channel 1
    unsigned char cmd[] = {0x09, 0x04, 0x06, 0x00, 0x00, 0x00, 
                          0x01, 0x00,  // Channel 1
                          0x00, 0x00, 0x00, 0x00};  // Position 0
    int ret = SendCommand(cmd, SET_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    curSteps_ = 0;
    return DEVICE_OK;
}

int myFocusController::Stop()
{
    if (!initialized_)
        return DEVICE_ERR;

    // Stop command with channel 1
    unsigned char cmd[] = {0x65, 0x04, 0x01, 0x01, 0x00, 0x00};  // Changed channel to 0x01
    return SendCommand(cmd, 6);
}

int myFocusController::SendCommand(const unsigned char* command, unsigned length)
{
    if (!command)
        return DEVICE_ERR;

    std::stringstream msg;
    msg << "Sending command: ";
    for (unsigned i = 0; i < length; i++)
        msg << std::hex << (int)command[i] << " ";
    LogMessage(msg.str().c_str(), true);

    int ret = GetCoreCallback()->WriteToSerial(this, port_.c_str(), command, length);
    if (ret != DEVICE_OK)
        return ret;

    // Add small delay after sending command
    CDeviceUtils::SleepMs(10);
    return DEVICE_OK;
}

int myFocusController::GetResponse(unsigned char* response, unsigned length)
{
    if (!response)
        return DEVICE_ERR;

    MM::MMTime startTime = GetCurrentMMTime();
    unsigned long bytesRead = 0;
    
    while ((bytesRead < length) && ((GetCurrentMMTime() - startTime).getMsec() < answerTimeoutMs_))
    {
        unsigned long readNow = 0;
        int ret = GetCoreCallback()->ReadFromSerial(this, port_.c_str(), response + bytesRead, length - bytesRead, readNow);
        if (ret != DEVICE_OK)
            return ret;
        
        if (readNow > 0)
        {
            bytesRead += readNow;
            
            // Log partial response
            std::stringstream msg;
            msg << "Received " << readNow << " bytes: ";
            for (unsigned long i = 0; i < readNow; i++)
                msg << std::hex << (int)response[bytesRead - readNow + i] << " ";
            LogMessage(msg.str().c_str(), true);
        }
        else
        {
            CDeviceUtils::SleepMs(5);
        }
    }
    
    if (bytesRead != length)
    {
        LogMessage("Response timeout", true);
        return DEVICE_SERIAL_TIMEOUT;
    }
    
    return DEVICE_OK;
}

int myFocusController::ClearPort()
{
    LogMessage("Clearing serial port...");
    
    unsigned char clear[100];
    unsigned long read = 100;
    int ret;
    
    MM::MMTime startTime = GetCurrentMMTime();
    do {
        read = 100;
        ret = GetCoreCallback()->ReadFromSerial(this, port_.c_str(), clear, read, read);
        if (ret != DEVICE_OK && ret != DEVICE_SERIAL_TIMEOUT)
        {
            LogMessage("Error clearing port");
            return ret;
        }
        CDeviceUtils::SleepMs(5);
    } while (read == 100 && (GetCurrentMMTime() - startTime).getMsec() < answerTimeoutMs_);
    
    LogMessage("Port cleared");
    return DEVICE_OK;
}

int myFocusController::OnPort(MM::PropertyBase* pProp, MM::ActionType eAct)
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
            return DEVICE_ERR;
        }
        pProp->Get(port_);
    }
    return DEVICE_OK;
}

int myFocusController::OnStepSizeUm(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(stepSizeUm_);
    }
    else if (eAct == MM::AfterSet)
    {
        if (initialized_)
            return DEVICE_ERR;
        pProp->Get(stepSizeUm_);
    }
    return DEVICE_OK;
}

int myFocusController::MoveBlocking(long steps, bool relative)
{
    if (!initialized_)
        return DEVICE_ERR;

    if (Busy())
        return ERR_BUSY;

    // Format move command with channel 1
    unsigned char cmd[SET_POS_LENGTH];
    cmd[0] = 0x53;  // Go to absolute position command
    cmd[1] = 0x04;
    cmd[2] = 0x06;
    cmd[3] = 0x00;
    cmd[4] = 0x00;
    cmd[5] = 0x00;
    cmd[6] = 0x01;  // Channel 1
    cmd[7] = 0x00;  // Channel high byte
    
    // If relative move, convert to absolute position
    if (relative) {
        long currentPos;
        int ret = GetPositionSteps(currentPos);
        if (ret != DEVICE_OK)
            return ret;
        steps += currentPos;
    }

    // Convert steps to little-endian bytes
    memcpy(cmd + 8, &steps, 4);

    LogMessage(std::string("Moving to position: ") + std::to_string(steps) + " steps");

    int ret = SendCommand(cmd, SET_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    lastMoveTime_ = GetCurrentMMTime();
    return DEVICE_OK;
}

int myFocusController::Home()
{
    if (!initialized_)
        return DEVICE_ERR;

    // Set encoder counter to 0
    unsigned char cmd[] = {0x09, 0x04, 0x06, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int ret = SendCommand(cmd, SET_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    curSteps_ = 0;
    home_ = true;
    return DEVICE_OK;
}

int myFocusController::SetAdapterOriginUm(double)
{
    return DEVICE_OK;
}

int myFocusController::Move(double /*velocity*/)
{
    // MCM3000 doesn't support continuous motion
    return DEVICE_UNSUPPORTED_COMMAND;
}

int myFocusController::GetFocusDirection(MM::FocusDirection& direction)
{
    direction = MM::FocusDirectionUnknown;
    return DEVICE_OK;
}

int myFocusController::IsStageSequenceable(bool& isSequenceable) const
{
    isSequenceable = false;
    return DEVICE_OK;
}

int myFocusController::IsStageLinearSequenceable(bool& isSequenceable) const 
{
    isSequenceable = false;
    return DEVICE_OK;
}

bool myFocusController::IsContinuousFocusDrive() const
{
    return false;
}
