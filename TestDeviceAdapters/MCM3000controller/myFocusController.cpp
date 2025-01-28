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
    if (initialized_)
        return DEVICE_OK;

    // Set default error messages
    InitializeDefaultErrorMessages();
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

    // Port
    CPropertyAction* pAct = new CPropertyAction(this, &myFocusController::OnPort);
    CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);

    // Wait for port to be set
    if (port_ == "Undefined")
        return DEVICE_OK;

    // Clear serial port
    int ret = ClearPort();
    if (ret != DEVICE_OK)
        return ret;

    // Test communication by getting position
    long steps;
    ret = GetPositionSteps(steps);
    if (ret != DEVICE_OK)
        return ret;

    // Add step size property
    pAct = new CPropertyAction(this, &myFocusController::OnStepSizeUm);
    ret = CreateProperty("StepSizeUm", CDeviceUtils::ConvertToString(stepSizeUm_), MM::Float, false, pAct);
    if (ret != DEVICE_OK)
        return ret;

    // Set origin at startup
    ret = SetOrigin();
    if (ret != DEVICE_OK)
        return ret;

    initialized_ = true;
    home_ = true;  // Consider device homed after initialization

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

    // Send status request command
    unsigned char cmd[] = {0x80, 0x04, 0x00, 0x00, 0x00, 0x00};
    int ret = SendCommand(cmd, STATUS_LENGTH);
    if (ret != DEVICE_OK)
        return false;

    // Get response (6 bytes)
    unsigned char response[6];
    ret = GetResponse(response, 6);
    if (ret != DEVICE_OK)
        return false;

    // Check if moving (byte 5)
    return (response[5] & 0x01) != 0;
}

int myFocusController::GetPositionSteps(long& steps)
{
    if (!initialized_)
        return DEVICE_ERR;

    // Query Position command
    unsigned char cmd[] = {0x0A, 0x04, 0x00, 0x00, 0x00, 0x00};
    int ret = SendCommand(cmd, QUERY_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    // Get response (12 bytes total)
    unsigned char response[12];
    memset(response, 0, sizeof(response));  // Initialize buffer
    ret = GetResponse(response, 12);
    if (ret != DEVICE_OK)
        return ret;

    // Position is a 4-byte signed integer in little-endian format
    memcpy(&steps, response, 4);
    curSteps_ = steps;

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

int myFocusController::SetPositionUm(double pos)
{
    long steps = (long)(pos / stepSizeUm_);
    return SetPositionSteps(steps);
}

int myFocusController::SetPositionSteps(long steps)
{
    return MoveBlocking(steps, false);
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

    // Set encoder counter to 0
    unsigned char cmd[] = {0x09, 0x04, 0x06, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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

    // Stop command
    unsigned char cmd[] = {0x65, 0x04, 0x00, 0x01, 0x00, 0x00};
    return SendCommand(cmd, 6);
}

int myFocusController::SendCommand(const unsigned char* command, unsigned length)
{
    int ret = GetCoreCallback()->WriteToSerial(this, port_.c_str(), command, length);
    if (ret != DEVICE_OK)
        return ret;
    
    // Add small delay after sending command
    CDeviceUtils::SleepMs(1);
    
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
        }
        else
        {
            CDeviceUtils::SleepMs(2);  // Increased sleep time
        }
    }
    
    if (bytesRead != length)
    {
        return DEVICE_SERIAL_TIMEOUT;
    }
    
    return DEVICE_OK;
}

int myFocusController::ClearPort()
{
    if (port_ == "Undefined")
        return DEVICE_OK;

    unsigned char clear[100];
    unsigned long read = 100;
    MM::MMTime startTime = GetCurrentMMTime();
    
    do {
        read = 100;
        int ret = GetCoreCallback()->ReadFromSerial(this, port_.c_str(), clear, read, read);
        if (ret != DEVICE_OK && ret != DEVICE_SERIAL_TIMEOUT)  // Ignore timeout errors
            return ret;
        CDeviceUtils::SleepMs(1);  // Add small delay
    } while (read == 100 && (GetCurrentMMTime() - startTime).getMsec() < answerTimeoutMs_);
    
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

    // Format move command
    unsigned char cmd[SET_POS_LENGTH];
    cmd[0] = relative ? 0x48 : 0x53;  // 0x48 for relative, 0x53 for absolute
    cmd[1] = 0x04;
    cmd[2] = 0x06;
    cmd[3] = 0x00;
    cmd[4] = 0x00;
    cmd[5] = 0x00;
    
    // Convert steps to little-endian bytes
    memcpy(cmd + 6, &steps, 4);

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
