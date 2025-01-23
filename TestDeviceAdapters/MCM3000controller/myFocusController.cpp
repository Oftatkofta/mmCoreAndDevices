#include "myFocusController.h"
#include "DeviceUtils.h"
#include <cstdio>
#include <string>
#include <sstream>

const char* g_ControllerName = "MCM3000Controller";
const char* g_PropertyPort = "Port";

// Error codes
const int ERR_PORT_CHANGE_FORBIDDEN = 101;
const int ERR_UNRECOGNIZED_ANSWER = 102;
const int ERR_RESPONSE_TIMEOUT = 103;
const int ERR_HOME_REQUIRED = 104;
const int ERR_BUSY = 105;

// Command lengths
const int SET_POS_LENGTH = 12;
const int QUERY_POS_LENGTH = 6;
const int STATUS_LENGTH = 6;

myFocusController::CommandThread::CommandThread(myFocusController* stage) :
    stop_(false), 
    moving_(false), 
    relative_(false),
    stage_(stage), 
    pos_(0),
    errCode_(DEVICE_OK)
{
}

void myFocusController::CommandThread::StartMove(long pos, bool relative)
{
    Reset();
    pos_ = pos;
    relative_ = relative;
    activate();
}

int myFocusController::CommandThread::svc()
{
    moving_ = true;
    errCode_ = stage_->MoveBlocking(pos_, relative_);
    moving_ = false;
    return errCode_;
}

myFocusController::myFocusController() :
    initialized_(false),
    port_("Undefined"),
    stepSizeUm_(0.2116667), // um per count from documentation
    answerTimeoutMs_(1000.0),
    cmdThread_(nullptr),
    home_(false),
    curSteps_(0),
    lastMoveTime_(0.0)
{
    InitializeDefaultErrorMessages();
    SetErrorText(ERR_PORT_CHANGE_FORBIDDEN, "Port change is not allowed after device has been initialized.");
    SetErrorText(ERR_UNRECOGNIZED_ANSWER, "Invalid response from the device.");
    SetErrorText(ERR_RESPONSE_TIMEOUT, "Device response timeout.");
    SetErrorText(ERR_HOME_REQUIRED, "Stage must be homed before sending MOVE commands.");
    SetErrorText(ERR_BUSY, "Device is busy.");

    // Pre-initialization properties
    CreateProperty(MM::g_Keyword_Name, g_ControllerName, MM::String, true);
    CreateProperty(MM::g_Keyword_Description, "MCM3000 Focus Controller", MM::String, true);

    CPropertyAction* pAct = new CPropertyAction(this, &myFocusController::OnPort);
    CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);

    cmdThread_ = new CommandThread(this);
}

myFocusController::~myFocusController()
{
    Shutdown();
    delete cmdThread_;
}

void myFocusController::GetName(char* Name) const
{
    CDeviceUtils::CopyLimitedString(Name, g_ControllerName);
}

int myFocusController::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    // Clear serial port
    int ret = ClearPort();
    if (ret != DEVICE_OK)
        return ret;

    // Check if we can communicate with the device
    if (!GetMotorStatus())
        return ERR_UNRECOGNIZED_ANSWER;

    // Add step size property
    CPropertyAction* pAct = new CPropertyAction(this, &myFocusController::OnStepSizeUm);
    ret = CreateProperty("StepSizeUm", CDeviceUtils::ConvertToString(stepSizeUm_), MM::Float, false, pAct);
    if (ret != DEVICE_OK)
        return ret;

    // Set origin at startup
    ret = SetOrigin();
    if (ret != DEVICE_OK)
        return ret;

    home_ = true;
    initialized_ = true;
    return DEVICE_OK;
}

int myFocusController::Shutdown()
{
    if (cmdThread_ && cmdThread_->IsMoving())
    {
        cmdThread_->Stop();
        cmdThread_->wait();
    }

    initialized_ = false;
    return DEVICE_OK;
}

bool myFocusController::Busy()
{
    return cmdThread_->IsMoving() || GetMotorStatus();
}

int myFocusController::SetPositionUm(double pos)
{
    long steps = (long)(pos / stepSizeUm_);
    return SetPositionSteps(steps);
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
    if (!home_)
        return ERR_HOME_REQUIRED;

    if (Busy())
        return ERR_BUSY;

    cmdThread_->StartMove(steps);
    lastMoveTime_ = GetCurrentMMTime();
    return DEVICE_OK;
}

int myFocusController::GetPositionSteps(long& steps)
{
    unsigned char cmd[] = {0x0A, 0x04, 0x00, 0x00, 0x00, 0x00};
    int ret = SendCommand(cmd, QUERY_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    unsigned char response[12];
    ret = GetResponse(response, 12);
    if (ret != DEVICE_OK)
        return ret;

    // Parse position from response (little endian format)
    steps = ((long)response[10] << 24) |
            ((long)response[9] << 16) |
            ((long)response[8] << 8) |
            (long)response[7];

    curSteps_ = steps;
    return DEVICE_OK;
}

int myFocusController::SetOrigin()
{
    // Set encoder counter to 0
    unsigned char cmd[] = {0x09, 0x04, 0x06, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int ret = SendCommand(cmd, 12);
    if (ret != DEVICE_OK)
        return ret;

    curSteps_ = 0;
    return DEVICE_OK;
}

int myFocusController::Stop()
{
    // Stop command with abrupt stop mode
    unsigned char cmd[] = {0x65, 0x04, 0x00, 0x01, 0x00, 0x00};
    int ret = SendCommand(cmd, 6);
    if (ret != DEVICE_OK)
        return ret;

    // Wait for the device to actually stop
    MM::MMTime startTime = GetCurrentMMTime();
    while (GetMotorStatus() && (GetCurrentMMTime() - startTime).getMsec() < answerTimeoutMs_)
    {
        CDeviceUtils::SleepMs(5);
    }

    // Also stop any ongoing thread operations
    if (cmdThread_->IsMoving())
    {
        cmdThread_->Stop();
        cmdThread_->wait();
    }

    return DEVICE_OK;
}

int myFocusController::Home()
{
    SetOrigin();
    home_ = true;
    return DEVICE_OK;
}

int myFocusController::GetLimits(double& lower, double& upper)
{
    // Using reasonable limits based on device capabilities
    lower = -25000.0 * stepSizeUm_; // -25mm
    upper = 25000.0 * stepSizeUm_;  // +25mm
    return DEVICE_OK;
}

int myFocusController::SendCommand(const unsigned char* command, unsigned length)
{
    int ret = GetCoreCallback()->WriteToSerial(this, port_.c_str(), command, length);
    if (ret != DEVICE_OK)
        return ret;
    
    return DEVICE_OK;
}

int myFocusController::GetResponse(unsigned char* response, unsigned length)
{
    MM::MMTime startTime = GetCurrentMMTime();
    unsigned long bytesRead = 0;
    unsigned char* pos = response;
    
    while ((bytesRead < length) && ((GetCurrentMMTime() - startTime).getMsec() < answerTimeoutMs_))
    {
        unsigned long readNow = 0;
        int ret = GetCoreCallback()->ReadFromSerial(this, port_.c_str(), pos, length - bytesRead, readNow);
        if (ret != DEVICE_OK)
            return ret;
        bytesRead += readNow;
        pos += readNow;
    }
    
    if (bytesRead != length)
        return ERR_RESPONSE_TIMEOUT;
    
    return DEVICE_OK;
}

bool myFocusController::GetMotorStatus()
{
    unsigned char cmd[] = {0x80, 0x04, 0x00, 0x00, 0x00, 0x00};
    if (SendCommand(cmd, STATUS_LENGTH) != DEVICE_OK)
        return true;

    unsigned char response[34];
    if (GetResponse(response, 34) != DEVICE_OK)
        return true;

    // Check if motor is moving (bits 4-5 of byte 16)
    return (response[16] & 0x30) != 0;
}

int myFocusController::ClearPort()
{
    unsigned char clear[100];
    unsigned long read = 100;
    int ret;
    while (read == 100)
    {
        ret = GetCoreCallback()->ReadFromSerial(this, port_.c_str(), clear, 100, read);
        if (ret != DEVICE_OK)
            return ret;
    }
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
            return ERR_PORT_CHANGE_FORBIDDEN;
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
            return ERR_PORT_CHANGE_FORBIDDEN;
        pProp->Get(stepSizeUm_);
    }
    return DEVICE_OK;
}

int myFocusController::MoveBlocking(long steps, bool relative)
{
    if (!home_)
        return ERR_HOME_REQUIRED;

    unsigned char cmd[12];
    if (relative) {
        cmd[0] = 0x48; // Relative move command
    } else {
        cmd[0] = 0x53; // Absolute move command
    }
    
    cmd[1] = 0x04;
    cmd[2] = 0x06;
    cmd[3] = 0x00;
    cmd[4] = 0x00;
    cmd[5] = 0x00;
    cmd[6] = (unsigned char)(steps & 0xFF);
    cmd[7] = (unsigned char)((steps >> 8) & 0xFF);
    cmd[8] = (unsigned char)((steps >> 16) & 0xFF);
    cmd[9] = (unsigned char)((steps >> 24) & 0xFF);
    
    int ret = SendCommand(cmd, SET_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    // Wait for move to complete
    MM::MMTime startTime = GetCurrentMMTime();
    bool busy;
    do {
        busy = GetMotorStatus();
        if ((GetCurrentMMTime() - startTime).getMsec() > answerTimeoutMs_)
            return ERR_RESPONSE_TIMEOUT;
        
        CDeviceUtils::SleepMs(5);
    } while (busy);

    return DEVICE_OK;
}

// Required Stage API methods
int myFocusController::SetAdapterOriginUm(double d)
{
    return DEVICE_OK;
}

int myFocusController::Move(double /*velocity*/)
{
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
