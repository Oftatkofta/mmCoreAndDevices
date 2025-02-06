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
#include <iomanip>

// Constants for axis/channel IDs
const unsigned char AXIS_ID_BYTE = 0x01;  // 8-bit axis ID
const unsigned short AXIS_ID_WORD = 0x0001;  // 16-bit axis ID

// Define static constants
const double myFocusController::POSITION_LIMIT_UM = 12700.0;  // ±12.7mm for ZFM2020/2030
const double myFocusController::DEFAULT_STEP_SIZE_UM = 0.2116667;  // From Python code

// Module interface
MODULE_API void InitializeModuleData()
{
    RegisterDevice(myFocusController::DeviceName(), MM::StageDevice, "MCM3000 Focus Controller");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    if (deviceName == 0)
        return 0;

    if (strcmp(deviceName, myFocusController::DeviceName()) == 0)
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
    stepSizeUm_(DEFAULT_STEP_SIZE_UM),  // From Python code
    answerTimeoutMs_(500),
    curSteps_(0),
    positionValid_(false),
    home_(false),
    lastCommand_(0)
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
    CreateProperty(MM::g_Keyword_Name, DeviceName(), MM::String, true);
    
    std::string description = Description();
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
    CDeviceUtils::CopyLimitedString(name, g_DeviceName);
}

int myFocusController::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    // Check if we have a valid port
    if (port_ == "Undefined")
    {
        LogMessage("Port not specified", false);
        return DEVICE_ERR;
    }

    // Set up properties
    int ret = CreateProperty("StepSize", 
                         std::to_string(DEFAULT_STEP_SIZE_UM).c_str(), 
                         MM::Float, 
                         true);
    if (ret != DEVICE_OK)
        return ret;

    // Set travel range
    ret = CreateProperty(MM::g_Keyword_Position, "0", MM::Float, false);
    SetPropertyLimits(MM::g_Keyword_Position, -POSITION_LIMIT_UM, POSITION_LIMIT_UM);
    if (ret != DEVICE_OK)
        return ret;

    // Clear any leftover data
    ret = ClearPort();
    if (ret != DEVICE_OK)
        return ret;

    // Test communication with position query
    unsigned char cmd[] = {CMD_QUERY_POS, 0x04, AXIS_ID_BYTE, 0x00, 0x00, 0x00};
    ret = SendCommand(cmd, QUERY_POS_LENGTH);
    if (ret != DEVICE_OK)
    {
        LogMessage("Failed to send position query", false);
        return ret;
    }

    // Get 12-byte position response
    unsigned char response[12];
    ret = GetResponse(response, 12);
    if (ret != DEVICE_OK)
    {
        LogMessage("Failed to get position response", false);
        return ret;
    }

    // Verify response format (first byte should be command + 1)
    if (response[0] != (CMD_QUERY_POS + 1))
    {
        LogMessage("Invalid position response", false);
        return ERR_UNRECOGNIZED_ANSWER;
    }

    // Extract initial position
    curSteps_ = (long)response[8] | ((long)response[9] << 8) | 
                ((long)response[10] << 16) | ((long)response[11] << 24);
    positionValid_ = true;

    initialized_ = true;
    return DEVICE_OK;
}

int myFocusController::ClearPort()
{
    int ret = GetCoreCallback()->PurgeSerial(this, port_.c_str());
    if (ret != DEVICE_OK)
    {
        GetCoreCallback()->LogMessage(this, "Failed to purge serial port", true);
        return ret;
    }
    return DEVICE_OK;
}

int myFocusController::Shutdown()
{
    if (initialized_)
    {
        LogMessage("Shutting down...", true);
        initialized_ = false;
    }
    return DEVICE_OK;
}

bool myFocusController::Busy()
{
    if (!initialized_)
        return false;

    // Get current position and compare with target
    long currentPos;
    int ret = GetPositionSteps(currentPos);
    if (ret != DEVICE_OK)
    {
        LogMessage("Failed to get position in Busy check", true);
        return false;
    }

    // If position is changing, we're busy
    if (currentPos != curSteps_)
    {
        positionValid_ = false;
        return true;
    }

    return false;
}

int myFocusController::GetPositionSteps(long& steps)
{
    // Query current position
    unsigned char cmd[] = {CMD_QUERY_POS, 0x04, AXIS_ID_BYTE, 0x00, 0x00, 0x00};
    int ret = SendCommand(cmd, QUERY_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    // Get response
    unsigned char response[12];  // Position response is 12 bytes
    ret = GetResponse(response, 12);
    if (ret != DEVICE_OK)
        return ret;

    // Log response
    std::ostringstream os;
    os << "Position response:";
    for (int i = 0; i < 12; i++)
        os << " " << std::hex << std::setw(2) << std::setfill('0') << (int)response[i];
    LogMessage(os.str().c_str(), true);

    // Extract position value (bytes 8-11, little endian)
    long pos = 0;
    pos |= response[8];
    pos |= (response[9] << 8);
    pos |= (response[10] << 16);
    pos |= (response[11] << 24);

    // Update cache and return
    curSteps_ = pos;
    steps = pos;

    os.str("");
    os << "Current position: " << std::dec << pos << " steps (0x" 
       << std::hex << std::setw(8) << std::setfill('0') << pos << ")";
    LogMessage(os.str().c_str(), true);

    return DEVICE_OK;
}

int myFocusController::GetPositionUm(double& pos)
{
    // Always get fresh position from device
    long steps;
    int ret = GetPositionSteps(steps);
    if (ret != DEVICE_OK)
        return ret;

    // Convert to microns
    pos = steps * stepSizeUm_;

    // Cache the position
    curSteps_ = steps;
    positionValid_ = true;

    return DEVICE_OK;
}

int myFocusController::SetPositionSteps(long steps)
{
    if (!initialized_)
        return DEVICE_ERR;

    if (Busy())
        return ERR_BUSY;

    // Log move details
    std::ostringstream os;
    os << "Move command: current=" << curSteps_ << " target=" << steps 
       << " delta=" << (steps - curSteps_) << " steps";
    LogMessage(os.str().c_str(), true);

    // Send move command
    unsigned char cmd[] = {CMD_GOTO_POS, 0x04, 0x06, 0x00, 0x00, 0x00,
                          (unsigned char)AXIS_ID_WORD,
                          (unsigned char)(AXIS_ID_WORD >> 8),
                          (unsigned char)(steps & 0xFF),
                          (unsigned char)((steps >> 8) & 0xFF),
                          (unsigned char)((steps >> 16) & 0xFF),
                          (unsigned char)((steps >> 24) & 0xFF)};

    int ret = SendCommand(cmd, SET_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    // Don't wait for response to move command - start monitoring position immediately
    
    // Wait for move completion with timeout
    MM::MMTime startTime = GetCurrentMMTime();
    MM::MMTime timeout = MM::MMTime::fromMs(10000);
    bool moveComplete = false;
    long lastPosition = curSteps_;
    MM::MMTime lastMoveTime = startTime;
    
    while (!moveComplete && ((GetCurrentMMTime() - startTime) <= timeout))
    {
        // Get current position
        long currentPos;
        ret = GetPositionSteps(currentPos);
        if (ret != DEVICE_OK)
            return ret;

        // Check if position is changing
        if (currentPos != lastPosition)
        {
            lastPosition = currentPos;
            lastMoveTime = GetCurrentMMTime();
        }
        
        // Check if we're stuck (no position change for 2 seconds)
        if ((GetCurrentMMTime() - lastMoveTime) > MM::MMTime::fromMs(2000))
        {
            LogMessage("Move appears stuck - no position change for 2 seconds", true);
            return ERR_RESPONSE_TIMEOUT;
        }

        // Check if within tolerance of target
        if (abs(currentPos - steps) <= ENCODER_COUNT_TOLERANCE)
        {
            moveComplete = true;
            curSteps_ = currentPos;
            positionValid_ = true;
            break;
        }

        CDeviceUtils::SleepMs(MOTOR_STATUS_POLL_MS);
    }

    if (!moveComplete)
    {
        LogMessage("Move did not complete within timeout", true);
        return ERR_RESPONSE_TIMEOUT;
    }

    return DEVICE_OK;
}

int myFocusController::SetPositionUm(double pos)
{
    if (!initialized_)
        return DEVICE_ERR;

    // Check position limits
    if (pos < -POSITION_LIMIT_UM || pos > POSITION_LIMIT_UM)
    {
        std::ostringstream os;
        os << "Position " << pos << " um exceeds limit of ±" << POSITION_LIMIT_UM << " um";
        LogMessage(os.str().c_str(), false);
        return ERR_STEPS_OUT_OF_RANGE;
    }

    // Convert um to steps with proper rounding
    long steps = (long)(pos / stepSizeUm_ + (pos >= 0 ? 0.5 : -0.5));
    
    // Log requested and actual positions
    std::ostringstream os;
    double actualPos = steps * stepSizeUm_;
    os << "Requested: " << pos << " um, Actual: " << actualPos << " um";
    LogMessage(os.str().c_str(), true);

    return SetPositionSteps(steps);
}

int myFocusController::SetRelativePositionSteps(long steps)
{
    // Get current position
    long curPos;
    int ret = GetPositionSteps(curPos);
    if (ret != DEVICE_OK)
        return ret;

    // Calculate target position
    long targetPos = curPos + steps;

    std::ostringstream moveLog;  // Unique name for move logging
    moveLog << "Relative move: current=" << curPos << " steps=" << steps << " target=" << targetPos;
    LogMessage(moveLog.str().c_str(), true);

    // Use SetPositionSteps to move
    return SetPositionSteps(targetPos);
}

int myFocusController::SetRelativePositionUm(double d)
{
    // Get current position in case it was changed externally (e.g. joystick)
    double curPos;
    int ret = GetPositionUm(curPos);
    if (ret != DEVICE_OK)
        return ret;

    // Calculate target position
    double targetPos = curPos + d;

    // Log the move
    std::ostringstream os;
    os << "Relative move: current=" << curPos << " offset=" << d << " target=" << targetPos;
    LogMessage(os.str().c_str(), true);

    // Use absolute positioning
    return SetPositionUm(targetPos);
}

int myFocusController::GetLimits(double& lower, double& upper)
{
    // Return the stage limits from constants
    lower = -POSITION_LIMIT_UM;
    upper = POSITION_LIMIT_UM;
    return DEVICE_OK;
}

int myFocusController::SetOrigin()
{
    if (!initialized_)
        return DEVICE_ERR;

    // Set encoder counter to 0 with channel 1
    unsigned char cmd[] = {CMD_SET_ENCODER, 0x04, 0x06, 0x00, 0x00, 0x00, 
                           (unsigned char)(AXIS_ID_WORD & 0xFF),        // Channel ID low byte
                           (unsigned char)((AXIS_ID_WORD >> 8) & 0xFF), // Channel ID high byte
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

    // Send stop command
    unsigned char cmd[] = {CMD_STOP, 0x04, AXIS_ID_BYTE, 0x00, 0x00, 0x00};
    int ret = SendCommand(cmd, STATUS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    // Update position after stop
    long steps;
    ret = GetPositionSteps(steps);
    if (ret != DEVICE_OK)
        return ret;

    curSteps_ = steps;
    return DEVICE_OK;
}

int myFocusController::SendCommand(const unsigned char* command, unsigned length)
{
    // Don't check initialized_ here - needed for initialization itself

    // Clear any leftover bytes
    int ret = ClearPort();
    if (ret != DEVICE_OK)
        return ret;

    // Send command
    ret = GetCoreCallback()->WriteToSerial(this, port_.c_str(), command, length);
    if (ret != DEVICE_OK)
        return ret;

    lastCommand_ = command[0];
    return DEVICE_OK;
}

int myFocusController::GetResponse(unsigned char* response, unsigned expectedLength)
{
    if (!response)
        return DEVICE_ERR;

    unsigned char buf[256];
    unsigned long bytesRead = 0;
    unsigned long totalRead = 0;
    
    const MM::MMTime startTime = GetCurrentMMTime();
    const MM::MMTime timeout = MM::MMTime::fromMs(answerTimeoutMs_);
    
    // Read response using MM serial interface
    while (totalRead < expectedLength)
    {
        if ((GetCurrentMMTime() - startTime) > timeout)
        {
            GetCoreCallback()->LogMessage(this, g_Msg_SERIAL_TIMEOUT, true);
            return DEVICE_SERIAL_TIMEOUT;
        }

        int ret = GetCoreCallback()->ReadFromSerial(this, port_.c_str(), 
                                                  buf + totalRead,
                                                  expectedLength - totalRead, 
                                                  bytesRead);
        if (ret != DEVICE_OK)
        {
            GetCoreCallback()->LogMessage(this, g_Msg_SERIAL_COMMAND_FAILED, true);
            return ret;
        }

        if (bytesRead > 0)
        {
            totalRead += bytesRead;
        }
        else
        {
            CDeviceUtils::SleepMs(2);
        }
    }

    // Log response in MM format
    std::ostringstream os;
    os << "Read <- (hex)";
    for (unsigned i = 0; i < totalRead; i++)
        os << " " << std::hex << std::setw(2) << std::setfill('0') << (int)buf[i];
    GetCoreCallback()->LogMessage(this, os.str().c_str(), true);

    // Validate response
    if (totalRead < 3)
    {
        GetCoreCallback()->LogMessage(this, g_Msg_SERIAL_INVALID_RESPONSE, true);
        return DEVICE_SERIAL_INVALID_RESPONSE;
    }

    // Check response code
    if ((buf[0] & 0x7F) != ((lastCommand_ + 1) & 0x7F))
    {
        GetCoreCallback()->LogMessage(this, g_Msg_SERIAL_INVALID_RESPONSE, true);
        return DEVICE_SERIAL_INVALID_RESPONSE;
    }

    // Copy valid packet
    memcpy(response, buf, expectedLength);

    return DEVICE_OK;
}

int myFocusController::OnPort(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(port_.c_str());
        LogMessage("OnPort BeforeGet: " + port_, true);
    }
    else if (eAct == MM::AfterSet)
    {
        if (initialized_)
        {
            LogMessage(g_Msg_PORT_CHANGE_FORBIDDEN, false);
            return ERR_PORT_CHANGE_FORBIDDEN;
        }
        std::string oldPort = port_;
        pProp->Get(port_);
        LogMessage("OnPort AfterSet: changed from " + oldPort + " to " + port_, true);
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
    if (Busy())
        return ERR_BUSY;

    long target = steps;
    if (relative)
    {
        if (!positionValid_)
        {
            LogMessage("Cannot do relative move without valid position", true);
            return ERR_STAGE_NOT_ZEROED;
        }
        target = curSteps_ + steps;
    }

    return SetPositionSteps(target);  // Use SetPositionSteps which now handles completion
}

int myFocusController::Home()
{
    if (Busy())
        return ERR_BUSY;

    // Set encoder to 0 at current position
    unsigned char cmd[] = {CMD_SET_ENCODER, 0x04, 0x06, 0x00, 0x00, 0x00,
                          (unsigned char)AXIS_ID_WORD,        // Channel ID (LSB)
                          (unsigned char)(AXIS_ID_WORD >> 8), // Channel ID (MSB)
                          0x00, 0x00, 0x00, 0x00};  // Position 0

    int ret = SendCommand(cmd, SET_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    // Wait for response
    unsigned char response[20];
    ret = GetResponse(response, 20);
    if (ret != DEVICE_OK)
        return ret;

    // Update cached position
    curSteps_ = 0;
    home_ = true;
    positionValid_ = true;

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
