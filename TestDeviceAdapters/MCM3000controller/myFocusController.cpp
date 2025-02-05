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
    stepSizeUm_(0.2116667), // From documentation: 0.2116667 um per step
    answerTimeoutMs_(500),
    curSteps_(0),
    positionValid_(false),
    home_(false),
    lastCommand_(0)  // Initialize error counter
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

    GetCoreCallback()->LogMessage(this, "MCM3000: Initializing...", true);
    
    // Use standard error messages from MMDeviceConstants.h
    int ret = GetCoreCallback()->PurgeSerial(this, port_.c_str());
    if (ret != DEVICE_OK)
        return ret;

    // Test communication
    unsigned char cmd[] = {CMD_QUERY_STATUS, 0x04, AXIS_ID_BYTE, 0x00, 0x00, 0x00};
    ret = SendCommand(cmd, STATUS_LENGTH);
    if (ret != DEVICE_OK)
    {
        GetCoreCallback()->LogMessage(this, g_Msg_SERIAL_COMMAND_FAILED, false);
        return ret;
    }

    // Get response
    unsigned char response[20];
    ret = GetResponse(response, 20);
    if (ret != DEVICE_OK)
        return ret;

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
    initialized_ = false;
    return DEVICE_OK;
}

bool myFocusController::Busy()
{
    if (!initialized_)
        return false;

    // Query Status uses 1 byte ID
    unsigned char cmd[] = {CMD_QUERY_STATUS, 0x04, AXIS_ID_BYTE, 0x00, 0x00, 0x00};
    int ret = SendCommand(cmd, STATUS_LENGTH);
    if (ret != DEVICE_OK)
    {
        LogMessage("Failed to send status command", true);
        return false;
    }

    // Buffer for response (20 bytes total)
    unsigned char response[20];
    memset(response, 0, sizeof(response));
    unsigned long totalRead = 0;
    MM::MMTime startTime = GetCurrentMMTime();

    // Keep reading until we get complete response or timeout
    while (totalRead < 20 && (GetCurrentMMTime() - startTime).getMsec() < 100)
    {
        unsigned long readNow = 0;
        ret = GetCoreCallback()->ReadFromSerial(this, port_.c_str(), 
                                              response + totalRead, 
                                              20 - totalRead, 
                                              readNow);
        if (ret != DEVICE_OK && ret != DEVICE_SERIAL_TIMEOUT)
        {
            LogMessage("Serial read error", true);
            return false;
        }

        if (readNow > 0)
        {
            std::ostringstream msg;
            msg << "Received " << readNow << " bytes: ";
            for (unsigned long i = 0; i < readNow; i++)
                msg << std::hex << (int)response[totalRead + i] << " ";
            LogMessage(msg.str().c_str(), true);
            
            totalRead += readNow;
        }
        else
        {
            CDeviceUtils::SleepMs(2);
        }
    }

    if (totalRead < 20)
    {
        LogMessage("Incomplete status response", true);
        return false;
    }

    // Check status bits in byte 17 (index 16)
    bool isMoving = (response[16] & 0x30) != 0;
    if (isMoving)
        positionValid_ = false;

    return isMoving;
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
    long steps;
    int ret = GetPositionSteps(steps);
    if (ret != DEVICE_OK)
        return ret;

    // Convert steps to microns using documented conversion factor
    pos = steps * stepSizeUm_;
    return DEVICE_OK;
}

int myFocusController::SetPositionSteps(long steps)
{
    if (!initialized_)
        return DEVICE_ERR;

    if (Busy())
        return ERR_BUSY;

    // Don't send move command if already at position
    if (steps == curSteps_ && positionValid_)
    {
        return DEVICE_OK;
    }

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
    {
        LogMessage("Failed to send move command", true);
        return ret;
    }

    // Wait for move completion using MM time
    const MM::MMTime startTime = GetCurrentMMTime();
    const MM::MMTime timeout = MM::MMTime::fromMs(answerTimeoutMs_);
    bool moveComplete = false;
    
    while (!moveComplete && ((GetCurrentMMTime() - startTime) <= timeout))
    {
        // Query status
        unsigned char statCmd[] = {CMD_QUERY_STATUS, 0x04, AXIS_ID_BYTE, 0x00, 0x00, 0x00};
        ret = SendCommand(statCmd, STATUS_LENGTH);
        if (ret != DEVICE_OK)
            return ret;

        unsigned char response[20];
        ret = GetResponse(response, 20);
        if (ret != DEVICE_OK)
        {
            std::ostringstream os;
            os << "Status response failed with error: " << ret;
            LogMessage(os.str().c_str(), true);
            return ret;
        }

        // Check if move complete (not busy)
        if ((response[16] & 0x30) == 0)
        {
            moveComplete = true;
            
            // Verify final position
            unsigned char posCmd[] = {CMD_QUERY_POS, 0x04, AXIS_ID_BYTE, 0x00, 0x00, 0x00};
            ret = SendCommand(posCmd, QUERY_POS_LENGTH);
            if (ret != DEVICE_OK)
            {
                LogMessage("Failed to query final position", true);
                return ret;
            }

            unsigned char posResponse[12];
            ret = GetResponse(posResponse, 12);
            if (ret != DEVICE_OK)
            {
                LogMessage("Failed to get position response", true);
                return ret;
            }

            // Extract position from response (bytes 8-11, little endian)
            long actualPos = 0;
            actualPos |= posResponse[8];
            actualPos |= (posResponse[9] << 8);
            actualPos |= (posResponse[10] << 16);
            actualPos |= (posResponse[11] << 24);

            if (actualPos != steps)
            {
                std::ostringstream os;
                os << "Move completed but position mismatch. Requested: " << steps 
                   << " Actual: " << actualPos;
                LogMessage(os.str().c_str(), true);
                return ERR_INVALID_PACKET_LENGTH;  // Or a more specific error code
            }

            curSteps_ = actualPos;
            break;
        }

        CDeviceUtils::SleepMs(10); // Use MM's sleep utility
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
    // Convert microns to steps using documented conversion factor
    long steps = (long)(pos / stepSizeUm_ + 0.5); // Round to nearest step
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

    // Skip move if target equals current position
    if (targetPos == curPos)
    {
        return DEVICE_OK;
    }

    // Use SetPositionSteps to move
    return SetPositionSteps(targetPos);
}

int myFocusController::SetRelativePositionUm(double d)
{
    // Convert um to steps
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

    // Stop command with channel 1
    unsigned char cmd[] = {CMD_STOP, 0x04, 0x01, AXIS_ID_BYTE, 0x00, 0x00};  // Changed channel to 0x01
    return SendCommand(cmd, 6);
}

int myFocusController::SendCommand(const unsigned char* command, unsigned length)
{
    // Log command in MM format
    std::ostringstream os;
    os << "Write -> (hex)";
    for (unsigned i = 0; i < length; i++)
        os << " " << std::hex << std::setw(2) << std::setfill('0') << (int)command[i];
    GetCoreCallback()->LogMessage(this, os.str().c_str(), true);

    // Store command for response validation
    lastCommand_ = command[0];

    // Clear any pending data
    int ret = ClearPort();
    if (ret != DEVICE_OK)
        return ret;

    // Send command using MM serial interface
    ret = GetCoreCallback()->WriteToSerial(this, port_.c_str(), command, length);
    if (ret != DEVICE_OK)
    {
        GetCoreCallback()->LogMessage(this, g_Msg_SERIAL_COMMAND_FAILED, true);
        return ret;
    }

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
    }
    else if (eAct == MM::AfterSet)
    {
        if (initialized_)
        {
            GetCoreCallback()->LogMessage(this, g_Msg_PORT_CHANGE_FORBIDDEN, false);
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

    // Send move command
    unsigned char cmd[] = {CMD_GOTO_POS, 0x04, 0x06, 0x00, 0x00, 0x00,
                          (unsigned char)AXIS_ID_WORD,        // Channel ID (LSB)
                          (unsigned char)(AXIS_ID_WORD >> 8), // Channel ID (MSB)
                          (unsigned char)(target & 0xFF),
                          (unsigned char)((target >> 8) & 0xFF),
                          (unsigned char)((target >> 16) & 0xFF),
                          (unsigned char)((target >> 24) & 0xFF)};

    int ret = SendCommand(cmd, SET_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    // Wait for move completion using MM time
    const MM::MMTime startTime = GetCurrentMMTime();
    const MM::MMTime timeout = MM::MMTime::fromMs(answerTimeoutMs_);
    bool moveComplete = false;
    
    while (!moveComplete && ((GetCurrentMMTime() - startTime) <= timeout))
    {
        // Query status
        unsigned char statCmd[] = {CMD_QUERY_STATUS, 0x04, AXIS_ID_BYTE, 0x00, 0x00, 0x00};
        ret = SendCommand(statCmd, STATUS_LENGTH);
        if (ret != DEVICE_OK)
            return ret;

        unsigned char response[20];
        ret = GetResponse(response, 20);
        if (ret != DEVICE_OK)
            return ret;

        // Check if move complete (not busy)
        if ((response[16] & 0x30) == 0)
        {
            moveComplete = true;
            curSteps_ = target;
            break;
        }

        CDeviceUtils::SleepMs(10); // Use MM's sleep utility
    }

    if (!moveComplete)
    {
        LogMessage("Move did not complete within timeout", true);
        return ERR_RESPONSE_TIMEOUT;
    }

    return DEVICE_OK;
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
