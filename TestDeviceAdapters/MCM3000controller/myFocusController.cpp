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

// Constants for axis/channel IDs
const unsigned char AXIS_ID_BYTE = 0x01;  // 8-bit axis ID
const unsigned short AXIS_ID_WORD = 0x0001;  // 16-bit axis ID

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
    home_(false),
    curSteps_(INVALID_POSITION),  // Initialize to invalid
    positionValid_(false),
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

    // Clear port
    LogMessage("MCM3000 initialization started...", true);
    int ret = GetCoreCallback()->PurgeSerial(this, port_.c_str());
    if (ret != DEVICE_OK)
        return ret;

    // Test communication with simple status query
    unsigned char cmd[] = {CMD_QUERY_STATUS, 0x04, AXIS_ID_BYTE, 0x00, 0x00, 0x00};
    ret = SendCommand(cmd, STATUS_LENGTH);
    if (ret != DEVICE_OK)
    {
        LogMessage(g_Msg_SERIAL_COMMAND_FAILED, true);
        return DEVICE_SERIAL_COMMAND_FAILED;
    }

    // Get response (20 bytes for status)
    unsigned char response[20];
    ret = GetResponse(response, 20);
    if (ret != DEVICE_OK)
    {
        LogMessage(g_Msg_SERIAL_INVALID_RESPONSE, true);
        return DEVICE_SERIAL_INVALID_RESPONSE;
    }

    // Log response in both hex and decimal for debugging
    std::ostringstream os;
    os << "Received " << 20 << " bytes:";
    for (int i = 0; i < 20; i++)
        os << " " << (int)response[i];
    LogMessage(os.str().c_str(), true);

    os.str("");
    os << "Received " << 20 << " bytes (hex):";
    for (int i = 0; i < 20; i++)
        os << " " << std::hex << (int)response[i];
    LogMessage(os.str().c_str(), true);

    // Check if device is ready from status response
    if ((response[16] & 0x30) != 0)
    {
        LogMessage("Device not ready in status response", true);
        return DEVICE_SERIAL_INVALID_RESPONSE;
    }

    // Initialize state using current position from status response
    // Position is at offset 8, little-endian 32-bit signed integer
    long pos = 0;
    pos |= response[8];
    pos |= (response[9] << 8);
    pos |= (response[10] << 16);
    pos |= (response[11] << 24);
    curSteps_ = pos;

    positionValid_ = true;
    home_ = true;
    initialized_ = true;

    LogMessage("MCM3000 initialization completed successfully", true);
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
    // Query Position uses 1 byte ID
    unsigned char cmd[] = {CMD_QUERY_POS, 0x04, AXIS_ID_BYTE, 0x00, 0x00, 0x00};
    int ret = SendCommand(cmd, QUERY_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    // Get response (12 bytes total)
    unsigned char response[12];
    memset(response, 0, sizeof(response));
    ret = GetResponse(response, 12);
    if (ret != DEVICE_OK)
    {
        positionValid_ = false;
        return ret;
    }

    // Log the full response for debugging
    std::ostringstream msg;
    msg << "Position response: ";
    for (int i = 0; i < 12; i++)
        msg << std::hex << (int)response[i] << " ";
    LogMessage(msg.str().c_str(), true);

    // Position is in bytes 8-11 (after channel ID)
    int32_t position;
    memcpy(&position, &response[8], 4);
    steps = position;
    
    // Cache the position
    curSteps_ = steps;
    positionValid_ = true;

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

    // Go to Position uses 2 byte ID
    unsigned char cmd[SET_POS_LENGTH];
    cmd[0] = CMD_GOTO_POS;
    cmd[1] = 0x04;
    cmd[2] = 0x06;
    cmd[3] = 0x00;
    cmd[4] = 0x00;
    cmd[5] = 0x00;
    cmd[6] = (unsigned char)(AXIS_ID_WORD & 0xFF);        // Low byte
    cmd[7] = (unsigned char)((AXIS_ID_WORD >> 8) & 0xFF); // High byte
    memcpy(cmd + 8, &steps, 4);

    std::ostringstream msg;
    msg << "Moving to position: " << steps << " steps";
    LogMessage(msg.str().c_str(), true);

    // Invalidate position cache before move
    positionValid_ = false;

    int ret = SendCommand(cmd, SET_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    lastMoveTime_ = GetCurrentMMTime();
    return DEVICE_OK;
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

    // Use SetPositionSteps to move
    return SetPositionSteps(targetPos);
}

int myFocusController::SetPositionUm(double pos)
{
    // Convert microns to steps
    long steps = (long)(pos / stepSizeUm_);
    return SetPositionSteps(steps);
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
    // Get core callback
    MM::Core* core = GetCoreCallback();
    if (core == NULL)
        return DEVICE_ERR;

    // Write command to serial port - cast command to const unsigned char* to match expected type
    int ret = core->WriteToSerial(this, port_.c_str(), command, length);
    if (ret != DEVICE_OK)
    {
        std::ostringstream os;
        os << "Serial write error: " << ret;
        LogMessage(os.str().c_str(), true);
        return ret;
    }
    return DEVICE_OK;
}

int myFocusController::GetResponse(unsigned char* response, unsigned expectedLength)
{
    if (!response)
        return DEVICE_ERR;

    unsigned char buf[256];
    unsigned long read = 0;
    int ret = GetCoreCallback()->ReadFromSerial(this, port_.c_str(), buf, expectedLength, read);
    
    if (ret != DEVICE_OK)
        return ret;

    if (read < 3) // Minimum packet size
    {
        LogMessage("Response too short", true);
        return ERR_INVALID_PACKET_LENGTH;
    }

    // Get actual packet length from response
    unsigned packetLength = buf[2];  // Length is in 3rd byte
    if (packetLength > read)
    {
        LogMessage("Incomplete packet received", true);
        return ERR_INVALID_PACKET_LENGTH;
    }

    // Copy only the actual packet
    memcpy(response, buf, packetLength);

    std::ostringstream os;
    os << "Received " << packetLength << " bytes:";
    for (unsigned i = 0; i < packetLength; i++)
        os << " " << (int)response[i];
    LogMessage(os.str().c_str(), true);

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

    // Format move command with axis 1
    unsigned char cmd[SET_POS_LENGTH];
    cmd[0] = CMD_GOTO_POS;
    cmd[1] = 0x04;
    cmd[2] = 0x06;
    cmd[3] = 0x00;
    cmd[4] = 0x00;
    cmd[5] = 0x00;
    cmd[6] = (unsigned char)(AXIS_ID_WORD & 0xFF);        // Channel ID low byte
    cmd[7] = (unsigned char)((AXIS_ID_WORD >> 8) & 0xFF); // Channel ID high byte
    
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
    unsigned char cmd[] = {CMD_SET_ENCODER, 0x04, 0x06, 0x00, 0x00, 0x00, 
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
