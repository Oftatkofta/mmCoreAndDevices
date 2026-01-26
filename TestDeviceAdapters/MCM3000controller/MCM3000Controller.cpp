// FILE:          MCM3000Controller.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   Device adapter implementation for the Thorlabs MCM3000/3001 Focus Controller.
//                This file implements the serial communication protocol and device control logic.
//                
// NOTES:         Serial Communication Settings:
//                - Baud Rate: 460800
//                - Data Bits: 8
//                - Stop Bits: 1
//                - Parity: None
//                - Flow Control: None
//
// AUTHOR:        Jens Eriksson, jens.eriksson@imbim.uu.se
// COPYRIGHT:     Jens Eriksson, Uppsala University, 2025
// LICENSE:       This file is distributed under the BSD license.
//                License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include "MCM3000Controller.h"
#include "ModuleInterface.h"
#include "DeviceUtils.h"
#include <cstdio>
#include <string>
#include <sstream>
#include <iomanip>
#include <map>

/**
 * Module interface implementation - this function registers our device with Micro-Manager.
 * It tells Micro-Manager what our device is called, what type it is (a stage device),
 * and provides a description including supported features.
 */
MODULE_API void InitializeModuleData()
{
    RegisterDevice(ThorlabsMCM3000::DeviceName(), 
                  MM::StageDevice, 
                  "MCM3000 Focus Controller (Axis ID: 0-2, Default step size: 0.2116667 µm)");
}

/**
 * Factory function that creates an instance of our device.
 * When Micro-Manager needs a new instance of this device, it calls this function.
 * Returns NULL if the requested device name doesn't match our device.
 */
MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    if (deviceName == 0)
        return 0;

    if (strcmp(deviceName, ThorlabsMCM3000::DeviceName()) == 0)
    {
        return new ThorlabsMCM3000();
    }
    return 0;
}

/**
 * Cleanup function that deletes a device instance.
 * Called by Micro-Manager when it needs to destroy our device.
 */
MODULE_API void DeleteDevice(MM::Device* pDevice)
{
    delete pDevice;
}

/**
 * This lookup table maps user-friendly stage descriptions to their exact step sizes.
 * Each entry shows the stage model(s) and their corresponding step size in microns.
 * This allows users to select their stage from a dropdown without needing to know
 * the exact step size value.
 */
const std::map<std::string, double> ThorlabsMCM3000::STEP_SIZE_MAP = {
    {"0.0390625 (LNR50S, PHYS24M, MTM-FN1, MTME-FN1, DRV014)", 0.0390625},  // LNR50S, PHYS24M, MTM-FN1, MTME-FN1, DRV014
    {"0.2116667 (ZFM2020/2030, PLS-X/Y)",                      0.2116667},  // ZFM2020/2030, PLS-X/Y 
    {"0.001 (AScope Z)",                                        0.001},      // AScope Z
    {"0.5 (MMP-2XY, PMP-2XY, Bergamo XY)",                     0.5},        // MMP-2XY, PMP-2XY, Bergamo XY
    {"0.1 (Bergamo Z)",                                         0.1}         // Bergamo Z
};

/**
 * Constructor
 * Initializes member variables and sets up device properties
 */
ThorlabsMCM3000::ThorlabsMCM3000() :
    initialized_(false),
    port_("Undefined"),
    axisID_(0),  // Default to axis 0
    stepSizeUm_(DEFAULT_STEP_SIZE_UM),
    answerTimeoutMs_(2000.0),
    curSteps_(0),
    positionValid_(false),
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
    SetErrorText(ERR_CANNOT_CHANGE_PROPERTY, "Cannot change this property after device initialization.");

    // Create pre-initialization properties
    CreateProperty(MM::g_Keyword_Name, DeviceName(), MM::String, true);
    
    // Add detailed serial port settings to description
    std::string description = Description();
    description += "\n\nSerial port settings:\n";
    description += "  Baud Rate: 460800\n";
    description += "  Data Bits: 8\n";
    description += "  Stop Bits: 1\n";
    description += "  Parity: None\n";
    description += "  Flow Control: None";
    CreateProperty(MM::g_Keyword_Description, description.c_str(), MM::String, true);

    // Create Port property
    CPropertyAction* pAct = new CPropertyAction(this, &ThorlabsMCM3000::OnPort);
    CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);

    // Create Axis ID property (0-2)
    pAct = new CPropertyAction(this, &ThorlabsMCM3000::OnAxisID);
    CreateProperty("AxisID", "0", MM::Integer, false, pAct, true);
    SetPropertyLimits("AxisID", 0, 2);
    AddAllowedValue("AxisID", "0");
    AddAllowedValue("AxisID", "1");
    AddAllowedValue("AxisID", "2");

    // Create Step Size property with lookup table values
    pAct = new CPropertyAction(this, &ThorlabsMCM3000::OnStepSize);
    CreateProperty("StepSize", "0.2116667", MM::String, false, pAct, true);
    
    for (const auto& pair : STEP_SIZE_MAP) {
        AddAllowedValue("StepSize", pair.first.c_str());
    }
}

/**
 * Destructor
 * Ensures device is properly shut down
 */
ThorlabsMCM3000::~ThorlabsMCM3000()
{
    Shutdown();
}

/**
 * Returns the device name
 * Required by MMDevice API
 */
void ThorlabsMCM3000::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(name, g_DeviceName);
}

/**
 * Initializes the hardware
 * Required by MMDevice API
 * 
 * Initialization sequence:
 * 1. Verifies valid port is selected
 * 2. Sets up position limits
 * 3. Clears communication buffer
 * 4. Tests communication with position query
 * 
 * @return DEVICE_OK on success, error code on failure
 */
int ThorlabsMCM3000::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    // Check if we have a valid port
    if (port_ == "Undefined")
    {
        LogMessage("Port not specified", false);
        return DEVICE_ERR;
    }

    // Set travel range with action handler for position control
    CPropertyAction* pAct = new CPropertyAction(this, &ThorlabsMCM3000::OnPosition);
    int ret = CreateProperty(MM::g_Keyword_Position, "0", MM::Float, false, pAct);
    SetPropertyLimits(MM::g_Keyword_Position, -POSITION_LIMIT_UM, POSITION_LIMIT_UM);
    if (ret != DEVICE_OK)
        return ret;

    // Add "Set Origin" property - sets current position as Z=0
    pAct = new CPropertyAction(this, &ThorlabsMCM3000::OnSetOrigin);
    ret = CreateProperty("Set Origin", "No", MM::String, false, pAct);
    if (ret != DEVICE_OK)
        return ret;
    AddAllowedValue("Set Origin", "No");
    AddAllowedValue("Set Origin", "Yes");

    // Clear any leftover data
    ret = ClearPort();
    if (ret != DEVICE_OK)
        return ret;

    // Test communication with position query
    unsigned char cmd[] = {CMD_QUERY_POS, 0x04, axisID_, 0x00, 0x00, 0x00};
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

int ThorlabsMCM3000::ClearPort()
{
    int ret = GetCoreCallback()->PurgeSerial(this, port_.c_str());
    if (ret != DEVICE_OK)
    {
        GetCoreCallback()->LogMessage(this, "Failed to purge serial port", true);
        return ret;
    }
    return DEVICE_OK;
}

int ThorlabsMCM3000::Shutdown()
{
    if (initialized_)
    {
        LogMessage("Shutting down...", true);
        initialized_ = false;
    }
    return DEVICE_OK;
}

bool ThorlabsMCM3000::Busy()
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

int ThorlabsMCM3000::GetPositionSteps(long& steps)
{
    // Query current position
    unsigned char cmd[] = {CMD_QUERY_POS, 0x04, axisID_, 0x00, 0x00, 0x00};
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

// Gets the current position in microns
// Converts from steps to microns using the configured step size
// Returns DEVICE_OK on success, error code on failure
int ThorlabsMCM3000::GetPositionUm(double& pos)
{
    long steps;
    int ret = GetPositionSteps(steps);
    if (ret != DEVICE_OK)
        return ret;
    pos = steps * stepSizeUm_;  // Uses stepSizeUm_ (double)
    return DEVICE_OK;
}

// Moves the stage to the specified position in steps
// Returns DEVICE_OK on success, error code on failure
int ThorlabsMCM3000::SetPositionSteps(long steps)
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
                          (unsigned char)(axisID_ & 0xFF),        // LSB
                          0x00,                                   // MSB (always 0 for axis 0-2)
                          (unsigned char)(steps & 0xFF),          // Position LSB
                          (unsigned char)((steps >> 8) & 0xFF),   // Position byte 2
                          (unsigned char)((steps >> 16) & 0xFF),  // Position byte 3
                          (unsigned char)((steps >> 24) & 0xFF)}; // Position MSB

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

int ThorlabsMCM3000::SetPositionUm(double pos)
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

    // Convert um to steps with proper rounding (works for negative values too)
    double stepsDouble = pos / stepSizeUm_;
    long steps = (stepsDouble >= 0) ? (long)(stepsDouble + 0.5) : (long)(stepsDouble - 0.5);
    
    // Log requested and actual positions
    std::ostringstream os;
    double actualPos = steps * stepSizeUm_;
    os << "Requested: " << pos << " um, Actual: " << actualPos << " um";
    LogMessage(os.str().c_str(), true);

    return SetPositionSteps(steps);
}

// Moves the stage by a relative amount in steps
// Returns DEVICE_OK on success, error code on failure
int ThorlabsMCM3000::SetRelativePositionSteps(long steps)
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

// Moves the stage by a relative amount in microns
// Returns DEVICE_OK on success, error code on failure
int ThorlabsMCM3000::SetRelativePositionUm(double d)
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

// Gets the stage travel limits in microns
// Returns DEVICE_OK on success, error code on failure
int ThorlabsMCM3000::GetLimits(double& lower, double& upper)
{
    // Return the stage limits from constants
    lower = -POSITION_LIMIT_UM;
    upper = POSITION_LIMIT_UM;
    return DEVICE_OK;
}

// Sets the current position as the origin (zero)
// Updates the encoder count to 0 at the current position
// Returns DEVICE_OK on success, error code on failure
int ThorlabsMCM3000::SetOrigin()
{
    if (!initialized_)
        return DEVICE_ERR;

    // Set encoder to 0 at current position
    unsigned char cmd[] = {CMD_SET_ENCODER, 0x04, 0x06, 0x00, 0x00, 0x00,
                          (unsigned char)(axisID_ & 0xFF),  // LSB
                          0x00,                             // MSB (always 0 for axis 0-2)
                          0x00, 0x00, 0x00, 0x00};         // Position 0

    int ret = SendCommand(cmd, SET_POS_LENGTH);
    if (ret != DEVICE_OK)
        return ret;

    curSteps_ = 0;
    return DEVICE_OK;
}

// Immediately stops any ongoing motion
// Updates position after stopping
// Returns DEVICE_OK on success, error code on failure
int ThorlabsMCM3000::Stop()
{
    if (!initialized_)
        return DEVICE_ERR;

    // Send stop command: 65 04 [Chan] [Mode] 00 00
    // Stop mode 0x01 = abrupt stop (immediate)
    unsigned char cmd[] = {CMD_STOP, 0x04, axisID_, STOP_MODE_ABRUPT, 0x00, 0x00};
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

int ThorlabsMCM3000::SendCommand(const unsigned char* command, unsigned length)
{
    // Clear any leftover bytes
    int ret = ClearPort();
    if (ret != DEVICE_OK)
        return ret;

    // Store command for response validation
    lastCommand_ = command[0];

    // Send command
    ret = GetCoreCallback()->WriteToSerial(this, port_.c_str(), command, length);
    if (ret != DEVICE_OK)
        return ret;

    return DEVICE_OK;
}

int ThorlabsMCM3000::GetResponse(unsigned char* response, unsigned length)
{
    if (!response)
        return DEVICE_ERR;

    unsigned char buf[256];
    unsigned long bytesRead = 0;
    unsigned long totalRead = 0;
    
    const MM::MMTime startTime = GetCurrentMMTime();
    const MM::MMTime timeout = MM::MMTime::fromMs(answerTimeoutMs_);
    
    // Read response using MM serial interface
    while (totalRead < length)
    {
        if ((GetCurrentMMTime() - startTime) > timeout)
        {
            GetCoreCallback()->LogMessage(this, g_Msg_SERIAL_TIMEOUT, true);
            return DEVICE_SERIAL_TIMEOUT;
        }

        int ret = GetCoreCallback()->ReadFromSerial(this, port_.c_str(), 
                                                  buf + totalRead,
                                                  length - totalRead, 
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
    memcpy(response, buf, length);

    return DEVICE_OK;
}

int ThorlabsMCM3000::OnPort(MM::PropertyBase* pProp, MM::ActionType eAct)
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

// Property action handler for position control
// Allows setting Z position from µM UI
// Returns DEVICE_OK on success, error code on failure
int ThorlabsMCM3000::OnPosition(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        double pos;
        int ret = GetPositionUm(pos);
        if (ret != DEVICE_OK)
            return ret;
        pProp->Set(pos);
    }
    else if (eAct == MM::AfterSet)
    {
        double pos;
        pProp->Get(pos);
        return SetPositionUm(pos);
    }
    return DEVICE_OK;
}

// Property action handler for setting origin (Z=0)
// When set to "Yes", zeros the stage at current position
// Returns DEVICE_OK on success, error code on failure
int ThorlabsMCM3000::OnSetOrigin(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::AfterSet)
    {
        std::string val;
        pProp->Get(val);
        if (val == "Yes")
        {
            int ret = SetOrigin();
            // Reset to "No" after setting origin
            pProp->Set("No");
            if (ret != DEVICE_OK)
                return ret;
            LogMessage("Origin set - current position is now Z=0", false);
        }
    }
    return DEVICE_OK;
}

// Property action handler for the step size selection
// Maps UI-friendly descriptions to exact step size values
// Returns DEVICE_OK on success, error code on failure
int ThorlabsMCM3000::OnStepSize(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        // Find string for current value
        for (const auto& pair : STEP_SIZE_MAP) {
            if (pair.second == stepSizeUm_) {
                pProp->Set(pair.first.c_str());
                break;
            }
        }
    }
    else if (eAct == MM::AfterSet)
    {
        if (initialized_)
            return ERR_CANNOT_CHANGE_PROPERTY;

        std::string stepStr;
        pProp->Get(stepStr);

        auto it = STEP_SIZE_MAP.find(stepStr);
        if (it != STEP_SIZE_MAP.end()) {
            stepSizeUm_ = it->second;  // This is where the exact value is set
            return DEVICE_OK;
        }
        return ERR_INVALID_VALUE;
    }
    return DEVICE_OK;
}

// Property action handler for the axis ID selection
// Validates axis ID is in range 0-2
// Returns DEVICE_OK on success, error code on failure
int ThorlabsMCM3000::OnAxisID(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set((long)axisID_);
    }
    else if (eAct == MM::AfterSet)
    {
        if (initialized_)
        {
            LogMessage("Cannot change axis ID after initialization", false);
            return ERR_CANNOT_CHANGE_PROPERTY;
        }
        long id;
        pProp->Get(id);
        if (id < 0 || id > 2)  // Validate range
        {
            LogMessage("Axis ID must be between 0 and 2", false);
            return ERR_INVALID_VALUE;
        }
        axisID_ = (unsigned char)id;
    }
    return DEVICE_OK;
}

int ThorlabsMCM3000::MoveBlocking(long steps, bool relative)
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

int ThorlabsMCM3000::Home()
{
    if (Busy())
        return ERR_BUSY;

    // MCM3000 doesn't have a hardware home command, so we just set current position as origin
    // This is the same as SetOrigin()
    return SetOrigin();
}

int ThorlabsMCM3000::SetAdapterOriginUm(double)
{
    return DEVICE_OK;
}

int ThorlabsMCM3000::Move(double /*velocity*/)
{
    // MCM3000 doesn't support continuous motion
    return DEVICE_UNSUPPORTED_COMMAND;
}

int ThorlabsMCM3000::GetFocusDirection(MM::FocusDirection& direction)
{
    direction = MM::FocusDirectionUnknown;
    return DEVICE_OK;
}

int ThorlabsMCM3000::IsStageSequenceable(bool& isSequenceable) const
{
    isSequenceable = false;
    return DEVICE_OK;
}

int ThorlabsMCM3000::IsStageLinearSequenceable(bool& isSequenceable) const 
{
    isSequenceable = false;
    return DEVICE_OK;
}

bool ThorlabsMCM3000::IsContinuousFocusDrive() const
{
    return false;
}
