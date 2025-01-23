#include "myFocusController.h"
#include <cstdio>
#include <string>
#include <sstream>

const char* g_ControllerName = "MCM3000Controller";
const char* g_PropertyPort = "Port";

myFocusController::myFocusController() :
    initialized_(false),
    conversionFactor_(0.2116667) // um per count
{
    InitializeDefaultErrorMessages();

    // Pre-initialization properties
    CreateProperty(MM::g_Keyword_Name, g_ControllerName, MM::String, true);
    CreateProperty(MM::g_Keyword_Description, "MCM3000 Focus Controller", MM::String, true);

    CPropertyAction* pAct = new CPropertyAction(this, &myFocusController::OnPort);
    CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);
}

myFocusController::~myFocusController()
{
    Shutdown();
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

    initialized_ = true;
    return DEVICE_OK;
}

int myFocusController::Shutdown()
{
    if (initialized_)
    {
        initialized_ = false;
    }
    return DEVICE_OK;
}

bool myFocusController::Busy()
{
    return GetMotorStatus();
}

int myFocusController::SetPositionSteps(long steps)
{
    unsigned char cmd[] = {0x53, 0x04, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
                          (unsigned char)(steps & 0xFF),
                          (unsigned char)((steps >> 8) & 0xFF),
                          (unsigned char)((steps >> 16) & 0xFF),
                          (unsigned char)((steps >> 24) & 0xFF)};
    
    return SendCommand(cmd, SET_POS_LENGTH);
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

    steps = ((long)response[10] << 24) |
            ((long)response[9] << 16) |
            ((long)response[8] << 8) |
            (long)response[7];

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
    
    while ((bytesRead < length) && ((GetCurrentMMTime() - startTime).getMsec() < 1000.0))
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
