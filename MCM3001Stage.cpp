#include "MCM3001Stage.h"
#include "MCM3001Hub.h"
#include "../../MMDevice/ModuleInterface.h"
#include <sstream>

const char* g_StageDeviceName = "MCM3001Stage";
const char* g_PropertyAxisNumber = "Axis";
const char* g_PropertyStepSize = "StepSize(um)";

MCM3001Stage::MCM3001Stage() :
    initialized_(false),
    axisNumber_(1),
    stepSizeUm_(0.1),
    hub_(NULL)
{
    InitializeDefaultErrorMessages();
    
    // Add to the standard error messages
    SetErrorText(DEVICE_SERIAL_COMMAND_FAILED, "Command failed on the controller.");
    SetErrorText(DEVICE_SERIAL_NO_RESPONSE, "No response received from the controller.");
    
    // Create pre-initialization properties
    CreateProperty(g_PropertyAxisNumber, "1", MM::Integer, false);
    CreateProperty(g_PropertyStepSize, "0.1", MM::Float, false);
}

MCM3001Stage::~MCM3001Stage()
{
    Shutdown();
}

void MCM3001Stage::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(name, g_StageDeviceName);
}

bool MCM3001Stage::Busy()
{
    if (!initialized_)
        return false;

    std::string response;
    std::ostringstream command;
    command << "?busy " << axisNumber_;
    
    if (hub_->SendCommand(command.str(), response))
    {
        return (response.find("1") != std::string::npos);
    }
    return false;
}

int MCM3001Stage::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    // Get hub
    MM::Hub* genericHub = GetParentHub();
    if (!genericHub)
        return DEVICE_ERR;
    hub_ = static_cast<MCM3001Hub*>(genericHub);

    // Set property limits
    SetPropertyLimits(g_PropertyAxisNumber, 1, 3);
    SetPropertyLimits(g_PropertyStepSize, 0.0001, 1000.0);

    initialized_ = true;
    return DEVICE_OK;
}

int MCM3001Stage::Shutdown()
{
    initialized_ = false;
    return DEVICE_OK;
}

int MCM3001Stage::SetPositionUm(double pos)
{
    long steps = (long)(pos / stepSizeUm_);
    return SetPositionSteps(steps);
}

int MCM3001Stage::GetPositionUm(double& pos)
{
    long steps;
    int ret = GetPositionSteps(steps);
    if (ret != DEVICE_OK)
        return ret;
    
    pos = steps * stepSizeUm_;
    return DEVICE_OK;
}

int MCM3001Stage::SetPositionSteps(long steps)
{
    std::ostringstream command;
    command << "move " << axisNumber_ << " " << steps;
    
    std::string response;
    if (!hub_->SendCommand(command.str(), response))
        return DEVICE_SERIAL_NO_RESPONSE;
    
    if (response.find("OK") == std::string::npos)
        return DEVICE_SERIAL_COMMAND_FAILED;
    
    return DEVICE_OK;
}

int MCM3001Stage::GetPositionSteps(long& steps)
{
    std::ostringstream command;
    command << "?pos " << axisNumber_;
    
    std::string response;
    if (!hub_->SendCommand(command.str(), response))
        return DEVICE_SERIAL_NO_RESPONSE;
    
    std::istringstream iss(response);
    iss >> steps;
    
    return DEVICE_OK;
}

int MCM3001Stage::SetOrigin()
{
    std::ostringstream command;
    command << "zero " << axisNumber_;
    
    std::string response;
    if (!hub_->SendCommand(command.str(), response))
        return DEVICE_SERIAL_NO_RESPONSE;
    
    if (response.find("OK") == std::string::npos)
        return DEVICE_SERIAL_COMMAND_FAILED;
    
    return DEVICE_OK;
}

int MCM3001Stage::GetLimits(double& lower, double& upper)
{
    // MCM3001 has fixed travel range of ±12.7mm
    lower = -12700.0;  // micrometers
    upper = 12700.0;   // micrometers
    return DEVICE_OK;
}

int MCM3001Stage::OnAxisNumber(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set((long)axisNumber_);
    }
    else if (eAct == MM::AfterSet)
    {
        long axis;
        pProp->Get(axis);
        if (axis >= 1 && axis <= 3)
            axisNumber_ = (int)axis;
    }
    return DEVICE_OK;
}

int MCM3001Stage::OnStepSizeUm(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        pProp->Set(stepSizeUm_);
    }
    else if (eAct == MM::AfterSet)
    {
        pProp->Get(stepSizeUm_);
    }
    return DEVICE_OK;
} 