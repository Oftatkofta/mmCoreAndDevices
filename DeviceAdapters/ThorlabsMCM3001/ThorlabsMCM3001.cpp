#include "ThorlabsMCM3001.h"
#include "ModuleInterface.h"
#include "MCM3000_SDK.h"

ThorlabsMCM3001::ThorlabsMCM3001() :
    initialized_(false),
    busy_(false),
    stepSizeUm_(ENCODER_RESOLUTION_UM),
    posUm_(0.0),
    port_("")
{
    InitializeDefaultErrorMessages();

    // Add custom error messages
    SetErrorText(ERR_PORT_CHANGE_FORBIDDEN, "Port change is not allowed while the device is connected.");
    SetErrorText(ERR_INVALID_SERIAL_PARAMS, "Invalid serial port parameters.");
    SetErrorText(ERR_COMMAND_FAILED, "Command failed or no response from device.");
    
    // Create pre-initialization properties
    CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false);
}



ThorlabsMCM3001::~ThorlabsMCM3001()
{
    Shutdown(); // Ensure proper resource cleanup
}

int ThorlabsMCM3001::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    // Set up serial port
    if (port_.empty())
        return DEVICE_ERR;

    int ret = SetupSerialPort();
    if (ret != DEVICE_OK)
        return ret;

    // Clear communication
    ret = ClearPort();
    if (ret != DEVICE_OK)
        return ret;

    // Create post-initialization properties
    CreateProperty("StepSize", CDeviceUtils::ConvertToString(stepSizeUm_), 
        MM::Float, true);

    // Add status property
    CPropertyAction* pAct = new CPropertyAction(this, &ThorlabsMCM3001::OnStatus);
    int ret = CreateProperty("Status", "Idle", MM::String, true, pAct);
    if (ret != DEVICE_OK)
        return ret;

    initialized_ = true;
    return DEVICE_OK;
}

int ThorlabsMCM3001::OnStatus(MM::PropertyBase* pProp, MM::ActionType eAct)
{
    if (eAct == MM::BeforeGet)
    {
        std::string status = busy_ ? "Busy" : "Idle";
        pProp->Set(status.c_str());
    }
    return DEVICE_OK;
}






MODULE_API void InitializeModuleData()
{
    RegisterDevice("ThorlabsMCM3001", MM::StageDevice, "Thorlabs MCM3001 Stage");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    if (deviceName == nullptr)
        return nullptr;

    if (strcmp(deviceName, "ThorlabsMCM3001") == 0)
    {
        return new ThorlabsMCM3001();
    }

    return nullptr;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
    delete pDevice;
}

int ThorlabsMCM3001::SetPositionUm(double posUm)
{
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    long deviceUnits = UmToDeviceUnits(posUm);

    if (!SetParam(PARAM_Z_POS, static_cast<double>(deviceUnits)))
    {
        LogMessage("Failed to set Z position.", true);
        return DEVICE_ERR;
    }

    return PrepareAndStartMovement();
}



int ThorlabsMCM3001::GetPositionUm(double& posUm)
{
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    double deviceUnits = 0.0;
    if (!GetParam(PARAM_Z_POS_CURRENT, deviceUnits))
    {
        LogMessage("Failed to get current Z position.", true);
        return DEVICE_ERR;
    }

    long counts = static_cast<long>(deviceUnits);
    posUm = DeviceUnitsToUm(counts);
    return DEVICE_OK;
}



int ThorlabsMCM3001::SetPositionSteps(long steps)
{
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    if (!SetParam(PARAM_Z_POS, static_cast<double>(steps)))
    {
        LogMessage("Failed to set Z position (steps).", true);
        return DEVICE_ERR;
    }

    return PrepareAndStartMovement();
}


int ThorlabsMCM3001::GetPositionSteps(long& steps)
{
    if (!initialized_)
        return DEVICE_NOT_CONNECTED;

    double deviceUnits = 0.0;
    if (!GetParam(PARAM_Z_POS_CURRENT, deviceUnits))
    {
        LogMessage("Failed to get current Z position (steps).", true);
        return DEVICE_ERR;
    }

    steps = static_cast<long>(deviceUnits);
    return DEVICE_OK;
}



int ThorlabsMCM3001::Shutdown()
{
    initialized_ = false;
    return DEVICE_OK; // or appropriate return code
}

void ThorlabsMCM3001::GetName(char* name) const
{
    CDeviceUtils::CopyLimitedString(name, "ThorlabsMCM3001");
}

bool ThorlabsMCM3001::Busy()
{
    // Update busy_ based on device status
    // For example:
    long status = STATUS_READY;
    if (!StatusPosition(status))
    {
        LogMessage("Failed to get status position.", true);
        return false;
    }
    busy_ = (status == STATUS_BUSY);
    return busy_;
}



int ThorlabsMCM3001::SetOrigin()
{
    if (!SetParam(PARAM_Z_ZERO, 0.0))
    {
        LogMessage("Failed to zero Z axis.", true);
        return DEVICE_ERR;
    }
    return DEVICE_OK;
}



int ThorlabsMCM3001::GetLimits(double& min, double& max)
{
    long paramType = 0, paramAvailable = 0, paramReadOnly = 0;
    double paramMin = 0.0, paramMax = 0.0, paramDefault = 0.0;

    if (!GetParamInfo(PARAM_Z_POS, paramType, paramAvailable, paramReadOnly,
        paramMin, paramMax, paramDefault))
    {
        LogMessage("Failed to get parameter info for Z position.", true);
        return DEVICE_ERR;
    }

    min = DeviceUnitsToUm(static_cast<long>(paramMin));
    max = DeviceUnitsToUm(static_cast<long>(paramMax));
    return DEVICE_OK;
}



int ThorlabsMCM3001::IsStageSequenceable(bool& isSequenceable) const
{
    isSequenceable = false; // Update this based on your hardware capabilities
    return DEVICE_OK;
}

bool ThorlabsMCM3001::IsContinuousFocusDrive() const
{
    return false; // Update based on the intended usage of the device
}

int ThorlabsMCM3001::PrepareAndStartMovement()
{
    busy_ = true;

    if (!SetupPosition())
    {
        LogMessage("Failed to setup position.", true);
        busy_ = false;
        return DEVICE_ERR;
    }

    if (!StartPosition())
    {
        LogMessage("Failed to start position movement.", true);
        busy_ = false;
        return DEVICE_ERR;
    }

    // Optionally, you can wait for movement to complete here
    // Or rely on the Busy() method to check the status

    busy_ = false; // Set to false if movement is complete
    return DEVICE_OK;
}

int ThorlabsMCM3001::SetupSerialPort()
{
    int ret = DEVICE_OK;
    
    // Set serial port properties
    ret = SetPropertyValue(port_.c_str(), MM::g_Keyword_BaudRate, "460800");
    if (ret != DEVICE_OK) return ret;

    ret = SetPropertyValue(port_.c_str(), MM::g_Keyword_DataBits, "8");
    if (ret != DEVICE_OK) return ret;

    ret = SetPropertyValue(port_.c_str(), MM::g_Keyword_StopBits, "1");
    if (ret != DEVICE_OK) return ret;

    ret = SetPropertyValue(port_.c_str(), MM::g_Keyword_Parity, "None");
    if (ret != DEVICE_OK) return ret;

    ret = SetPropertyValue(port_.c_str(), MM::g_Keyword_Handshaking, "Off");
    if (ret != DEVICE_OK) return ret;

    return DEVICE_OK;
}




