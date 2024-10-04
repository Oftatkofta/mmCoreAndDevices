#include "ThorlabsMCM3000.h"
#include "ModuleInterface.h"
#include "MCM3000_SDK.h"

ThorlabsMCM3000::ThorlabsMCM3000() : initialized_(false), busy_(false), posUm_(0.0)
{
    InitializeDefaultErrorMessages();
}

ThorlabsMCM3000::~ThorlabsMCM3000()
{
    // Any required cleanup
}

int ThorlabsMCM3000::Initialize()
{
    if (initialized_)
        return DEVICE_OK;

    long deviceCount = 0;
    if (!FindDevices(deviceCount) || deviceCount == 0)
    {
        return DEVICE_ERR;
    }

    if (!SelectDevice(0))
    {
        return DEVICE_ERR;
    }

    initialized_ = true;
    return DEVICE_OK;
}

int ThorlabsMCM3000::SetPositionUm(double pos)
{
    if (0 != SetParam(PARAM_X_POS, pos / 10.0))
    {
        return DEVICE_ERR;
    }

    return PrepareAndStartMovement();
}

MODULE_API void InitializeModuleData()
{
    RegisterDevice("ThorlabsMCM3000", MM::ZStageDevice, "Thorlabs MCM3000 Z Stage");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    if (deviceName == nullptr)
        return nullptr;

    if (strcmp(deviceName, "ThorlabsMCM3000") == 0)
    {
        return new ThorlabsMCM3000();
    }

    return nullptr;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
    delete pDevice;
}
int ThorlabsMCM3000::SetPositionSteps(long steps)
{
    // Convert steps to micrometers if required by the device
    double posUm = static_cast<double>(steps) * 0.1;  // Example conversion factor
    return SetPositionUm(posUm);
}

int ThorlabsMCM3000::GetPositionSteps(long& steps)
{
    double posUm = 0.0;
    int ret = GetPositionUm(posUm);
    if (ret != DEVICE_OK)
        return ret;

    // Convert micrometers to steps
    steps = static_cast<long>(posUm * 10);  // Example conversion factor
    return DEVICE_OK;
}

int ThorlabsMCM3000::IsStageSequenceable(bool& isSequenceable) const
{
    // If the stage supports sequenced positions, set this to true
    isSequenceable = false;  // Assuming the MCM3000 does not support this feature
    return DEVICE_OK;
}

bool ThorlabsMCM3000::IsContinuousFocusDrive() const
{
    // Return true if this stage functions as a continuous focus drive
    return false;  // Assuming this is not a continuous focus drive
}
int ThorlabsMCM3000::Shutdown()
{
    initialized_ = false;
    return DEVICE_OK; // or appropriate status code
}

void ThorlabsMCM3000::GetName(char* pszName) const
{
    CDeviceUtils::CopyLimitedString(pszName, "Thorlabs MCM3000 XYZ Stage");
}

bool ThorlabsMCM3000::Busy()
{
    return busy_;
}


int ThorlabsMCM3000::SetOrigin()
{
    // Implement logic to set the origin, depending on the device capabilities
    posUm_ = 0.0; // Example implementation
    return DEVICE_OK;
}

int ThorlabsMCM3000::GetLimits(double& min, double& max)
{
    min = 0.0;        // Set appropriate minimum limit
    max = 25000.0;    // Set appropriate maximum limit in micrometers
    return DEVICE_OK;
}

int ThorlabsMCM3000::PrepareAndStartMovement()
{
    // Add the logic for preparing the movement (e.g., communication with hardware)
    busy_ = true; // Example: set the stage to busy before movement
    // ... add movement logic
    busy_ = false; // Example: clear busy after movement completes
    return DEVICE_OK;
}

int ThorlabsMCM3000::SetPositionSteps(long steps)
{
    double posUm = static_cast<double>(steps) * 0.1;  // Conversion factor for Z-stage
    return SetPositionUm(posUm);
}

int ThorlabsMCM3000::GetPositionSteps(long& steps)
{
    double posUm = 0.0;
    int ret = GetPositionUm(posUm);
    if (ret != DEVICE_OK)
        return ret;

    steps = static_cast<long>(posUm * 10);  // Conversion factor for Z-stage
    return DEVICE_OK;
}





