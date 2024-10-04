#include "ThorlabsMCM3000.h"
#include "ModuleInterface.h"
#include "MCM3000_SDK.h"

ThorlabsMCM3000::ThorlabsMCM3000() : initialized_(false), busy_(false), posUm_(0.0)
{
    InitializeDefaultErrorMessages();
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
    RegisterDevice("ThorlabsMCM3000", MM::StageDevice, "Thorlabs MCM3000 XYZ Stage");
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
