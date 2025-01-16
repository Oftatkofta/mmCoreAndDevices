#include "ThorlabsMCM3001.h"
#include "ModuleInterface.h"
#include <sstream>

// Define the static member
const double ThorlabsMCM3001::DEFAULT_ENCODER_RESOLUTION_UM = 0.2116667;  // For ZFM2020/ZFM2030

MODULE_API void InitializeModuleData()
{
    RegisterDevice("ThorlabsMCM3001", 
                  MM::StageDevice, 
                  "Thorlabs MCM3001 3-Channel Controller for Motorized Stages (ZFM2020/ZFM2030)");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
    if (deviceName == 0)
        return 0;

    if (strcmp(deviceName, "ThorlabsMCM3001") == 0)
        return new ThorlabsMCM3001();

    return 0;
}

MODULE_API void DeleteDevice(MM::Device* pDevice)
{
    delete pDevice;
}

ThorlabsMCM3001::ThorlabsMCM3001() :
    initialized_(false),
    busy_(false),
    stepSizeUm_(0.1),
    posUm_(0.0),
    port_(""),
    currentAxis_(0),
    encoderResolutionUm_(DEFAULT_ENCODER_RESOLUTION_UM)
{
    InitializeDefaultErrorMessages();

    // Add custom error messages
    SetErrorText(ERR_PORT_CHANGE_FORBIDDEN, "Port cannot be changed while device is in use.");
    SetErrorText(ERR_INVALID_AXIS, "Invalid axis specified (valid: 0-2).");
    SetErrorText(ERR_COMMAND_FAILED, "Command failed or no response from device.");
    SetErrorText(DEVICE_SERIAL_COMMAND_FAILED, "Command failed or no response from device.");
    SetErrorText(DEVICE_NOT_CONNECTED, "Invalid serial port configuration.");
    SetErrorText(DEVICE_INVALID_PROPERTY_VALUE, "Invalid axis specified (valid: 0-2).");
    
    // Create pre-initialization properties
    CPropertyAction* pAct = new CPropertyAction (this, &ThorlabsMCM3001::OnPort);
    CreateProperty(MM::g_Keyword_Port, "Undefined", MM::String, false, pAct, true);

    // Create axis selection property
    pAct = new CPropertyAction(this, &ThorlabsMCM3001::OnAxis);
    CreateProperty("Axis", "0", MM::Integer, false, pAct);
    SetPropertyLimits("Axis", 0, 2);

    // Add encoder resolution property
    pAct = new CPropertyAction(this, &ThorlabsMCM3001::OnEncoderResolution);
    std::ostringstream defaultValue;
    defaultValue.precision(7);
    defaultValue << DEFAULT_ENCODER_RESOLUTION_UM;
    
    CreateProperty("EncoderResolution(um/count)", 
                  defaultValue.str().c_str(), 
                  MM::Float, 
                  false, 
                  pAct, 
                  "Stage-specific conversion factor (micrometers per encoder count). "
                  "Default value 0.2116667 is calibrated for Thorlabs ZFM2020 and ZFM2030 stages. "
                  "Change only if using a different stage model.");
                  
    SetPropertyLimits("EncoderResolution(um/count)", 0.0001, 10.0);
}




