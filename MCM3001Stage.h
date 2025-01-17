#pragma once

#include "../../MMDevice/MMDevice.h"
#include "../../MMDevice/DeviceBase.h"
#include <string>
#include <cmath>

class MCM3001Hub;  // Forward declaration

/**
 * MCM3001Stage - Stage device for MCM3001 controller
 * 
 * Error codes used (defined in MMDevice.h):
 * DEVICE_OK: Operation successful
 * DEVICE_ERR: Unspecified error occurred
 * DEVICE_SERIAL_COMMAND_FAILED: Command was sent but stage rejected it
 * DEVICE_SERIAL_NO_RESPONSE: No response received from stage
 * DEVICE_INVALID_PROPERTY_VALUE: Property value is out of range
 * DEVICE_NOT_INITIALIZED: Device used before initialization
 * 
 * Properties:
 * - Axis: Integer (1-3) specifying which axis this stage controls
 * - StepSize: Float (μm) specifying the size of one step
 */
class MCM3001Stage : public CStageBase<MCM3001Stage>
{
public:
    MCM3001Stage();
    ~MCM3001Stage();

    // MM::Device API
    int Initialize();
    int Shutdown();
    void GetName(char* name) const;
    bool Busy();

    // Stage API
    int SetPositionUm(double pos);
    int GetPositionUm(double& pos);
    int SetPositionSteps(long steps);
    int GetPositionSteps(long& steps);
    int SetOrigin();
    int GetLimits(double& lower, double& upper);
    
    // Property handlers
    int OnAxisNumber(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnStepSizeUm(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
    int axisNumber_;
    double stepSizeUm_;
    bool initialized_;
    std::string name_;
    MCM3001Hub* hub_;
}; 