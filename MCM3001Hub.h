#pragma once

#include "../../MMDevice/MMDevice.h"  // For error codes and device base classes
#include "../../MMDevice/DeviceBase.h"
#include <string>
#include <vector>

/**
 * MCM3001Hub - Hub device for MCM3001 controller
 * 
 * Error codes used (defined in MMDevice.h):
 * DEVICE_OK: Operation successful
 * DEVICE_ERR: Unspecified error occurred
 * DEVICE_SERIAL_COMMAND_FAILED: Command was sent but controller rejected it
 * DEVICE_SERIAL_NO_RESPONSE: No response received from controller
 * DEVICE_SERIAL_INVALID_RESPONSE: Response received but in wrong format
 * DEVICE_SERIAL_PORT_COMMAND_TIMEOUT: Serial command timed out
 * DEVICE_NOT_INITIALIZED: Device used before initialization
 */
class MCM3001Hub : public HubBase<MCM3001Hub>
{
public:
    MCM3001Hub();
    ~MCM3001Hub();

    // MM::Device API
    int Initialize();
    int Shutdown();
    void GetName(char* name) const;
    bool Busy();
    
    // HubBase API
    int DetectInstalledDevices();

    // Custom interface for child devices
    bool SendCommand(const std::string& command, std::string& response);
    int GetControllerVersion(std::string& version);
    
private:
    std::string port_;
    bool initialized_;
    std::string name_;
    
    // Serial port related
    int ReadResponse(std::string& response);
    int ClearPort();
}; 