#pragma once
#include <MMDevice.h>
#include <DeviceBase.h>
#include <string>

class myFocusController : public CGenericBase<myFocusController>
{
public:
    myFocusController();
    ~myFocusController();

    // MM Device API
    int Initialize();
    int Shutdown();
    void GetName(char* pszName) const;
    bool Busy();

    // Movement commands
    int SetPositionSteps(long steps);
    int GetPositionSteps(long& steps);
    int Stop();
    int SetOrigin();

    // Helper methods
    double StepsToMicrons(long steps) const { return steps * conversionFactor_; }
    long MicronsToSteps(double microns) const { return static_cast<long>(microns / conversionFactor_); }

    // Action interface
    int OnPort(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
    // Communication methods
    int SendCommand(const unsigned char* command, unsigned length);
    int GetResponse(unsigned char* response, unsigned length);
    bool GetMotorStatus();
    int ClearPort();

    // Member variables
    std::string port_;
    bool initialized_;
    const double conversionFactor_; // 0.2116667 um per count

    // Command constants
    static const int QUERY_POS_LENGTH = 6;
    static const int SET_POS_LENGTH = 12;
    static const int STOP_LENGTH = 6;
    static const int STATUS_LENGTH = 6;
};

