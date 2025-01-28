#pragma once
#include "MMDevice.h"
#include "DeviceBase.h"
#include "DeviceThreads.h"
#include "ModuleInterface.h"
#include <string>

// Error codes
#define ERR_PORT_CHANGE_FORBIDDEN    10004
#define ERR_UNRECOGNIZED_ANSWER      10009
#define ERR_UNSPECIFIED_ERROR        10010
#define ERR_HOME_REQUIRED            10011
#define ERR_INVALID_PACKET_LENGTH    10012
#define ERR_RESPONSE_TIMEOUT         10013
#define ERR_BUSY                     10014
#define ERR_STEPS_OUT_OF_RANGE       10015
#define ERR_STAGE_NOT_ZEROED         10016

// Command lengths
const int SET_POS_LENGTH = 12;
const int QUERY_POS_LENGTH = 6;
const int STATUS_LENGTH = 6;

class myFocusController : public CStageBase<myFocusController>
{
public:
    myFocusController();
    ~myFocusController();

    // MMDevice API
    int Initialize();
    int Shutdown();
    void GetName(char* name) const;
    bool Busy();

    // Stage API
    int SetPositionUm(double pos);
    int GetPositionUm(double& pos);
    int SetPositionSteps(long steps);
    int GetPositionSteps(long& steps);
    int GetLimits(double& lower, double& upper);
    int Home();
    int Stop();
    int SetOrigin();
    int Move(double velocity);
    int SetAdapterOriginUm(double d);
    int MoveBlocking(long steps, bool relative = false);

    // Focus-specific functions
    int GetFocusDirection(MM::FocusDirection& direction);
    bool IsContinuousFocusDrive() const;
    int IsStageSequenceable(bool& isSequenceable) const;
    int IsStageLinearSequenceable(bool& isSequenceable) const;

    // Action interface
    int OnPort(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnStepSizeUm(MM::PropertyBase* pProp, MM::ActionType eAct);

    // Device specific constants
    static const char* DeviceName;
    static const char* Description;

private:
    int SendCommand(const unsigned char* command, unsigned length);
    int GetResponse(unsigned char* response, unsigned length);
    int ClearPort();

    bool initialized_;
    std::string port_;
    double stepSizeUm_;
    double answerTimeoutMs_;
    bool home_;
    long curSteps_;
    MM::MMTime lastMoveTime_;
};

