#pragma once
#include <MMDevice.h>
#include <DeviceBase.h>
#include <string>
#include <DeviceThreads.h>

// Command lengths
const int SET_POS_LENGTH = 12;
const int QUERY_POS_LENGTH = 6;
const int STATUS_LENGTH = 6;

// Error codes
const int ERR_PORT_CHANGE_FORBIDDEN = 10004;
const int ERR_UNRECOGNIZED_ANSWER = 10009;
const int ERR_RESPONSE_TIMEOUT = 10013;
const int ERR_BUSY = 10014;
const int ERR_HOME_REQUIRED = 10015;

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
    int SetOrigin() { return SetPositionSteps(0); }

    // Focus-specific functions
    int GetFocusDirection(MM::FocusDirection& direction) { direction = MM::FocusDirectionUnknown; return DEVICE_OK; }
    bool IsContinuousFocusDrive() const { return false; }
    int IsStageSequenceable(bool& isSequenceable) const { isSequenceable = false; return DEVICE_OK; }
    int IsStageLinearSequenceable(bool& isSequenceable) const { isSequenceable = false; return DEVICE_OK; }

    // Action interface
    int OnPort(MM::PropertyBase* pProp, MM::ActionType eAct);
    int OnStepSizeUm(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
    class CommandThread : public MMDeviceThreadBase
    {
    public:
        CommandThread(myFocusController* stage);
        ~CommandThread() {}
        
        int svc();
        void Stop() {stop_ = true;}
        bool GetStop() const {return stop_;}
        int GetErrorCode() const {return errCode_;}
        bool IsMoving() const {return moving_;}
        
        void StartMove(long pos, bool relative = false);

    private:
        void Reset() {stop_ = false; errCode_ = DEVICE_OK; moving_ = false;}
        bool stop_;
        bool moving_;
        bool relative_;
        myFocusController* stage_;
        long pos_;
        int errCode_;
    };

    int SendCommand(const unsigned char* command, unsigned length);
    int GetResponse(unsigned char* response, unsigned length);
    int ClearPort();
    bool GetMotorStatus();
    int MoveBlocking(long steps, bool relative = false);

    bool initialized_;
    std::string port_;
    double stepSizeUm_;
    double answerTimeoutMs_;
    CommandThread* cmdThread_;
    bool home_;
    long curSteps_;
    MM::MMTime lastMoveTime_;
};

