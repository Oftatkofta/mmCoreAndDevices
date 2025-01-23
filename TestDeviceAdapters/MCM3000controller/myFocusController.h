#pragma once
#include <MMDevice.h>
#include <DeviceBase.h>
#include <string>
#include <DeviceThreads.h>

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

    // Focus-specific functions
    int GetFocusDirection(MM::FocusDirection& direction);
    bool IsContinuousFocusDrive() const;
    int IsStageSequenceable(bool& isSequenceable) const;
    int IsStageLinearSequenceable(bool& isSequenceable) const;

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

    // Private constants
    enum {
        SET_POS_LENGTH = 12,
        QUERY_POS_LENGTH = 6,
        STATUS_LENGTH = 6
    };

    bool initialized_;
    std::string port_;
    double stepSizeUm_;
    double answerTimeoutMs_;
    CommandThread* cmdThread_;
    bool home_;
    long curSteps_;
    MM::MMTime lastMoveTime_;
};

