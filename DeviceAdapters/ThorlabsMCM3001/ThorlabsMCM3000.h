#ifndef THORLABS_MCM3000_H
#define THORLABS_MCM3000_H

#include "DeviceBase.h"
#include "MCM3000_SDK.h" // Include the SDK header file

class ThorlabsMCM3000 : public CStageBase<ThorlabsMCM3000>
{
public:
    ThorlabsMCM3000();
    ~ThorlabsMCM3000();

    int Initialize() override;
    int Shutdown() override;
    void GetName(char* pszName) const override;
    bool Busy() override;
    int SetPositionUm(double pos);
    int GetPositionUm(double& pos);
    int SetOrigin() override;
    int GetLimits(double& min, double& max) override;

    // Implementations of pure virtual functions from MM::Stage
    int SetPositionSteps(long steps) override;
    int GetPositionSteps(long& steps) override;
    int IsStageSequenceable(bool& isSequenceable) const override;
    bool IsContinuousFocusDrive() const override;

private:
    bool initialized_;
    bool busy_;
    double posUm_;

    int PrepareAndStartMovement();
    int CheckStatus();
};

#endif // THORLABS_MCM3000_H
