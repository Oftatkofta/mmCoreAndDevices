#ifndef THORLABS_MCM3001_H
#define THORLABS_MCM3001_H

#include "DeviceBase.h"
#include "MCM3000_SDK.h" // Include the correct SDK header file

#include <mutex>

class ThorlabsMCM3001 : public CStageBase<ThorlabsMCM3001>
{
public:
    ThorlabsMCM3001();
    ~ThorlabsMCM3001();

    // Device API
    int Initialize() override;
    int Shutdown() override;
    void GetName(char* pszName) const override;
    bool Busy() override;

    // Stage API
    int SetPositionUm(double pos) override;
    int GetPositionUm(double& pos) override;
    int SetPositionSteps(long steps) override;
    int GetPositionSteps(long& steps) override;
    int SetOrigin() override;
    int GetLimits(double& min, double& max) override;
    int IsStageSequenceable(bool& isSequenceable) const override;
    bool IsContinuousFocusDrive() const override;

private:
    bool initialized_;
    bool busy_; // Add this line
    double posUm_;
    double stepSizeUm_;
    double maxTravelUm_;
    static constexpr double scalingFactor_ = 0.2116667; // μm per encoder count
    std::string deviceID_;
    std::mutex sdkMutex_;

    // Helper methods
    long UmToDeviceUnits(double um) const
    {
        return static_cast<long>(um / scalingFactor_);
    }

    double DeviceUnitsToUm(long counts) const
    {
        return counts * scalingFactor_;
    }
    int PrepareAndStartMovement();
    int CheckStatus();
};

#endif // THORLABS_MCM3001_H
