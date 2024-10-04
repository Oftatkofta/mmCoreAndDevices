#ifndef _THORLABS_MCM3000_H_
#define _THORLABS_MCM3000_H_

#include "DeviceBase.h"
#include "MCM3000_SDK.h" // Include the SDK header file

class ThorlabsMCM3000 : public CStageBase<ThorlabsMCM3000>
{
public:
    ThorlabsMCM3000();
    ~ThorlabsMCM3000();

    int Initialize();
    int Shutdown();
    void GetName(char* pszName) const;
    bool Busy();
    int SetPositionUm(double pos);
    int GetPositionUm(double& pos);
    int SetOrigin();
    int GetLimits(double& min, double& max);

private:
    bool initialized_;
    bool busy_;
    double posUm_;

    int PrepareAndStartMovement();
    int CheckStatus();
};

#endif // _THORLABS_MCM3000_H_
