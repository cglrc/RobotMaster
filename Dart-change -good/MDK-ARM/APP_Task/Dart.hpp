#ifndef __Dart_Hpp
#define __Dart_Hpp

#include "InHpp.hpp"

#define Dart_Launch_Opening_Status dart_client_cmd_0x020A.dart_launch_opening_status
#define Dart_Remaining_Time        dart_info_0x0105.dart_remaining_time
#define DT7ConnectState            DT7UartCom.Connect_State
#define YAW_CENTER                 3497

class DartRobot
{
private:
    int PlatRL, PlatLL, PlatLH;
    int TensLim, ReloadL, ReloadHL, ReloadHR;

    void ReadSensors();

public:
    int RefereeState;
    int YawAngle;
    bool CtrlLock;
    bool Misaligned;

    static const int DEAD = 200;

    enum Speed {
        LOAD_UP   =  1500,
        LOAD_DOWN = -4000,
        SPRING_UP =  500,
        SPRING_DN = -500,
        STOP      =  0
    };

    __attribute__((used)) void Init();
    __attribute__((used)) void Run();
    __attribute__((used)) void CheckControllerConnection();
    __attribute__((used)) void ReloadCtrl();
    __attribute__((used)) void MotorCtrl();
    __attribute__((used)) void AutoLaunchMode();
    __attribute__((used)) void EStop();
    __attribute__((used)) void AddTarget(int *target, int add, int min, int max);
    __attribute__((used)) int Clamp(int val, int min, int max);

    bool PlatHighL()  const { return PlatLH == 1; }
    bool PlatLowR()   const { return PlatRL == 1; }
    bool PlatLowL()   const { return PlatLL == 1; }
    bool ReloadHigh() const { return ReloadHL == 0 || ReloadHR == 0; }
    bool ReloadLow()  const { return ReloadL == 0; }
    bool TensionHit() const { return TensLim == 0; }
};

enum Switch {
    SW_UP   = 1,
    SW_DOWN = 2,
    SW_MID  = 3,
};

enum YawLimit {
    YAW_MAX = 3700,
    YAW_MIN = 3300,
};

extern DartRobot DartBot;
extern TickType_t SystemTick;

#endif // __Dart_Hpp