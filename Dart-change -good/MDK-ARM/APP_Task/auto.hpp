#ifndef __AutoLaunch_Hpp
#define __AutoLaunch_Hpp

#include "InHpp.hpp"

enum LaunchMode {
    DT7CtrlMode = 1,
    RfSysMode   = 2,
};

class AutoLaunchSystem
{
public:
    // ===================== 主状态 =====================
    enum State {
        IDLE = 0,
        AIM_RESET,      // 瞄准前清零/跳变脉冲 
        RELOAD_AIM,     // 装填 + 视觉瞄准 
        RELEASE,        // 舵机3释放飞镖
        DONE,           // 本发完成
        FINISH          // 全部发射完毕
    };

    // ===================== 装填子状态 =====================
    enum SubState {
        PLATFORM_UP = 0,    // 升降台上升 2、3、4
        SLIDE_DART,         // 推镖滑落 3、4
        DOWN_PARALLEL,      // 并行下行 (升降台 + 上膛)
        RELOAD_UP,          // 上膛电机上行
        PLATFORM_CHECK,     // 升降台检查到顶 1
        SUB_DONE            // 装填完成
    };

    // ===================== 电机速度常量 =====================
    enum Speed {
        LIFT_SPEED_DOWN  = 4000,
        LIFT_SPEED_UP    = 4000,
        LOAD_SPEED_DOWN  = -4000,
        LOAD_SPEED_UP    = 5000
    };

public:
    int  launchState;           // 当前主状态机状态
    int  launchedCount;         // 已发射飞镖数 (0~4)
    int  currentDartIndex;      // 当前飞镖序号 (1~4)，供视觉使用
    int  launchTime;            // 定时器 (SystemTick)
    int  resetStage;            // 瞄准重置子状态 (0/1/2)
    bool angleReached;          // Yaw角度是否到达

    // ---- 并行装填轨道 ----
    int  reloadSubState;        // 装填子状态机
    bool reloadReady;           // 装填是否完成
    int  reloadTime;            // 装填轨道定时器

    void StartAutoLaunch(int mode);
    void HandleLaunchTrigger(int mode);
    void StartNewLaunch();
    void CheckStateReset(int mode);

    void VisionControl();
    void SyncYawAngle();

    void StopLoadMotors(void);
    void StopLiftMotors(void);

private:
    bool targetLocked;
    bool hasCounted;
    bool hasAimedThisDart;

    void HandleAimReset();
    void HandleReloadSubMachine();

    bool NeedSlideDart(void) const;
    bool NeedPlatformMove(void) const;
};

extern AutoLaunchSystem AutoLaunch;

#endif