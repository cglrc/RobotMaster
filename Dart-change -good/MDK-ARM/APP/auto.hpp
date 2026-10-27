#ifndef __AutoLaunch_Hpp
#define __AutoLaunch_Hpp

#include "InHpp.hpp"

enum LaunchMode {
    DT7CtrlMode = 1,
    RfSysMode   = 2,
};

class AutolauchSys
{
public:
    // ---- 主状态机：单发飞镖的完整生命周期，由 launchState 驱动 ----
    enum State {
        IDLE = 0,      // 待命：舵机归位、清标志位，电机不动作
        AIM_RESET,     // 每发开始前给视觉打重置脉冲，防止上一发旧指令导致误发射
        RELOAD_AIM,    // 核心状态：并行推进「装填子状态机」与「视觉瞄准」
        RELEASE,       // 舵机3 撒放飞镖，等待 500ms 机械回弹
        DONE,          // 单发后处理：计数，未满 4 发则回到 AIM_RESET 发下一发
        FINISH         // 4 发打完：升降台升到顶、舵机全部归位，等待复位条件
    };

    // ---- 装填子状态机：仅在 RELOAD_AIM 内串行推演，由 reloadSubState 驱动 ----
    enum SubState {
        PLATFORM_UP = 0,    // 升降台升到顶位(PlatHighL)；第 1 发不需要，由 NeedPlatformMove() 跳过
        SLIDE_DART,         // 推镖：舵机4 前 400ms 打 25° 再回 0°；仅第 3/4 发需要，由 NeedSlideDart() 跳过
        DOWN_PARALLEL,      // 时间优化点：上膛电机下行找 ReloadLow() 的同时升降台同步下降
        RELOAD_UP,          // 上膛电机上升找合拢限位 ReloadHigh()
        PLATFORM_CHECK,     // 复查升降台是否到顶(PlatHighL)，未到则继续升
        SUB_DONE            // 装填完成，置 reloadReady = true，与视觉的 aimReady 汇合才进 RELEASE
    };

    enum Speed {
        LIFT_SPEED_DOWN  = 4000,
        LIFT_SPEED_UP    = 4000,
        LOAD_SPEED_DOWN  = -6000,
        LOAD_SPEED_UP    = 2700
    };

public:
    int  launchState;
    int  launchedCount;
    int  currentDartIndex;
    int  launchTime;
    int  resetStage;
    bool angleReached;

    int  reloadSubState;
    bool reloadReady;
    int  reloadTime;

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

extern AutolauchSys AutoLaunch;

#endif