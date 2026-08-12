
#include "InHpp.hpp"

// 全局变量 
AutoLaunchSystem AutoLaunch;

//公用动作封装

void AutoLaunchSystem::StopLoadMotors(void) {
    Motors[RIGHT_LOAD].Target = 0;
}

void AutoLaunchSystem::StopLiftMotors(void) {
    Motors[LEFT_LIFT].Target  = 0;
    Motors[RIGHT_LIFT].Target = 0;
}

// 第3、4发需要推镖滑落
bool AutoLaunchSystem::NeedSlideDart(void) const {
    return (launchedCount >= 2);
}

// 第2、3、4发需要升降台升降
bool AutoLaunchSystem::NeedPlatformMove(void) const {
    return (launchedCount >= 1);
}

/*  ===================== 主状态机分发 =====================  */

void AutoLaunchSystem::StartAutoLaunch(int mode)
{
    // 判断是否符合触发新发射流程的条件
    HandleLaunchTrigger(mode);//根据当前 launchState 执行不同的状态逻辑

    switch (launchState) {
        case AIM_RESET:
            ServoControl.SetAngle(3, 140.0f);
            HandleAimReset();
            break;

        case RELOAD_AIM:
        {
            ServoControl.SetAngle(3, 140.0f);

            // --------- 瞄准轨：每个 tick 都持续运行 ---------
            VisionControl();

            // --------- 装填轨：子状态机独立推进 ---------
            HandleReloadSubMachine();

            // --------- 汇合判断：两条轨道都完成 → 进入 RELEASE ---------
            bool aimReady = (VisionUartReceive.DetectFlag == 3 && angleReached && hasAimedThisDart);

            // [调试开关] 如果想跳过视觉瞄准直接发射，取消下面这一行的注释：
             //aimReady = true; 

            if (reloadReady && aimReady) {
                launchState = RELEASE;
                launchTime  = SystemTick;
            }
            break;
        }

        case RELEASE:
            // 一进入本状态，立刻开启发射偏角 (150.0f)
            ServoControl.SetAngle(3, 180.0f);

            // 等待 500ms 确保机械结构完全释放
            if (SystemTick - launchTime > 500) {
                // 发射动作完成后，复位到 待机/锁定状态 (175.0f)
                ServoControl.SetAngle(3, 140.0f);
                launchState = DONE;
            }
            break;

        case DONE:
            launchedCount++;
            if (launchedCount >= 4) {
                // 4发全部打完
                StopLoadMotors();
                StopLiftMotors();
                launchState = FINISH;
            } else {
                // 还有下一发, 先更新序号再进入瞄准（视觉需要在瞄准前知道是第几发）
                currentDartIndex = launchedCount + 1;  // launchedCount=1→第2发, =2→第3发, =3→第4发
                hasAimedThisDart = false;
                launchState = AIM_RESET;
            }
            break;

        case FINISH:
            // 在完成发射后，务必把发射台运动到上方（到达上方限位后停止）
            Motors[LEFT_LIFT].Target  = LIFT_SPEED_UP;
            Motors[RIGHT_LIFT].Target = -LIFT_SPEED_UP;
            
            // 使用 DartBot 感知接口判断升降台高位限位
            if (!DartBot.PlatHighL()) {  
                StopLiftMotors();
            }

            // 复位推镖舵机 (4号) 和 释放舵机 (3号)
            ServoControl.SetAngle(4, 0.0f);
            ServoControl.SetAngle(3, 140.0f);
            break;

        case IDLE:
        default:
            // 在空闲时保持复位保护
            ServoControl.SetAngle(4, 0.0f);
            ServoControl.SetAngle(3, 140.0f);
            break;
    }

    CheckStateReset(mode);
}

// -------------------------------------------------------------------------
// 内部拆分子状态机
// -------------------------------------------------------------------------

void AutoLaunchSystem::HandleAimReset()
{
    // 在开始这发飞镖的新一轮瞄准前，重置自瞄内部的所有标志位
    targetLocked     = false;
    hasCounted       = false;
    hasAimedThisDart = false;

    // --- 稳健重置逻辑 (resetStage 分段执行) ---
    // 协议规定：aim_finish_flag 归 0 即解除视觉锁定、回到待命状态
    if (resetStage == 0) {
        VisionUartSend.AimingFinishCount = 0;
        launchTime = SystemTick;
        resetStage = 1; // 进入归零等待阶段
    } else if (resetStage == 1) {
        // 归零等待阶段：确保上位机看到 0 并重置逻辑后再进入自瞄
        if (SystemTick - launchTime >= 100) {
            launchState = RELOAD_AIM;
            launchTime  = SystemTick; // 重置给下一阶段使用
            resetStage  = 0;          // 重置子状态机

            // 初始化进入物理装填轨道
            reloadSubState = PLATFORM_UP;
            reloadReady    = false;
            reloadTime     = SystemTick;
        }
    }
}

void AutoLaunchSystem::HandleReloadSubMachine()
{
    switch (reloadSubState) {

        // --- 升降台上升 (仅第2、3、4发，上一发释放后升降台需要回到顶部) ---
        case PLATFORM_UP:
            if (!NeedPlatformMove()) {
                reloadSubState = SLIDE_DART;
                break;
            }
            Motors[LEFT_LIFT].Target  = LIFT_SPEED_UP;
            Motors[RIGHT_LIFT].Target = -LIFT_SPEED_UP;
            
            // 到达向上限位后停止 
            if (!DartBot.PlatHighL()) {  
                StopLiftMotors();
                reloadSubState = SLIDE_DART;
                reloadTime = SystemTick; 
            }
            break;

        // --- 推镖滑落 (仅第3、4发) ---
        case SLIDE_DART:
            if (!NeedSlideDart()) {
                reloadSubState = DOWN_PARALLEL;
                reloadTime = SystemTick;
                break;
            }
            
            if (SystemTick - reloadTime < 400) {
                ServoControl.SetAngle(4, 25.0f);
            } else {
                ServoControl.SetAngle(4, 0.0f);
            }
            
            // 给舵机 500ms 的物理滑落时间
            if (SystemTick - reloadTime > 500) {
                ServoControl.SetAngle(4, 0.0f);
                reloadSubState = DOWN_PARALLEL;
                reloadTime = SystemTick;
            }
            break;

        // --- 并行下行 (升降台下降 + 上膛下行) ---
        case DOWN_PARALLEL:
        {
            // 升降台下行轨道
            bool platformDone = true;
            if (NeedPlatformMove()) {
                // 原逻辑验证两个底端限位是否触发
                if (!(!DartBot.PlatLowR() && !DartBot.PlatLowL())) {
                    Motors[LEFT_LIFT].Target  = -LIFT_SPEED_DOWN;
                    Motors[RIGHT_LIFT].Target = LIFT_SPEED_DOWN;
                    platformDone = false;
                } else {
                    StopLiftMotors();
                }
            }

            // 上膛下行轨道
            bool reloadDone = false;
            Motors[RIGHT_LOAD].Target = LOAD_SPEED_DOWN;
            if (DartBot.ReloadLow()) { 
                StopLoadMotors();
                reloadDone = true;
            }

            if (platformDone && reloadDone) {
                StopLiftMotors();
                StopLoadMotors();
                reloadSubState = RELOAD_UP;
                reloadTime = SystemTick;
            }
            break;
        }

        // --- 上膛上行 ---
        case RELOAD_UP:
            Motors[RIGHT_LOAD].Target = LOAD_SPEED_UP;
            if (DartBot.ReloadHigh()) { 
                StopLoadMotors();
                if (!NeedPlatformMove()) {
                    reloadSubState = PLATFORM_CHECK; // 第1发特殊检查
                } else {
                    reloadSubState = SUB_DONE;
                }
            }
            break;

        // --- 升降台检查到顶 (第1发专用) ---
        case PLATFORM_CHECK:
            if (!DartBot.PlatHighL()) {  
                StopLiftMotors();
                reloadSubState = SUB_DONE;
            } else {
                Motors[LEFT_LIFT].Target  = LIFT_SPEED_UP;
                Motors[RIGHT_LIFT].Target = -LIFT_SPEED_UP;
            }
            break;

        case SUB_DONE:
            reloadReady = true;
            break;
    }
}

// -------------------------------------------------------------------------
// 外部命令与辅助工具
// -------------------------------------------------------------------------

void AutoLaunchSystem::HandleLaunchTrigger(int mode)
{
    if (mode == DT7CtrlMode && launchedCount == 0 && launchState == IDLE) {
        StartNewLaunch();
    } else if (mode == RfSysMode && DartBot.RefereeState == 2 && launchedCount == 0 && launchState == IDLE) {
        StartNewLaunch();
    } else if (mode == RfSysMode && DartBot.RefereeState == 0 && DT7UartCom.rc.ch3 < 370) {
        VisionControl();
    }
}

void AutoLaunchSystem::StartNewLaunch()
{
    launchedCount    = 0;
    currentDartIndex = 1;          // 第一发飞镖，在进入 AIM_RESET 之前就确定
    resetStage       = 0;            
    hasAimedThisDart = false;  
    reloadReady      = false;       
    reloadSubState   = PLATFORM_UP;
    launchState      = AIM_RESET;
}

void AutoLaunchSystem::CheckStateReset(int mode)
{
    bool shouldReset = ((DT7UartCom.rc.s2 == SW_MID && mode == DT7CtrlMode) ||
                        (DartBot.RefereeState == 0 && mode == RfSysMode)) &&
                       launchState == FINISH;

    if (shouldReset) {
        launchState      = IDLE;
        launchedCount    = 0;
        currentDartIndex = 0;
        resetStage       = 0;
        reloadReady      = false;
        reloadSubState   = PLATFORM_UP;
    }
}

void AutoLaunchSystem::VisionControl()
{
    if (DT7UartCom.rc.ch4 > 1600) {
        VisionUartSend.AimingFinishCount = 0;
    }

    static const uint8_t DETECT_FLAG_TARGET = 2; // 视觉检测到目标并请求瞄准
    static const uint8_t DETECT_FLAG_FIRE   = 3; // 视觉瞄准完成并请求开火

    static uint8_t last_detect_flag = DETECT_FLAG_FIRE; 
    if (VisionUartReceive.DetectFlag == DETECT_FLAG_FIRE && last_detect_flag != DETECT_FLAG_FIRE) {
        current_song = &Song_VisionTargetLocked;
        playState = 1;
        isBuzzerPlaying = false; 
    }
    last_detect_flag = VisionUartReceive.DetectFlag;

    if (VisionUartReceive.DetectFlag != DETECT_FLAG_TARGET && VisionUartReceive.DetectFlag != DETECT_FLAG_FIRE) {
        targetLocked = false;
        hasCounted   = false;
        return;
    }

    if (!targetLocked && VisionUartReceive.DetectFlag == DETECT_FLAG_TARGET) {
        DartBot.YawAngle = VisionUartReceive.Target_Yaw;
        targetLocked   = true;
        angleReached  = false;
        hasCounted     = false;
    }

    if (targetLocked && angleReached && !hasCounted) {
        VisionUartSend.AimingFinishCount++; 
        hasCounted = true; 
        hasAimedThisDart = true; 
    }
}

void AutoLaunchSystem::SyncYawAngle()
{
    if (std::abs(DartBot.YawAngle - SpeedPID_AngleSensorM3508.Current) >= 1)
        angleReached = false;
    else
        angleReached = true;

    SpeedPID_AngleSensorM3508.Target = DartBot.YawAngle;
}