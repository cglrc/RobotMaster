#include "InHpp.hpp"

AutolauchSys AutoLaunch;

void AutolauchSys::HandleReloadSubMachine()
{
    switch (reloadSubState) {

        case PLATFORM_UP:
            if (!NeedPlatformMove()) {
                reloadSubState = SLIDE_DART;
                break;
            }
            Motors[LEFT_LIFT].Target  = LIFT_SPEED_UP;
            Motors[RIGHT_LIFT].Target = -LIFT_SPEED_UP;
            
            if (!DartBot.PlatHighL()) {  
                StopLiftMotors();
                reloadSubState = SLIDE_DART;
                reloadTime = SystemTick; 
            }
            break;

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
            
            if (SystemTick - reloadTime > 500) {
                ServoControl.SetAngle(4, 0.0f);
                reloadSubState = DOWN_PARALLEL;
                reloadTime = SystemTick;
            }
            break;

        case DOWN_PARALLEL:
        {
            bool platformDone = true;
            if (NeedPlatformMove()) {
                if (!(!DartBot.PlatLowR() && !DartBot.PlatLowL())) {
                    Motors[LEFT_LIFT].Target  = -LIFT_SPEED_DOWN;
                    Motors[RIGHT_LIFT].Target = LIFT_SPEED_DOWN;
                    platformDone = false;
                } else {
                    StopLiftMotors();
                }
            }

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

        case RELOAD_UP:
            Motors[RIGHT_LOAD].Target = LOAD_SPEED_UP;
            if (DartBot.ReloadHigh()) { 
                StopLoadMotors();
                if (!NeedPlatformMove()) {
                    reloadSubState = PLATFORM_CHECK;
                } else {
                    reloadSubState = SUB_DONE;
                }
            }
            break;

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

void AutolauchSys::HandleLaunchTrigger(int mode)
{
    if (mode == DT7CtrlMode && launchedCount == 0 && launchState == IDLE) {
        StartNewLaunch();
    } else if (mode == RfSysMode && DartBot.RefereeState == 2 && launchedCount == 0 && launchState == IDLE) {
        StartNewLaunch();
    } else if (mode == RfSysMode && DartBot.RefereeState == 0 && DT7UartCom.rc.ch3 < 370) {
        VisionControl();
    }
}

void AutolauchSys::StartNewLaunch()
{
    launchedCount    = 0;
    currentDartIndex = 1;
    resetStage       = 0;            
    hasAimedThisDart = false;  
    reloadReady      = false;       
    reloadSubState   = PLATFORM_UP;
    launchState      = AIM_RESET;
}

void AutolauchSys::CheckStateReset(int mode)
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

void AutolauchSys::VisionControl()
{
    if (DT7UartCom.rc.ch4 > 1600) {
        VisionUartSend.AimingFinishCount = 0;
    }

    static const uint8_t DETECT_FLAG_TARGET = 2;
    static const uint8_t DETECT_FLAG_FIRE   = 3;

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

void AutolauchSys::SyncYawAngle()
{
    if (std::abs(DartBot.YawAngle - SpeedPID_AngleSensorM3508.Current) >= 1)
        angleReached = false;
    else
        angleReached = true;

    SpeedPID_AngleSensorM3508.Target = DartBot.YawAngle;
}

void AutolauchSys::StartAutoLaunch(int mode)
{
    HandleLaunchTrigger(mode);

    switch (launchState) {
        case AIM_RESET:
            ServoControl.SetAngle(3, 140.0f);
            HandleAimReset();
            break;

        case RELOAD_AIM:
        {
            ServoControl.SetAngle(3, 140.0f);

            VisionControl();

            HandleReloadSubMachine();

            bool aimReady = (VisionUartReceive.DetectFlag == 3 && angleReached && hasAimedThisDart);
           
             //aimReady = true; 
            if (reloadReady && aimReady) {
                launchState = RELEASE;
                launchTime  = SystemTick;
            }
            break;
        }

        case RELEASE:
            ServoControl.SetAngle(3, 180.0f);

            if (SystemTick - launchTime > 500) {
                ServoControl.SetAngle(3, 140.0f);
                launchState = DONE;
            }
            break;

        case DONE:
            launchedCount++;
            if (launchedCount >= 4) {
                StopLoadMotors();
                StopLiftMotors();
                launchState = FINISH;
            } else {
                currentDartIndex = launchedCount + 1;
                hasAimedThisDart = false;
                launchState = AIM_RESET;
            }
            break;

        case FINISH:
            Motors[LEFT_LIFT].Target  = LIFT_SPEED_UP;
            Motors[RIGHT_LIFT].Target = -LIFT_SPEED_UP;
            
            if (!DartBot.PlatHighL()) {  
                StopLiftMotors();
            }

            ServoControl.SetAngle(4, 0.0f);
            ServoControl.SetAngle(3, 140.0f);
            break;

        case IDLE:
        default:
            ServoControl.SetAngle(4, 0.0f);
            ServoControl.SetAngle(3, 140.0f);
            break;
    }

    CheckStateReset(mode);
}

void AutolauchSys::HandleAimReset()
{
    targetLocked     = false;
    hasCounted       = false;
    hasAimedThisDart = false;

    if (resetStage == 0) {
        if (VisionUartSend.AimingFinishCount == 0) {
            VisionUartSend.AimingFinishCount = 250;
            launchTime = SystemTick;
            resetStage = 1;
        } else {
            VisionUartSend.AimingFinishCount = 0;
            launchTime = SystemTick;
            resetStage = 2;
        }
    } else if (resetStage == 1) {
        if (SystemTick - launchTime >= 100 ) {
            VisionUartSend.AimingFinishCount = 0;
            launchTime = SystemTick;
            resetStage = 2;
        }
    } else if (resetStage == 2) {
        if (SystemTick - launchTime >= 100) {
            launchState = RELOAD_AIM;
            launchTime  = SystemTick;
            resetStage  = 0;

            reloadSubState = PLATFORM_UP;
            reloadReady    = false;
            reloadTime     = SystemTick;
        }
    }
}

void AutolauchSys::StopLoadMotors(void) {
    Motors[RIGHT_LOAD].Target = 0;
}

void AutolauchSys::StopLiftMotors(void) {
    Motors[LEFT_LIFT].Target  = 0;
    Motors[RIGHT_LIFT].Target = 0;
}

bool AutolauchSys::NeedSlideDart(void) const {
    return (launchedCount >= 2);
}

bool AutolauchSys::NeedPlatformMove(void) const {
    return (launchedCount >= 1);
}