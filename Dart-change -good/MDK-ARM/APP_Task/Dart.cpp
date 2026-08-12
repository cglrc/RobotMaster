#include "InHpp.hpp"

DartRobot DartBot;
TickType_t SystemTick;
uint16_t servo_test_count = 0;

void Dart_Task_Entry(void *argument)
{
    TickType_t Lasttick = xTaskGetTickCount();
    DartBot.Init();
    for (;;) {
        DartBot.Run();
        SystemTick = xTaskGetTickCount();
        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
    }
}
void DartRobot::AutoLaunchMode()
{
    if (remote.get_s2() == SW_UP)
        AutoLaunch.StartAutoLaunch(DT7CtrlMode);
    else if (remote.get_s2() == SW_MID)
        AutoLaunch.StartAutoLaunch(RfSysMode);
    else if (remote.get_s2() == SW_DOWN)
        EStop();
}
void DartRobot::Init()
{
    YawAngle = YAW_CENTER;
    CtrlLock = true;
   AutoLaunch.launchState   = 0;
    AutoLaunch.launchedCount = 0;
    can_filter_init();
    
     HAL_UART_Receive_DMA(&DT7UartHandle, DT7UartCom.ReceiveArr, DT7UartReceiveLength);
    HAL_UART_Receive_DMA(&VisionUartHandle, VisionUartReceive.ReceiveArr, VisionUartReceiveLength);
    HAL_UART_Receive_IT(&HuartHandle_RMRefereeSystem, &MyRefereeSys8Data, sizeof(MyRefereeSys8Data));
    ServoControl.Init();

}

void DartRobot::ReadSensors()
{
    // 裁判系统状态抽象判断
    if (Dart_Launch_Opening_Status == 1 || Dart_Remaining_Time == 0)
        RefereeState = 0; // 仓门关闭或者倒计时结束
    else if (Dart_Launch_Opening_Status == 2)
        RefereeState = 1; // 开启中或者关闭中
    else if (Dart_Launch_Opening_Status == 0 && Dart_Remaining_Time > 0)
        RefereeState = 2; // 仓门开启并且倒计时>0的时候


    PlatLL  = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
    PlatRL  = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
    PlatLH  = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
    TensLim = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    ReloadL = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    ReloadHL = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
    ReloadHR = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
}

void DartRobot::CheckControllerConnection()
{
    bool stickNeutral = (std::abs(DT7UartCom.Coord.ch0) + std::abs(DT7UartCom.Coord.ch1) +
                         std::abs(DT7UartCom.Coord.ch2) + std::abs(DT7UartCom.Coord.ch3)) < DEAD * 4;
    bool switchesDown = (remote.get_s1() == SW_DOWN) && (remote.get_s2() == SW_DOWN);

    if (stickNeutral && switchesDown) {
        DT7UartCom.isConnected = true;
    }
}

void DartRobot::Run()
{
    ReadSensors();
    CheckControllerConnection();

    if (remote.get_s1() == SW_MID) {
        if (remote.get_scroll() >= 1600)
            ServoControl.SetAngle(4, 25.0f);
        else
            ServoControl.SetAngle(4, 0.0f);

        if (remote.get_scroll() < 400)
            ServoControl.SetAngle(3, 180.0f);
        else
            ServoControl.SetAngle(3, 140.0f);
    }

    if (!remote.isConnected()) {
        EStop();
        return;
    }

    if (remote.get_s1() == SW_UP && remote.isConnected())
        ReloadCtrl();
    else if (remote.get_s1() == SW_MID && remote.isConnected())
        MotorCtrl();
    else if (remote.get_s1() == SW_DOWN && remote.isConnected())
        AutoLaunchMode();
    Motors[LEFT_LOAD].Target = Motors[RIGHT_LOAD].Target * -1;

    SpeedPID_AngleSensorM3508.Target = DartBot.YawAngle;
    AutoLaunch.SyncYawAngle();
}

void DartRobot::ReloadCtrl()
{
    if (remote.get_s2() == SW_MID &&
        std::abs(remote.get_right_stick_y()) < DEAD &&
        std::abs(remote.get_left_stick_y()) < DEAD)
    {
        CtrlLock = false;
    }

    if (CtrlLock) return;

    float spd = STOP;

    if (remote.get_s2() == SW_UP)
        spd = LOAD_UP;
    else if (remote.get_s2() == SW_DOWN)
        spd = LOAD_DOWN;

    if (spd < 0 && ReloadLow())  spd = STOP;
    if (spd > 0 && ReloadHigh()) spd = STOP;

    Motors[RIGHT_LOAD].Target = spd;
}

void DartRobot::MotorCtrl()
{
    float springSpd = STOP;

    if (remote.get_s2() == SW_MID) {
        CtrlLock = false;
    } else if (!CtrlLock) {
        if (remote.get_s2() == SW_UP)
            springSpd = TensionHit() ? STOP : SPRING_UP;
        else if (remote.get_s2() == SW_DOWN)
            springSpd = SPRING_DN;
    }

    Motors[SPRING].Target = springSpd;

    int stick = remote.get_left_stick_y();
    bool up   = (stick > 0) && PlatHighL();
    bool down = (stick < 0) && (PlatLowR() || PlatLowL());

    if (std::abs(stick) > DEAD && (up || down)) {
        Motors[RIGHT_LIFT].Target = -stick * 2.5f;
        Motors[LEFT_LIFT].Target  =  stick * 2.5f;
    } else {
        Motors[RIGHT_LIFT].Target = STOP;
        Motors[LEFT_LIFT].Target  = STOP;
    }
}

void DartRobot::EStop()
{
    AutoLaunch.launchedCount = 0;
    AutoLaunch.launchState   = 0;
    CtrlLock = true;

    for (int i = 0; i < NUM_MOTORS; i++)
        Motors[i].Target = STOP;

    YawAngle = YAW_CENTER;
    SpeedPID_AngleSensorM3508.Target = SpeedPID_AngleSensorM3508.Current;
}

void DartRobot::AddTarget(int *target, int add, int min, int max)
{
    *target = Clamp(*target + add, min, max);
}

int DartRobot::Clamp(int val, int min, int max)
{
    if (val <= min) return min;
    if (val >= max) return max;
    return val;
}