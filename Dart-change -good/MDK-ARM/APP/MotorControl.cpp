#include "InHpp.hpp"

Motor Motors[NUM_MOTORS];

// 角度传感器滑动平均滤波
__attribute__((used)) float FilterAngle(float raw) {
    static float buf[100] = {0};
    static int idx = 0;
    static float sum = 0;
    static bool init = false;

    if (!init) {
        for (int i = 0; i < 100; i++) buf[i] = raw;
        sum = raw * 100;
        init = true;
    }

    sum -= buf[idx];
    buf[idx] = raw;
    sum += raw;
    idx = (idx + 1) % 100;

    return sum / 100.0f;
}

// 统一更新所有电机PID
__attribute__((used)) void MotorUpdate()
{
    SpeedPID_AngleSensorM3508.Compute();

    for (int i = 0; i < NUM_MOTORS; i++) {
        if (i == YAW)
            Motors[YAW].Target = -SpeedPID_AngleSensorM3508.getOutput();
        Motors[i].Update();
    }
}

// 初始化所有电机PID参数
static void MotorInit()
{
    Motors[LEFT_LIFT].SpeedPID.setK(14.0f, 0.3f, 0.0f);
    Motors[LEFT_LIFT].SpeedPID.setMax(16000.0f);
    Motors[LEFT_LIFT].SpeedPID.setIntegralLimit(10000.0f);

    Motors[RIGHT_LOAD].SpeedPID.setK(12.0f, 0.1f, 0.0f);
    Motors[RIGHT_LOAD].SpeedPID.setMax(16000.0f);
    Motors[RIGHT_LOAD].SpeedPID.setIntegralLimit(30000.0f);
    Motors[RIGHT_LOAD].Filter.setParams(380.0f, 0.001f);

    Motors[LEFT_LOAD].SpeedPID.setK(12.0f, 0.1f, 0.0f);
    Motors[LEFT_LOAD].SpeedPID.setMax(16000.0f);
    Motors[LEFT_LOAD].SpeedPID.setIntegralLimit(30000.0f);
    Motors[LEFT_LOAD].Filter.setParams(380.0f, 0.001f);

    Motors[YAW].SpeedPID.setK(13.0f, 0.0f, 0.0f);
    Motors[YAW].SpeedPID.setMax(16000.0f);

    Motors[RIGHT_LIFT].SpeedPID.setK(14.0f, 0.3f, 0.0f);
    Motors[RIGHT_LIFT].SpeedPID.setMax(16000.0f);
    Motors[RIGHT_LIFT].SpeedPID.setIntegralLimit(10000.0f);

    Motors[SPRING].SpeedPID.setK(10.0f, 0.7f, 0.0f);
    Motors[SPRING].SpeedPID.setMax(16000.0f);
    Motors[SPRING].SpeedPID.setIntegralLimit(4286.0f);
    Motors[SPRING].Filter.setParams(380.0f, 0.001f);
}

void Motor_Task_Entry(void *argument)
{
    MotorInit();

    TickType_t Lasttick = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));

        taskDISABLE_INTERRUPTS();
        MotorUpdate();
        taskENABLE_INTERRUPTS();

        RmMotorSendCanID0X200Data(&hcan1, Motors[RESERVED].Output, Motors[LEFT_LIFT].Output, Motors[RIGHT_LOAD].Output, Motors[LEFT_LOAD].Output);

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));

        RmMotorSendCanID0X1FFData(&hcan1, Motors[YAW].Output, Motors[RIGHT_LIFT].Output, Motors[SPRING].Output, 0);
    }
}
