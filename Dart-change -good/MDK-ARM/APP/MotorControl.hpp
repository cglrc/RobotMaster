#ifndef __MotorControl_Hpp
#define __MotorControl_Hpp

#include "RmMotor.hpp"
#include "PID.hpp"
#include "Filter.hpp"

class Motor {
public:
    RmMotorMeasure_t  Feedback;
    ALG::PID::PID     SpeedPID;
    TD_t              Filter;
    float             Target;
    int               Output;

    Motor() : Target(0), Output(0) {
        memset(&Feedback, 0, sizeof(Feedback));
    }

    void Update() {
        Output = (int)SpeedPID.UpDate(Target, (float)Feedback.RPM);
    }
};

// 电机索引枚举
enum MotorIdx {
    RESERVED = 0,   // 保留位 (0x201)
    LEFT_LIFT,      // 左边升降 M2006 (0x202)
    RIGHT_LOAD,     // 右边上膛 M3508 (0x203)
    LEFT_LOAD,      // 左边上膛 M3508 (0x204)
    YAW,            // Yaw轴 M6020 内环速度环 (0x205)
    RIGHT_LIFT,     // 右边升降 M2006 (0x206)
    SPRING,         // 拉簧调节 M3508 (0x207)
    NUM_MOTORS      // 电机总数
};

extern Motor Motors[NUM_MOTORS];

#endif