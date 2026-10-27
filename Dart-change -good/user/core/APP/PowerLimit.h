#ifndef CRT_CHASSIS_H
#define CRT_CHASSIS_H

#include "alg_slope.h"
#include "dvc_sampler.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"

enum Enum_Chassis_Control_Type
{
    Chassis_Control_Type_DISABLE = 0,
    Chassis_Control_Type_ABSOLUTE,
};

class Class_Tricycle_Chassis
{
public:
    Class_PID PID_Power_Limit_Steer;
    Class_PID PID_Power_Limit_Wheel;

    Class_Slope Slope_Velocity_X;
    Class_Slope Slope_Velocity_Y;
    Class_Slope Slope_Omega;

    Class_Referee *Referee;

    Class_Sampler Sampler;

    Class_DJI_Motor_C620 Motor_Wheel[3];
    Class_DJI_Motor_GM6020 Motor_Steer[3];

    void Init(float __Velocity_X_Max = 2.0f, float __Velocity_Y_Max = 2.0f, float __Omega_Max = 4.0f, float __Steer_Power_Ratio = 0.5);

    inline Enum_Chassis_Control_Type Get_Chassis_Control_Type();
    inline float Get_Velocity_X_Max();
    inline float Get_Velocity_Y_Max();
    inline float Get_Omega_Max();
    inline float Get_Now_Power();
    inline float Get_Now_Steer_Power();
    inline float Get_Target_Steer_Power();
    inline float Get_Now_Wheel_Power();
    inline float Get_Target_Wheel_Power();
    inline float Get_Target_Velocity_X();
    inline float Get_Target_Velocity_Y();
    inline float Get_Target_Omega();

    inline void Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type);
    inline void Set_Target_Velocity_X(float __Target_Velocity_X);
    inline void Set_Target_Velocity_Y(float __Target_Velocity_Y);
    inline void Set_Target_Omega(float __Target_Omega);
    inline void Set_Now_Velocity_X(float __Now_Velocity_X);
    inline void Set_Now_Velocity_Y(float __Now_Velocity_Y);
    inline void Set_Now_Omega(float __Now_Omega);

    void TIM_Calculate_PeriodElapsedCallback();

protected:

    float Velocity_X_Max;
    float Velocity_Y_Max;
    float Omega_Max;
    float Steer_Power_Ratio = 0.5f;

    float Steer_Max_Output = 30000.0f;
    float Wheel_Max_Output = 16384.0f;

    float Target_Steer_Angle[3];
    float Target_Wheel_Omega[3];

    float Now_Power = 0.0f;
    float Now_Steer_Power = 0.0f;
    float Target_Steer_Power = 0.0f;
    float Now_Wheel_Power = 0.0f;
    float Target_Wheel_Power = 0.0f;

    Enum_Chassis_Control_Type Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    float Target_Velocity_X = 0.0f;
    float Target_Velocity_Y = 0.0f;
    float Target_Omega = 0.0f;
    float Now_Velocity_X = 0.0f;
    float Now_Velocity_Y = 0.0f;
    float Now_Omega = 0.0f;

    void Power_Limit_Steer();
    void Power_Limit_Wheel();
    void Power_Limit();
    void Speed_Resolution();
};

const float WHEEL_RADIUS = 0.0520f;

const float WHEEL_TO_CORE_DISTANCE[3] = {0.23724f, 0.21224f, 0.21224f};

const float FRONT_CENTER_TO_CORE_DISTANCE = 0.11862f;

const float FRONT_TO_REAR_DISTANCE = WHEEL_TO_CORE_DISTANCE[0] + FRONT_CENTER_TO_CORE_DISTANCE;

const float FRONT_TO_FRONT_CENTER_DISTANCE = 0.176f;

const float WHEEL_AZIMUTH[3] = {0.0f, atan2f(-FRONT_TO_FRONT_CENTER_DISTANCE, -FRONT_CENTER_TO_CORE_DISTANCE), atan2f(FRONT_TO_FRONT_CENTER_DISTANCE, -FRONT_CENTER_TO_CORE_DISTANCE)};

Enum_Chassis_Control_Type Class_Tricycle_Chassis::Get_Chassis_Control_Type()
{
    return (Chassis_Control_Type);
}

float Class_Tricycle_Chassis::Get_Velocity_X_Max()
{
    return (Velocity_X_Max);
}

float Class_Tricycle_Chassis::Get_Velocity_Y_Max()
{
    return (Velocity_Y_Max);
}

float Class_Tricycle_Chassis::Get_Omega_Max()
{
    return (Omega_Max);
}

float Class_Tricycle_Chassis::Get_Target_Velocity_X()
{
    return (Target_Velocity_X);
}

float Class_Tricycle_Chassis::Get_Target_Velocity_Y()
{
    return (Target_Velocity_Y);
}

float Class_Tricycle_Chassis::Get_Target_Omega()
{
    return (Target_Omega);
}

float Class_Tricycle_Chassis::Get_Now_Power()
{
    return (Now_Power);
}

float Class_Tricycle_Chassis::Get_Now_Steer_Power()
{
    return (Now_Steer_Power);
}

float Class_Tricycle_Chassis::Get_Target_Steer_Power()
{
    return (Target_Steer_Power);
}

float Class_Tricycle_Chassis::Get_Now_Wheel_Power()
{
    return (Now_Wheel_Power);
}

float Class_Tricycle_Chassis::Get_Target_Wheel_Power()
{
    return (Target_Wheel_Power);
}

void Class_Tricycle_Chassis::Set_Chassis_Control_Type(Enum_Chassis_Control_Type __Chassis_Control_Type)
{
    Chassis_Control_Type = __Chassis_Control_Type;
}

void Class_Tricycle_Chassis::Set_Target_Velocity_X(float __Target_Velocity_X)
{
    Target_Velocity_X = __Target_Velocity_X;
}

void Class_Tricycle_Chassis::Set_Target_Velocity_Y(float __Target_Velocity_Y)
{
    Target_Velocity_Y = __Target_Velocity_Y;
}

void Class_Tricycle_Chassis::Set_Target_Omega(float __Target_Omega)
{
    Target_Omega = __Target_Omega;
}

void Class_Tricycle_Chassis::Set_Now_Velocity_X(float __Now_Velocity_X)
{
    Now_Velocity_X = __Now_Velocity_X;
}

void Class_Tricycle_Chassis::Set_Now_Velocity_Y(float __Now_Velocity_Y)
{
    Now_Velocity_Y = __Now_Velocity_Y;
}

void Class_Tricycle_Chassis::Set_Now_Omega(float __Now_Omega)
{
    Now_Omega = __Now_Omega;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
