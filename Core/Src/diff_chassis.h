#ifndef DIFF_CHASSIS_H
#define DIFF_CHASSIS_H

#include "stm32h7xx_hal.h"  // 根据你的具体MCU型号更改此头文件
#include "motor_controller.h"

#define WHEEL_DISTANCE 0.2 // distance between two wheels 20cm = 0.2m

typedef struct {
	double Vx = 0.0;
	double Vy = 0.0;
	double Omega = 0.0;
} CAR_INFO;

class DiffChassis {
public:
    DiffChassis(const int motor1_index, const int motor2_index);

    MOTOR_V Set_GoalCarInfo(CAR_INFO goal);
    void Set_MotorVGoal();
    void Update_NowCarInfo(float motor1_Vnow, float motor2_Vnow);
    void Update_NowCarLocation(float temp_dis1, float temp_dis2);
    MOTOR_V Get_NowMotorV();
    CAR_INFO Get_NowCarInfo();
    CAR_INFO Get_NowCarLocation();


private:

    const int motor1_index;
    const int motor2_index;
    CAR_INFO NowCarInfo;
    CAR_INFO GoalCarInfo;
    CAR_INFO NowCarLocation;
    MOTOR_V GoalMotorV;
    MOTOR_V NowMotorV;

};

#endif // DIFF_CHASSIS_H
