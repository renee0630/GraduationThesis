#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "stm32h7xx_hal.h"  // 根据你的具体MCU型号更改此头文件
#include "pi_controller.h"


// Gearbox -> 26 : 1
// Encoder Resolution : 1024
// Wheel Radius : 0.05 m
// Car Radius : 0.155 m
// Timer for reading encoder CNT : 1k Hz
// 4 * Resolution * GearBox * CountTime(0.001s)
#define RES_Ratio 106.496

#define MOTOR_PWM_PULSE 6400

typedef struct {
	double V1 = 0.0;
	double V2 = 0.0;
	double V3 = 0.0;
	double V4 = 0.0;

} MOTOR_V;

class MotorController {
public:
    MotorController(const int index, TIM_HandleTypeDef* htim_pwm, uint32_t pwm_channel, TIM_HandleTypeDef* htim_encoder, GPIO_TypeDef* dir_port, uint16_t dir_pin, float kp, float ki, float max_output, float min_output);

    void SetSpeed(float speed);
    float Update(float dt);
    float GetVnow();
    float MoveDistance();

private:
    void SetPWM(float value);
    float GetMeasuredSpeed();

    const int index;
    PIController pi_controller;
    TIM_HandleTypeDef* htim_pwm;
    uint32_t pwm_channel;
    TIM_HandleTypeDef* htim_encoder;
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;
    float setpoint;
    int32_t last_encoder_count;
    int32_t continue_CNT = 0;
    float Vnow;
};

#endif // MOTOR_CONTROLLER_H
