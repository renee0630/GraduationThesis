#include "motor_controller.h"

double WheelRadius = 0.05;
double ROUND = 2 * WheelRadius * 3.14159;
double CONST_FOR_MOTOR[4] = {-ROUND / RES_Ratio, ROUND / RES_Ratio, ROUND / RES_Ratio, ROUND / RES_Ratio};

MotorController::MotorController(const int index, TIM_HandleTypeDef* htim_pwm, uint32_t pwm_channel, TIM_HandleTypeDef* htim_encoder, GPIO_TypeDef* dir_port, uint16_t dir_pin, float kp, float ki, float max_output, float min_output)
    : index(index), pi_controller(kp, ki, max_output, min_output, index), htim_pwm(htim_pwm), pwm_channel(pwm_channel), htim_encoder(htim_encoder), dir_port(dir_port), dir_pin(dir_pin), setpoint(0), last_encoder_count(0) {}

void MotorController::SetSpeed(float speed) {
    setpoint = speed;
}

float MotorController::Update(float dt) {
    Vnow = GetMeasuredSpeed();
    float control_signal = pi_controller.Compute(index, setpoint, Vnow, dt);
    SetPWM(control_signal);
    return Vnow;
}

void MotorController::SetPWM(float value) {
    if (value >= 0) {
        HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);
    }
	__HAL_TIM_SET_COMPARE(htim_pwm, pwm_channel, static_cast<uint32_t>(std::fabs(value* MOTOR_PWM_PULSE)));
}

float MotorController::GetMeasuredSpeed() {
    int32_t encoder_count = __HAL_TIM_GET_COUNTER(htim_encoder);
    continue_CNT += encoder_count;
    __HAL_TIM_SET_COUNTER(htim_encoder, 0); // 复位计数器
    float speed = encoder_count * CONST_FOR_MOTOR[index]; // 根据实际情况调整常数
    return speed;
}

float MotorController::GetVnow() {
	return Vnow;
}
float MotorController::MoveDistance() {
	float distance = continue_CNT * CONST_FOR_MOTOR[index];
	continue_CNT = 0;

	return distance;
}

//float MotorController::ComputeWheelSpeed(float Vx, float Vy, float Omega) {
//
//}


