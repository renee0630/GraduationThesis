#include "pi_controller.h"

float i[4] = {0, 0, 0, 0};

PIController::PIController(float kp, float ki, float max_output, float min_output, const int index)
    : kp(kp), ki(ki), max_output(max_output), min_output(min_output), integral(i[index]) {}

float PIController::Compute(const int index, float setpoint, float measured_value, float dt) {
    float error = setpoint - measured_value;
    integral += error * dt;
    if (setpoint == 0) integral = 0;
    else if (ki * integral > 1) integral = 0.99/ki;
    else if (ki * integral < -1) integral = -0.99/ki;
    i[index] = integral;

    float output = kp * error + ki * integral;
    if (output > max_output) output = max_output;
    if (output < min_output) output = min_output;
    return output;
}
