#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

extern float i[4];

class PIController {
public:
    PIController(float kp, float ki, float max_output, float min_output, const int index);

    float Compute(const int index, float setpoint, float measured_value, float dt);

private:
    float kp;
    float ki;
    float max_output;
    float min_output;
    float integral;
};

#endif // PI_CONTROLLER_H
