#include "diff_chassis.h"

#include "motor_controller.h"

int direction[4] = {1, -1, 1, 1};

DiffChassis::DiffChassis(const int motor1_index, const int motor2_index)
    : motor1_index(motor1_index), motor2_index(motor2_index) {}


MOTOR_V DiffChassis::Set_GoalCarInfo(CAR_INFO goal) {
	GoalCarInfo.Vx = goal.Vx;
	GoalCarInfo.Vy = goal.Vy;
	GoalCarInfo.Omega = goal.Omega;
	Set_MotorVGoal();

	return GoalMotorV;
}

void DiffChassis::Set_MotorVGoal() {
	GoalMotorV.V1 = (GoalCarInfo.Vx - (GoalCarInfo.Omega * WHEEL_DISTANCE / 2)) * direction[motor1_index]; // motor1 is left motor
	GoalMotorV.V2 = (GoalCarInfo.Vx + (GoalCarInfo.Omega * WHEEL_DISTANCE / 2)) * direction[motor2_index]; // motor2 is right motor
}

void DiffChassis::Update_NowCarInfo(float motor1_Vnow, float motor2_Vnow){
	NowMotorV.V1 = motor1_Vnow;
	NowMotorV.V2 = motor2_Vnow;

	NowCarInfo.Vx = (motor1_Vnow*direction[motor1_index] + motor2_Vnow*direction[motor2_index])/2;
	NowCarInfo.Vy = 0; // differential wheels don't have side move
	NowCarInfo.Omega = (motor2_Vnow*direction[motor1_index] - motor1_Vnow*direction[motor2_index])/WHEEL_DISTANCE;
}

void DiffChassis::Update_NowCarLocation(float temp_dis1, float temp_dis2){
	NowCarLocation.Vx += (temp_dis1*direction[motor1_index] + temp_dis2*direction[motor2_index])/2;
	NowCarLocation.Vy = 0; // differential wheels don't have side move
	NowCarLocation.Omega += (temp_dis2*direction[motor2_index] - temp_dis1*direction[motor1_index])/WHEEL_DISTANCE;
}

MOTOR_V DiffChassis::Get_NowMotorV(){
	return NowMotorV;
}

CAR_INFO DiffChassis::Get_NowCarInfo(){
	return NowCarInfo;
}

CAR_INFO DiffChassis::Get_NowCarLocation(){
	return NowCarLocation;
}
