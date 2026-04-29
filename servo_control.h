#pragma once
#include <SCServo.h>

HLSCL hlscl;

void servoInit()
{
    Serial1.begin(1000000, SERIAL_8N1, 11, 10);
    hlscl.pSerial = &Serial1;
}

void moveServo(int id, int pos)
{
    hlscl.EnableTorque(id,1);
    hlscl.WritePosEx(id,pos,SERVO_SPEED,SERVO_ACC,SERVO_TORQUE);
}

void disableServo(int id)
{
    hlscl.EnableTorque(id,0);
}