#pragma once
#include "servo_control.h"

void brake_all()
{
    moveServo(SERVO_BACK_ID,POS_BACK_BRAKE);
    moveServo(SERVO_FRONT_ID,POS_FRONT_BRAKE);
}

void release_all()
{
    moveServo(SERVO_BACK_ID,POS_BACK_RELEASE);
    moveServo(SERVO_FRONT_ID,POS_FRONT_RELEASE);
}

void brake_front()
{
    moveServo(SERVO_FRONT_ID,POS_FRONT_BRAKE);
}

void release_front()
{
    moveServo(SERVO_FRONT_ID,POS_FRONT_RELEASE);
}

void brake_back()
{
    moveServo(SERVO_BACK_ID,POS_BACK_BRAKE);
}

void release_back()
{
    moveServo(SERVO_BACK_ID,POS_BACK_RELEASE);
}