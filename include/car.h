/**
* Car hardware interface
* Author: Makan Dehizadeh
*/

#pragma once

#include "mbed.h"

namespace SimpleSlam::Car {

#define WHEEL1_FORWARD PB_4
#define WHEEL1_BACKWARD PB_1
#define WHEEL2_FORWARD PA_15
#define WHEEL2_BACKWARD PA_2

void Init();
void MoveForward();
void TurnLeft();
void TurnRight();
void Stop();

}  // namespace SimpleSlam::Car