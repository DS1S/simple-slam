/**
* Car hardware interface
* Author: Makan Dehizadeh
*/

#include "car.h"

PwmOut wheel1_forward(WHEEL1_BACKWARD);
PwmOut wheel1_backward(WHEEL1_FORWARD);
PwmOut wheel2_forward(WHEEL2_FORWARD);
PwmOut wheel2_backward(WHEEL2_BACKWARD);

void SimpleSlam::Car::Init() {
    printf("Car::Init\n");

    // Set the duty cycle
    wheel1_forward.write(0.0f);
    wheel1_backward.write(0.0f);
    wheel2_forward.write(0.0f);
    wheel2_backward.write(0.0f);
}

void SimpleSlam::Car::MoveForward() {
    printf("Car::MoveForward\n");
    // Move the car forward
    wheel1_backward.write(0.0f);
    wheel2_backward.write(0.0f);
    wheel1_forward.write(0.65f);
    wheel2_forward.write(1.0f);
}

void SimpleSlam::Car::TurnLeft() {
    printf("Car::TurnLeft\n");
    // Turn the car left
    wheel1_backward.write(0.0f);
    wheel2_backward.write(0.0f);
    wheel1_forward.write(0.0f);
    wheel2_forward.write(1.0f);
}

void SimpleSlam::Car::TurnRight() {
    printf("Car::TurnRight\n");
    // Turn the car right
    wheel1_backward.write(0.0f);
    wheel2_backward.write(0.0f);
    wheel1_forward.write(1.0f);
    wheel2_forward.write(0.0f);
}

void SimpleSlam::Car::Stop() {
    printf("Car::MoveBackward\n");
    // Move the car backward
    wheel1_forward.write(0.0f);
    wheel2_forward.write(0.0f);
    wheel1_backward.write(0.0f);
    wheel2_backward.write(0.0f);
}