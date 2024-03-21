/**
* Car hardware interface
* Author: Makan Dehizadeh
*/

#include "car.h"

CarHardwareInterface::CarHardwareInterface()
    : wheel1_forward(PB_4), wheel1_backward(PB_1),
      wheel2_forward(PA_15), wheel2_backward(PA_2) {
}

void CarHardwareInterface::init() {
    printf("Car::Init\n");

    // Set the duty cycle
    wheel1_forward.write(0.0f);
    wheel1_backward.write(0.0f);
    wheel2_forward.write(0.0f);
    wheel2_backward.write(0.0f);
}

void CarHardwareInterface::moveForward() {
    printf("Car::MoveForward\n");

    // Move the car forward
    wheel1_backward.write(0.0f);
    wheel2_backward.write(0.0f);
    wheel1_forward.write(0.65f);
    wheel2_forward.write(1.0f);
}

void CarHardwareInterface::turnLeft() {
    printf("Car::TurnLeft\n");

    // Turn the car left
    wheel1_backward.write(0.0f);
    wheel2_backward.write(0.0f);
    wheel1_forward.write(0.0f);
    wheel2_forward.write(1.0f);
}

void CarHardwareInterface::turnRight() {
    printf("Car::TurnRight\n");

    // Turn the car right
    wheel1_backward.write(0.0f);
    wheel2_backward.write(0.0f);
    wheel1_forward.write(1.0f);
    wheel2_forward.write(0.0f);
}

void CarHardwareInterface::stop() {
    printf("Car::Stop\n");

    // Stop the car
    wheel1_forward.write(0.0f);
    wheel2_forward.write(0.0f);
    wheel1_backward.write(0.0f);
    wheel2_backward.write(0.0f);
}