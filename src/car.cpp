/**
* Car hardware interface
* Author: Makan Dehizadeh
*/

#include "car.h"

CarHardwareInterface::CarHardwareInterface()
    : _wheel1_forward(PB_4), _wheel1_backward(PB_1),
      _wheel2_forward(PA_15), _wheel2_backward(PA_2) {
}

void CarHardwareInterface::init() {
    printf("Car::Init\n");

    // Set the duty cycle
    _wheel1_forward.write(0.0f);
    _wheel1_backward.write(0.0f);
    _wheel2_forward.write(0.0f);
    _wheel2_backward.write(0.0f);
}

void CarHardwareInterface::move_forward() {
    printf("Car::MoveForward\n");

    // Move the car forward
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);
    _wheel1_forward.write(0.65f);
    _wheel2_forward.write(1.0f);
}

void CarHardwareInterface::turn_left() {
    printf("Car::TurnLeft\n");

    // Turn the car left
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);
    _wheel1_forward.write(0.0f);
    _wheel2_forward.write(1.0f);
}

void CarHardwareInterface::turn_right() {
    printf("Car::TurnRight\n");

    // Turn the car right
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);
    _wheel1_forward.write(1.0f);
    _wheel2_forward.write(0.0f);
}

void CarHardwareInterface::stop() {
    printf("Car::Stop\n");

    // Stop the car
    _wheel1_forward.write(0.0f);
    _wheel2_forward.write(0.0f);
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);
}