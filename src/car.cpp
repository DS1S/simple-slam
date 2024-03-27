/**
 * Car hardware interface
 * Author: Makan Dehizadeh
 */

#include "car.h"

SimpleSlam::CarHardwareInterface::CarHardwareInterface()
    : _wheel1_forward(PB_4),
      _wheel1_backward(PB_1),
      _wheel2_forward(PA_15),
      _wheel2_backward(PA_2),
      _mutex(),
      _should_adjust(false) {}

void SimpleSlam::CarHardwareInterface::init() {
    printf("Car::Init\n");

    // Set the duty cycle
    _wheel1_forward.write(0.0f);
    _wheel2_forward.write(0.0f);
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);
}

void SimpleSlam::CarHardwareInterface::begin_processing() {
    while (true) {
        _mutex.lock();
        if (_should_adjust) {
            _should_adjust = false;
            _mutex.unlock();
            stop();
            ThisThread::sleep_for(1s);

            // Pick a random direction
            if (rand() % 2 == 0) {
                turn_left();
                ThisThread::sleep_for(750ms);
            } else {
                turn_right();
                ThisThread::sleep_for(750ms);
            }
        } else {
            move_forward();
            _mutex.unlock();
        }
        ThisThread::sleep_for(250ms);
    }
}

void SimpleSlam::CarHardwareInterface::check_collision(
    uint16_t distance_infront) {
    bool mutex_accquired = _mutex.trylock();
    if (!mutex_accquired) {
        return;
    }
    // If the distance infront is less than 25 cm then we should turn.
    if (distance_infront < 25) {
        _should_adjust = true;
    }
    _mutex.unlock();
}

void SimpleSlam::CarHardwareInterface::move_forward() {
    // Move the car forward
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);
    _wheel1_forward.write(0.65f);
    _wheel2_forward.write(1.0f);
}

void SimpleSlam::CarHardwareInterface::turn_left() {
    // Turn the car left
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);
    _wheel1_forward.write(0.0f);
    _wheel2_forward.write(1.0f);
}

void SimpleSlam::CarHardwareInterface::turn_right() {
    // Turn the car right
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);
    _wheel1_forward.write(1.0f);
    _wheel2_forward.write(0.0f);
}

void SimpleSlam::CarHardwareInterface::stop() {
    // Stop the car
    _wheel1_forward.write(0.0f);
    _wheel2_forward.write(0.0f);
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);
}