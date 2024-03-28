/**
 * Car hardware interface
 * Author: Makan Dehizadeh
 */

#include "car.h"

#define DISTANCE_THRESHOLD 30

SimpleSlam::CarHardwareInterface::CarHardwareInterface()
    : _wheel1_forward(PB_1),
      _wheel1_backward(PB_4),
      _wheel2_forward(PA_15),
      _wheel2_backward(PA_2),
      _mutex(),
      _should_adjust(false),
      _led_red(PB_9),
      _led_blue(PB_8) {}

void SimpleSlam::CarHardwareInterface::init() {
    printf("Car::Init\n");

    // Set the duty cycle
    _wheel1_forward.write(0.0f);
    _wheel2_forward.write(0.0f);
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);

    // Set the LED
    _led_red.write(1);
    _led_blue.write(0);
}

void SimpleSlam::CarHardwareInterface::begin_processing() {
    while (true) {
        _mutex.lock();
        if (_should_adjust) {
            stop();
            ThisThread::sleep_for(1s);

            // Pick a random direction
            turn_left();
            ThisThread::sleep_for(880ms);

            _should_adjust = false;
            _led_blue.write(0);
            _mutex.unlock();

            stop();
            ThisThread::sleep_for(3500ms);
        } else {
            move_forward();
            _mutex.unlock();
        }

        ThisThread::sleep_for(150ms);
    }
}

void SimpleSlam::CarHardwareInterface::check_collision(
    uint16_t distance_infront) {
    bool mutex_accquired = _mutex.trylock();
    if (!mutex_accquired) {
        return;
    }
    // If the distance infront is less than 25 cm then we should turn.
    if (distance_infront <= DISTANCE_THRESHOLD) {
        _should_adjust = true;
        _led_blue.write(1);
    }
    _mutex.unlock();
}

void SimpleSlam::CarHardwareInterface::move_forward() {
    // Move the car forward
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);
    _wheel1_forward.write(0.72f);
    _wheel2_forward.write(1.0f);

    _led_red.write(0);
}

void SimpleSlam::CarHardwareInterface::turn_left() {
    // Turn the car left
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);
    _wheel1_forward.write(0.0f);
    _wheel2_forward.write(1.0f);

    _led_red.write(0);
}

void SimpleSlam::CarHardwareInterface::turn_right() {
    // Turn the car right
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);
    _wheel1_forward.write(1.0f);
    _wheel2_forward.write(0.0f);

    _led_red.write(0);
}

void SimpleSlam::CarHardwareInterface::stop() {
    // Stop the car
    _wheel1_forward.write(0.0f);
    _wheel2_forward.write(0.0f);
    _wheel1_backward.write(0.0f);
    _wheel2_backward.write(0.0f);

    _led_red.write(1);
}