#include <mbed.h>

/**
 * Car hardware interface
 * Author: Makan Dehizadeh
 */

#pragma once

class CarHardwareInterface {
   public:
    CarHardwareInterface();

    void init();
    void move_forward();
    void turn_left();
    void turn_right();
    void stop();

   private:
    PwmOut _wheel1_forward;
    PwmOut _wheel1_backward;
    PwmOut _wheel2_forward;
    PwmOut _wheel2_backward;
};