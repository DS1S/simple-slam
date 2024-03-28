#pragma once

#include <mbed.h>

/**
 * Car hardware interface
 * Author: Makan Dehizadeh
 */

namespace SimpleSlam {

class CarHardwareInterface {
   private:
    PwmOut _wheel1_forward;
    PwmOut _wheel1_backward;
    PwmOut _wheel2_forward;
    PwmOut _wheel2_backward;
    Mutex _mutex;
    bool _should_adjust;
    DigitalOut _led_red;
    DigitalOut _led_blue;

   public:
    CarHardwareInterface();

    void init();
    void begin_processing();
    void check_collision(uint16_t distance_infront);

   private:
    void move_forward();
    void turn_left();
    void turn_right();
    void stop();
};
}  // namespace SimpleSlam
