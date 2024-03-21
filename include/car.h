#include <mbed.h>

/**
 * Car hardware interface
 * Author: Makan Dehizadeh
 */

class CarHardwareInterface {
   public:
    CarHardwareInterface();

    void init();
    void moveForward();
    void turnLeft();
    void turnRight();
    void stop();

   private:
    PwmOut wheel1_forward;
    PwmOut wheel1_backward;
    PwmOut wheel2_forward;
    PwmOut wheel2_backward;
};