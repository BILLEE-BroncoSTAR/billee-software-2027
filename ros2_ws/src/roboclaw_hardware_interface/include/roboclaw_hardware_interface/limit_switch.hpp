#ifndef ROBOCLAW_HARDWARE_INTERFACE__LIMIT_SWITCH_HPP_
#define ROBOCLAW_HARDWARE_INTERFACE__LIMIT_SWITCH_HPP_

#include <JetsonGPIO.h>

namespace roboclaw_hardware_interface
{

  class LimitSwitch
  {
  public:
    LimitSwitch(int pin, bool active_high = true);

    bool is_pressed() const;

    int get_pin() const {return pin_;}

  private:
    int pin_;
    bool active_high_;
  };

}

#endif  // ROBOCLAW_HARDWARE_INTERFACE__LIMIT_SWITCH_HPP_
