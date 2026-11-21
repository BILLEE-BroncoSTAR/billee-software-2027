#include "roboclaw_hardware_interface/limit_switch.hpp"
#include <stdexcept>

namespace roboclaw_hardware_interface{

  LimitSwitch::LimitSwitch(int pin, bool active_high)
    : pin_(pin), active_high_(active_high){
    if (pin_ < 0) 
      throw std::invalid_argument("GPIO Pin Cannot Be Negative!");

    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(pin_, GPIO::IN);
  }

  bool LimitSwitch::is_pressed() const{
    return GPIO::input(pin_) == (active_high_ ? GPIO::HIGH : GPIO::LOW);
  }

}  
