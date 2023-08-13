#include "input.h"

PlaydateAPI* i_pd;

PDButtons buttons_pressed;
PDButtons buttons_held;
PDButtons buttons_released;

void init_buttons(PlaydateAPI* playdate) {
  i_pd = playdate;
}

void update_buttons(void) {
  i_pd->system->getButtonState(&buttons_pressed, &buttons_held, &buttons_released);
}

// call this after update_buttons() to make sure you have
// latest button state in memory
bool is_button_pressed(PDButtons button) {
  if(button & buttons_pressed) {
    return true;
  }
  return false;
}

// call this after update_buttons() to make sure you have
// latest button state in memory
bool is_button_released(PDButtons button) {
  if(button & buttons_released) {
    return true;
  }
  return false;
}

// call this after update_buttons() to make sure you have
// latest button state in memory
bool is_button_held(PDButtons button) {
  if(button & buttons_held) {
    return true;
  }
  return false;
}

