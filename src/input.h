#ifndef INPUT_HEADER
#define INPUT_HEADER

#include "pd_api.h"
#include <stdbool.h>

void init_buttons(PlaydateAPI* playdate);
void update_buttons(void);
bool is_button_pressed(PDButtons button);
bool is_button_released(PDButtons button);
bool is_button_held(PDButtons button);

#endif
