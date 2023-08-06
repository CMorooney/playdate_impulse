#include "pd_api.h"
#include "constants.h"
#include "input.h"
#include <stdbool.h>
#include <math.h>

static int update(void *userdata);

PlaydateAPI* pd;

int frame = 0;
int last_time;
float delta_time = 0.0f;

int eventHandler(PlaydateAPI* playdate, PDSystemEvent event, uint32_t arg) {
  (void)arg; // silence warning

  if (event == kEventInit) {
    pd = playdate;
    init_buttons(pd);
    pd->display->setRefreshRate(0);
    pd->system->setUpdateCallback(update, playdate);
  }

  return 0;
}

void update_delta_time(void) {
  int current_time = pd->system->getCurrentTimeMilliseconds();
  delta_time = current_time - last_time;
  last_time = current_time;
}

static int update(void* userdata) {
  update_delta_time();

  update_buttons();

  pd->sprite->updateAndDrawSprites();
  frame ++;
  pd->system->drawFPS(0, 0);
  return 1;
}
