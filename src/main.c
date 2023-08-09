#include "math.h"
#include "pd_api.h"
#include "constants.h"
#include "input.h"
#include "types.h"
#include <stdbool.h>
#include <math.h>

static int update(void *userdata);

PlaydateAPI* pd;

int frame = 0;
int last_time;
float delta_time = 0.0f;

LCDBitmap* ball30_bmp;
LCDBitmap* ball50_bmp;
LCDBitmap* rect50x100_bmp;
LCDBitmap* rect50x25_bmp;

LCDSprite* ball30_sprite;
LCDSprite* ball50_sprite;
LCDSprite* rect50x100_sprite;
LCDSprite* rect50x25_sprite;

RigidBody* bodies;

static void init_bitmaps(void);
static void init_sprites(void);

int eventHandler(PlaydateAPI* playdate, PDSystemEvent event, uint32_t arg) {
  (void)arg; // silence warning

  if (event == kEventInit) {
    pd = playdate;
    init_bitmaps();
    init_sprites();
    init_buttons(pd);
    pd->display->setRefreshRate(0);
    pd->system->setUpdateCallback(update, playdate);
  }

  return 0;
}

SpriteCollisionResponseType ball_collider_handler(LCDSprite* sprite, LCDSprite* other) {
  Collision* c = pd->system->realloc(NULL, sizeof(Collision));

  RigidBody* b1 = &bodies[0];
  RigidBody* b2 = &bodies[1];

  bool collided = circle_vs_circle_optimized(b1, b2, c);
  if(collided && c != NULL) {
    collide(b1, b2, c->normal);
  }
  // free the collision object
  pd->system->realloc(c, 0);
  return kCollisionTypeOverlap;
}

void init_bitmaps(void) {
  const char** outerr = NULL;
  ball30_bmp = pd->graphics->loadBitmap("image/ball30", outerr);
  ball50_bmp = pd->graphics->loadBitmap("image/ball50", outerr);
  rect50x100_bmp = pd->graphics->loadBitmap("image/rect50x100", outerr);
  rect50x25_bmp = pd->graphics->loadBitmap("image/rect50x25", outerr);
}

void init_sprites(void) {
  bodies = pd->system->realloc(NULL, sizeof(RigidBody)*2);
  Vector ball30_pos = (Vector){ .x = SCREEN_MID_X+80, .y = 180 };
  ball30_sprite = pd->sprite->newSprite();
  pd->sprite->setImage(ball30_sprite, ball30_bmp, kBitmapUnflipped);
  pd->sprite->setCollideRect(ball30_sprite, (PDRect){ .x=-10, .y=-10, .width=50, .height=50 });
  pd->sprite->setCollisionResponseFunction(ball30_sprite, ball_collider_handler);
  pd->sprite->moveTo(ball30_sprite, ball30_pos.x, ball30_pos.y);
  pd->sprite->addSprite(ball30_sprite);
  RigidBody* ball30_body = &(RigidBody) {
    .pos = ball30_pos,
    .velocity = (Vector) { -1, -1 },
    .inv_mass = .5f,
    .restitution = .2f,
    .density = .2f,
    .collider_shape = (union ColliderShape){ .circle = (Circle){ .radius = 15.0f } },
    .sprite = ball30_sprite
  };
  bodies[0] = *ball30_body;

  Vector ball50_pos = (Vector){ .x=SCREEN_MID_X-85, .y=40 };
  ball50_sprite = pd->sprite->newSprite();
  pd->sprite->setImage(ball50_sprite, ball50_bmp, kBitmapUnflipped);
  pd->sprite->setCollideRect(ball50_sprite, (PDRect){ .x=-10, .y=-10, .width=70, .height=70 });
  pd->sprite->setCollisionResponseFunction(ball50_sprite, ball_collider_handler);
  pd->sprite->moveTo(ball50_sprite, ball50_pos.x, ball50_pos.y);
  pd->sprite->addSprite(ball50_sprite);
  RigidBody* ball50_body = &(RigidBody) {
    .pos = ball50_pos,
    .velocity = (Vector) { 1, 1 },
    .inv_mass = .2f,
    .restitution = .4f,
    .density = .3f,
    .collider_shape = (union ColliderShape){ .circle = (Circle){ .radius = 25.0f } },
    .sprite = ball50_sprite
  };
  bodies[1] = *ball50_body;
  return;

  rect50x100_sprite = pd->sprite->newSprite();
  pd->sprite->setImage(rect50x100_sprite, rect50x100_bmp, kBitmapUnflipped);
  pd->sprite->setCollideRect(rect50x100_sprite, (PDRect){ .x=0, .y=0, .width=50, .height=100 });
  pd->sprite->moveTo(rect50x100_sprite, SCREEN_MID_X+30, 80);
  /* pd->sprite->addSprite(rect50x100_sprite); */

  rect50x25_sprite = pd->sprite->newSprite();
  pd->sprite->setImage(rect50x25_sprite, rect50x25_bmp, kBitmapUnflipped);
  pd->sprite->setCollideRect(rect50x25_sprite, (PDRect){ .x=0, .y=0, .width=50, .height=25 });
  pd->sprite->moveTo(rect50x25_sprite, SCREEN_MID_X+90, 20);
  /* pd->sprite->addSprite(rect50x25_sprite); */
}

void update_delta_time(void) {
  int current_time = pd->system->getCurrentTimeMilliseconds();
  delta_time = (current_time - last_time);
  last_time = current_time;
}

LCDSprite* create_ball(void) {
  LCDSprite* sprite = pd->sprite->newSprite();
  pd->sprite->addSprite(sprite);
  return sprite;
}

void tick(void) {
  for(int i = 0; i < 2; i++) {
    RigidBody* body = &bodies[i];
    if(body == NULL || body->sprite == NULL) {
      pd->system->logToConsole("sprite null");
    }
    Vector vv = add_vectors(body->pos, body->velocity);
    float a, b;
    int l;
    pd->sprite->moveWithCollisions(body->sprite, vv.x, vv.y, &a, &b, &l);
    body->pos.x = a;
    body->pos.y = b;
  }
}

static int update(void* userdata) {
  update_delta_time();

  update_buttons();

  tick();

  pd->sprite->updateAndDrawSprites();

  frame ++;
  pd->system->drawFPS(0, 0);
  return 1;
}
