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
LCDBitmap* floor_bmp;

LCDSprite* ball30_sprite;
LCDSprite* ball50_sprite;
LCDSprite* rect50x100_sprite;
LCDSprite* rect50x25_sprite;
LCDSprite* floor_sprite;

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
  ColliderShapeType collider_type = pd->sprite->getTag(other);
  RigidBody* body = pd->sprite->getUserdata(sprite);
  RigidBody* other_body = pd->sprite->getUserdata(other);

  Collision* c = pd->system->realloc(NULL, sizeof(Collision));

  bool collided = false;
  switch(collider_type)
  {
    case circle:
      collided = circle_vs_circle_optimized(body, other_body, c);
      break;
    case aabb:
      collided = AABB_vs_circle(other_body, body, c);
      break;
  }

  if(collided && c != NULL) {
    collide(body, other_body, c->normal);
  }
  // free the collision object
  pd->system->realloc(c, 0);
  return kCollisionTypeOverlap;
}


SpriteCollisionResponseType transparent_collider_handler(LCDSprite* sprite, LCDSprite* other) {
  return kCollisionTypeOverlap;
}

void init_bitmaps(void) {
  const char** outerr = NULL;
  ball30_bmp = pd->graphics->loadBitmap("image/ball30", outerr);
  ball50_bmp = pd->graphics->loadBitmap("image/ball50", outerr);
  rect50x100_bmp = pd->graphics->loadBitmap("image/rect50x100", outerr);
  rect50x25_bmp = pd->graphics->loadBitmap("image/rect50x25", outerr);
  floor_bmp = pd->graphics->loadBitmap("image/floor", outerr);
}

void init_sprites(void) {
  bodies = pd->system->realloc(NULL, sizeof(RigidBody)*3);
  Vector ball30_pos = (Vector){ .x = SCREEN_MID_X+80, .y = 50 };
  ball30_sprite = pd->sprite->newSprite();
  pd->sprite->setImage(ball30_sprite, ball30_bmp, kBitmapUnflipped);
  pd->sprite->setCollideRect(ball30_sprite, (PDRect){ .x=-10, .y=-10, .width=50, .height=50 });
  pd->sprite->setCollisionResponseFunction(ball30_sprite, ball_collider_handler);
  pd->sprite->moveTo(ball30_sprite, ball30_pos.x, ball30_pos.y);
  pd->sprite->addSprite(ball30_sprite);
  RigidBody* ball30_body = &(RigidBody) {
    .pos = ball30_pos,
    .velocity = (Vector) { -0, -0 },
    .inv_mass = .5f,
    .restitution = .2f,
    .density = .2f,
    .collider_shape = (union ColliderShape){ .circle = (Circle){ .radius = 15.0f } },
    .collider_shape_type = circle,
    .sprite = ball30_sprite
  };
  bodies[0] = *ball30_body;
  pd->sprite->setUserdata(ball30_sprite, &bodies[0]);

  Vector ball50_pos = (Vector){ .x=SCREEN_MID_X+65, .y=120 };
  ball50_sprite = pd->sprite->newSprite();
  pd->sprite->setImage(ball50_sprite, ball50_bmp, kBitmapUnflipped);
  pd->sprite->setCollideRect(ball50_sprite, (PDRect){ .x=-10, .y=-10, .width=70, .height=70 });
  pd->sprite->setCollisionResponseFunction(ball50_sprite, ball_collider_handler);
  pd->sprite->moveTo(ball50_sprite, ball50_pos.x, ball50_pos.y);
  pd->sprite->addSprite(ball50_sprite);
  RigidBody* ball50_body = &(RigidBody) {
    .pos = ball50_pos,
    .velocity = (Vector) { 0, 0 },
    .inv_mass = .2f,
    .restitution = .4f,
    .density = .3f,
    .collider_shape = (union ColliderShape){ .circle = (Circle){ .radius = 25.0f } },
    .collider_shape_type = circle,
    .sprite = ball50_sprite
  };
  bodies[1] = *ball50_body;
  pd->sprite->setUserdata(ball50_sprite, &bodies[1]);

  Vector floor_pos = (Vector) { .x=SCREEN_MID_X, .y=LCD_ROWS-12 };
  floor_sprite = pd->sprite->newSprite();
  pd->sprite->setImage(floor_sprite, floor_bmp, kBitmapUnflipped);
  pd->sprite->setCollideRect(floor_sprite, (PDRect){ .x=0, .y=-10, .width=LCD_COLUMNS, .height=25 });
  pd->sprite->setCollisionResponseFunction(floor_sprite, transparent_collider_handler);
  pd->sprite->moveTo(floor_sprite, floor_pos.x, floor_pos.y);
  pd->sprite->setTag(floor_sprite, (uint8_t)aabb);
  pd->sprite->addSprite(floor_sprite);
  RigidBody* floor_body = &(RigidBody) {
    .pos = floor_pos,
    .velocity = (Vector){ .x=0, .y=0 },
    .inv_mass = 0,
    .restitution = 0.4f,
    .density = 0.3f,
    .collider_shape = (union ColliderShape) { .aabb = (AABB){ .min = (Vector){.x=0, .y=LCD_ROWS-24}, .max = (Vector){.x=LCD_COLUMNS, .y=LCD_ROWS} } },
    .collider_shape_type = aabb,
    .sprite = floor_sprite
  };
  bodies[2] = *floor_body;
  pd->sprite->setUserdata(floor_sprite, &bodies[2]);
  return;

  /* rect50x100_sprite = pd->sprite->newSprite(); */
  /* pd->sprite->setImage(rect50x100_sprite, rect50x100_bmp, kBitmapUnflipped); */
  /* pd->sprite->setCollideRect(rect50x100_sprite, (PDRect){ .x=0, .y=0, .width=50, .height=100 }); */
  /* pd->sprite->moveTo(rect50x100_sprite, SCREEN_MID_X+30, 80); */
  /* pd->sprite->addSprite(rect50x100_sprite); */

  /* rect50x25_sprite = pd->sprite->newSprite(); */
  /* pd->sprite->setImage(rect50x25_sprite, rect50x25_bmp, kBitmapUnflipped); */
  /* pd->sprite->setCollideRect(rect50x25_sprite, (PDRect){ .x=0, .y=0, .width=50, .height=25 }); */
  /* pd->sprite->moveTo(rect50x25_sprite, SCREEN_MID_X+90, 20); */
  /* pd->sprite->addSprite(rect50x25_sprite); */
}

void update_delta_time(void) {
  int current_time = pd->system->getCurrentTimeMilliseconds();
  delta_time = (current_time - last_time)/1000.0f;
  last_time = current_time;
}

// do I need this?
bool on_ground(RigidBody* body) {
  float bottom = 400;
  float pos_y = body->pos.y;
  ColliderShapeType t = body->collider_shape_type;
  if(t == circle){
    return pos_y + body->collider_shape.circle.radius > bottom;
  } else if (t == aabb) {
    AABB aabb = body->collider_shape.aabb;
    float h = aabb.max.y - aabb.min.y;
    return pos_y + (h/2) > bottom;
  }
  return false;
}

void tick(void) {
  Vector g = (Vector){ .x=0, .y=25 };
  for(int i = 0; i < 2; i++) {
    RigidBody* body = &bodies[i];
    Vector vv = add_vectors(body->pos, multiply_vector(body->velocity, delta_time));
    body->velocity = add_vectors(body->velocity, multiply_vector(g, delta_time));
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
