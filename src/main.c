#include "collision.h"
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
LCDBitmap* triangle_bmp;
LCDBitmap* floor_bmp;

LCDSprite* ball30_sprite;
LCDSprite* ball50_sprite;
LCDSprite* rect50x100_sprite;
LCDSprite* rect50x25_sprite;
LCDSprite* triangle_sprite;
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
    init_collision(pd);
    init_buttons(pd);
    pd->display->setRefreshRate(0);
    pd->system->setUpdateCallback(update, playdate);
  }

  return 0;
}

void init_bitmaps(void) {
  const char** outerr = NULL;
  ball30_bmp = pd->graphics->loadBitmap("image/ball30", outerr);
  ball50_bmp = pd->graphics->loadBitmap("image/ball50", outerr);
  rect50x100_bmp = pd->graphics->loadBitmap("image/rect50x100", outerr);
  rect50x25_bmp = pd->graphics->loadBitmap("image/rect50x25", outerr);
  triangle_bmp = pd->graphics->loadBitmap("image/triangle_200x100", outerr);
  floor_bmp = pd->graphics->loadBitmap("image/floor", outerr);
}

void init_sprites(void) {
  bodies = pd->system->realloc(NULL, sizeof(RigidBody)*6);

  Vector ball30_pos = (Vector){ .x = SCREEN_MID_X+80, .y = 30 };
  ball30_sprite = pd->sprite->newSprite();
  pd->sprite->setImage(ball30_sprite, ball30_bmp, kBitmapUnflipped);
  pd->sprite->setCollideRect(ball30_sprite, (PDRect){ .x=-2, .y=-2, .width=34, .height=34 });
  pd->sprite->setCollisionResponseFunction(ball30_sprite, circle_collider_handler);
  pd->sprite->moveTo(ball30_sprite, ball30_pos.x, ball30_pos.y);
  pd->sprite->setTag(ball30_sprite, (uint8_t)circle);
  pd->sprite->addSprite(ball30_sprite);
  RigidBody* ball30_body = &(RigidBody) {
    .pos = ball30_pos,
    .velocity = (Vector) { -0, -0 },
    .inv_mass = .5f,
    .restitution = .2f,
    .g_mult = 1,
    .collider_shape = (union ColliderShape){ .circle = (Circle){ .radius = 15.0f } },
    .collider_shape_type = circle,
    .sprite = ball30_sprite,
    .static_friction = 0.1f,
    .dynamic_friction = 0.12f
  };
  bodies[0] = *ball30_body;
  pd->sprite->setUserdata(ball30_sprite, &bodies[0]);

  Vector ball50_pos = (Vector){ .x=SCREEN_MID_X+65, .y=80 };
  ball50_sprite = pd->sprite->newSprite();
  pd->sprite->setImage(ball50_sprite, ball50_bmp, kBitmapUnflipped);
  pd->sprite->setCollideRect(ball50_sprite, (PDRect){ .x=-2, .y=-2, .width=54, .height=54 });
  pd->sprite->setCollisionResponseFunction(ball50_sprite, circle_collider_handler);
  pd->sprite->moveTo(ball50_sprite, ball50_pos.x, ball50_pos.y);
  pd->sprite->setTag(ball50_sprite, (uint8_t)circle);
  pd->sprite->addSprite(ball50_sprite);
  RigidBody* ball50_body = &(RigidBody) {
    .pos = ball50_pos,
    .velocity = (Vector) { 0, 0 },
    .inv_mass = .2f,
    .restitution = .4f,
    .g_mult = 1,
    .collider_shape = (union ColliderShape){ .circle = (Circle){ .radius = 25.0f } },
    .collider_shape_type = circle,
    .sprite = ball50_sprite,
    .static_friction = 0.0f,
    .dynamic_friction = 0.05f
  };
  bodies[1] = *ball50_body;
  pd->sprite->setUserdata(ball50_sprite, &bodies[1]);

  Vector floor_pos = (Vector) { .x=SCREEN_MID_X, .y=LCD_ROWS-12 };
  floor_sprite = pd->sprite->newSprite();
  pd->sprite->setImage(floor_sprite, floor_bmp, kBitmapUnflipped);
  pd->sprite->setCollideRect(floor_sprite, (PDRect){ .x=0, .y=0, .width=LCD_COLUMNS, .height=25 });
  pd->sprite->setCollisionResponseFunction(floor_sprite, rect_collider_handler);
  pd->sprite->moveTo(floor_sprite, floor_pos.x, floor_pos.y);
  pd->sprite->setTag(floor_sprite, (uint8_t)aabb);
  pd->sprite->addSprite(floor_sprite);
  RigidBody* floor_body = &(RigidBody) {
    .pos = floor_pos,
    .velocity = (Vector){ .x=0, .y=0 },
    .inv_mass = 0,
    .restitution = 0.4f,
    .g_mult = 0,
    .collider_shape = (union ColliderShape) { .aabb = (AABB){ .width=LCD_COLUMNS, .height = 25 } },
    .collider_shape_type = aabb,
    .sprite = floor_sprite,
    .static_friction = 0.0f,
    .dynamic_friction = 0.0f
  };
  bodies[2] = *floor_body;
  pd->sprite->setUserdata(floor_sprite, &bodies[2]);

  Vector rect100_pos = (Vector){ 150, 70 };
  rect50x100_sprite = pd->sprite->newSprite();
  pd->sprite->setImage(rect50x100_sprite, rect50x100_bmp, kBitmapUnflipped);
  pd->sprite->setCollideRect(rect50x100_sprite, (PDRect){ .x=0, .y=0, .width=50, .height=100 });
  pd->sprite->setCollisionResponseFunction(rect50x100_sprite, rect_collider_handler);
  pd->sprite->moveTo(rect50x100_sprite, rect100_pos.x, rect100_pos.y);
  pd->sprite->setTag(rect50x100_sprite, (uint8_t)aabb);
  pd->sprite->addSprite(rect50x100_sprite);
  RigidBody* rect100_body = &(RigidBody) {
    .pos = rect100_pos,
    .velocity = (Vector){ .x=0, .y=0 },
    .inv_mass = .4f,
    .restitution = 0.4f,
    .g_mult = 1,
    .collider_shape = (union ColliderShape) { .aabb = (AABB){ .width=50, .height=100 } },
    .collider_shape_type = aabb,
    .sprite = rect50x100_sprite,
    .static_friction = 0.0f,
    .dynamic_friction = 0.12f
  };
  bodies[3] = *rect100_body;
  pd->sprite->setUserdata(rect50x100_sprite, &bodies[3]);

  Vector rect50_pos = (Vector){ 80, 120 };
  rect50x25_sprite = pd->sprite->newSprite();
  pd->sprite->setImage(rect50x25_sprite, rect50x25_bmp, kBitmapUnflipped);
  pd->sprite->setCollideRect(rect50x25_sprite, (PDRect){ .x=0, .y=0, .width=50, .height=25 });
  pd->sprite->setCollisionResponseFunction(rect50x25_sprite, rect_collider_handler);
  pd->sprite->setTag(rect50x25_sprite, (uint8_t)aabb);
  pd->sprite->moveTo(rect50x25_sprite, rect50_pos.x, rect50_pos.y);
  pd->sprite->addSprite(rect50x25_sprite);
  RigidBody* rect50_body = &(RigidBody) {
    .pos = rect50_pos,
    .velocity = (Vector){ .x=0, .y=0 },
    .inv_mass = .4f,
    .restitution = 0.4f,
    .g_mult = 1,
    .collider_shape = (union ColliderShape) { .aabb = (AABB){ .width=50, .height=25 } },
    .collider_shape_type = aabb,
    .sprite = rect50x25_sprite,
    .static_friction = 0.0f,
    .dynamic_friction = 0.12f
  };
  bodies[4] = *rect50_body;
  pd->sprite->setUserdata(rect50x25_sprite, &bodies[4]);

  Vector triangle_pos = (Vector) { .x = 280, .y = LCD_ROWS-115 };
  triangle_sprite = pd->sprite->newSprite();
  pd->sprite->setImage(triangle_sprite, triangle_bmp, kBitmapUnflipped);
  pd->sprite->setCollideRect(triangle_sprite, (PDRect) { .x=0, .y=0, .width=200, .height=100 });
  pd->sprite->setCollisionResponseFunction(triangle_sprite, triangle_collider_handler);
  pd->sprite->setTag(triangle_sprite, (uint8_t)triangle);
  pd->sprite->moveTo(triangle_sprite, triangle_pos.x, triangle_pos.y);
  pd->sprite->addSprite(triangle_sprite);
  RigidBody* triangle_body = &(RigidBody) {
    .pos = triangle_pos,
    .velocity = (Vector) { .x=0, .y=0 },
    .inv_mass = 0,
    .restitution = .4f,
    .g_mult = 1,
    .collider_shape = (union ColliderShape){
      .triangle = (Triangle) {// points relative to center/sprite pos
        .p1=(Vector){.x = -100, .y = 50 },
        .p2=(Vector){.x = 100, .y = 50 },
        .p3=(Vector){.x = 100, .y = -50 }
      }
    },
    .collider_shape_type = triangle,
    .sprite = triangle_sprite,
    .static_friction = 0.0f,
    .dynamic_friction = 0.12f
  };
  bodies[5] = *triangle_body;
  pd->sprite->setUserdata(triangle_sprite, &bodies[5]);
}

void update_delta_time(void) {
  int current_time = pd->system->getCurrentTimeMilliseconds();
  delta_time = (current_time - last_time)/1000.0f;
  last_time = current_time;
}

void tick(void) {
  Vector g = (Vector){ .x=0, .y=25 };
  for(int i = 0; i < 6; i++) {
    RigidBody* body = &bodies[i];

    // calculate new position by adding velocity to current pos
    Vector vv = add_vectors(body->pos, multiply_vector(body->velocity, delta_time));

    // calculate angular velocity
    /* body->angular_velocity += body->torque * (1.0f / body->moment_of_inertia) * delta_time; */
    /* body->orientation == body->angular_velocity * delta_time; */

    // incorporate gravity
    body->velocity = add_vectors(body->velocity, multiply_vector(multiply_vector(g, body->g_mult), delta_time));

    // broad collision and move
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
