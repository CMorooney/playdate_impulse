#ifndef COLLISIONS_HEADER
#define COLLISIONS_HEADER

#include "types.h"
#include "math.h"

void init_collision(PlaydateAPI* pd);

bool AABB_vs_AABB(RigidBody* a, RigidBody* b, Collision* out_c);
bool AABB_vs_circle(RigidBody* aabb, RigidBody* circle, Collision* out_c);
bool circle_vs_circle(RigidBody* a, RigidBody* b, Collision* out_c);
void collide(RigidBody* a, RigidBody* b, Vector normal);

SpriteCollisionResponseType circle_collider_handler(LCDSprite* sprite, LCDSprite* other);
SpriteCollisionResponseType rect_collider_handler(LCDSprite* sprite, LCDSprite* other);

#endif

