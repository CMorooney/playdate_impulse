#ifndef TYPES_HEADER
#define TYPES_HEADER

#include "pd_api.h"

typedef struct {
  float x; float y;
} Vector;

typedef struct {
  Vector min, max;
} AABB;

typedef struct {
  float radius;
} Circle;

union ColliderShape {
  Circle circle;
  AABB aabb;
};

typedef struct {
  Vector pos;
  Vector velocity;
  float inv_mass;
  float restitution;
  float density;
  float force;
  union ColliderShape collider_shape;
  LCDSprite* sprite;
} RigidBody;

typedef struct {
  RigidBody* a;
  RigidBody* b;
  float penetration;
  Vector normal;
} Collision;

#endif
