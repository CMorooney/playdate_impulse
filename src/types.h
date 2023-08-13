#ifndef TYPES_HEADER
#define TYPES_HEADER

#include "pd_api.h"

typedef struct {
  float x; float y;
} Vector;

typedef struct {
  float width, height;
} AABB;

typedef struct {
  float radius;
} Circle;

union ColliderShape {
  Circle circle;
  AABB aabb;
};

typedef enum {
  circle,
  aabb
} ColliderShapeType;

typedef struct {
  Vector pos;
  Vector velocity;
  float inv_mass;
  float restitution;
  float density;
  float g_mult;
  float force;
  float static_friction;
  float dynamic_friction;
  union ColliderShape collider_shape;
  ColliderShapeType collider_shape_type;
  LCDSprite* sprite;
} RigidBody;

typedef struct {
  RigidBody* a;
  RigidBody* b;
  float penetration;
  Vector normal;
} Collision;

#endif
