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

typedef struct {
  // relative to center of rigidbody!!
  Vector p1;
  Vector p2;
  Vector p3;
} Triangle;

union ColliderShape {
  Circle circle;
  AABB aabb;
  Triangle triangle;
};

typedef enum {
  circle,
  aabb,
  triangle,
} ColliderShapeType;

typedef struct {
  Vector pos;
  Vector velocity;
  float inv_mass;
  float restitution;
  float g_mult;
  float static_friction;
  float dynamic_friction;
  float orientation;
  float angular_velocity;
  float torque;
  float moment_of_inertia;
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

LCDRect LCDRect_from_AABB(AABB aabb, Vector position);

#endif
