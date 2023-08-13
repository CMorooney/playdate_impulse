#ifndef MATH_HEADER
#define MATH_HEADER

#include "types.h"
#include <stdbool.h>

float clamp(float x, float upper, float lower);

Vector negate_vector(Vector v);
Vector subtract_vector(Vector v, float by);
Vector add_vector(Vector v, float by);
Vector subtract_vectors(Vector v1, Vector v2);
Vector divide_vector(Vector v, float by);
Vector multiply_vector(Vector v, float by);

bool equals(Vector v1, Vector v2);

Vector normalize_vector(Vector v);
float get_magnitude_squared(Vector v);
float get_magnitude(Vector v);

Vector add_vectors(Vector v1, Vector v2);
float dot_product(Vector v1, Vector v2);
Vector mirror_vector(Vector v, Vector normalizedVector);

float distance(Vector v1, Vector v2);

bool AABB_vs_AABB(RigidBody* a, RigidBody* b, Collision* c, PlaydateAPI* pd);
bool AABB_vs_circle(RigidBody* aabb, RigidBody* circle, Collision* c);
bool circle_vs_circle_unoptimized(Circle a, Circle b);
bool circle_vs_circle_optimized(RigidBody* a, RigidBody* b, Collision* c);

void collide(RigidBody* a, RigidBody* b, Vector normal);

#endif
