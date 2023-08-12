#include "math.h"
#include <math.h>

float clamp(float x, float upper, float lower) {
  return fminf(upper, fmaxf(x, lower));
}

bool equals(Vector v1, Vector v2) {
  return v1.x == v2.x && v1.y == v2.y;
}

Vector negate_vector(Vector v) {
  return (Vector) { .x = -v.x, .y = -v.y };
}

Vector add_vectors(Vector v1, Vector v2) {
  return (Vector) { .x = v1.x + v2.x, .y = v1.y + v2.y};
}

Vector subtract_vector(Vector v, float by) {
  return (Vector) { .x = v.x-by, .y = v.y-by };
}

Vector add_vector(Vector v, float by) {
  return (Vector) { .x = v.x+by, .y = v.y+by };
}

Vector subtract_vectors(Vector v1, Vector v2) {
  return (Vector) { .x = v1.x - v2.x, .y = v1.y - v2.y };
}

Vector divide_vector(Vector v, float by) {
  return (Vector) { .x = v.x / by, .y = v.y / by };
}

Vector multiply_vector(Vector v, float by) {
  return (Vector) { .x = v.x * by, .y = v.y * by };
}

Vector normalize_vector(Vector v) {
  return divide_vector(v, get_magnitude(v));
}

Vector mirror_vector(Vector v, Vector normalizedNormal) {
  float dot = dot_product(v, normalizedNormal);
  Vector reflected_normal = multiply_vector(normalizedNormal, 2 * dot);
  return subtract_vectors(v, reflected_normal);
}

float get_magnitude_squared(Vector v) {
  return powf(v.x, 2) + powf(v.y, 2);
}

float get_magnitude(Vector v) {
  return sqrtf(get_magnitude_squared(v));
}

float dot_product(Vector v1, Vector v2) {
  float sum = 0.0f;
  sum += v1.x * v2.x;
  sum += v1.y * v2.y;
  return sum;
}

bool AABB_vs_AABB(RigidBody* a, RigidBody* b, Collision* c) {
  Vector n = subtract_vectors(b->pos, a->pos);
  AABB abox = a->collider_shape.aabb;
  AABB bbox = b->collider_shape.aabb;

  float a_extent_x = abox.width / 2;
  float b_extent_x = bbox.width / 2;
  float x_overlap = a_extent_x + b_extent_x - fabsf(n.x);

  float a_extent_y = abox.height / 2;
  float b_extent_y = abox.height / 2;
  float y_overlap = a_extent_y + b_extent_y - fabsf(n.y);

  if(y_overlap <=0 && x_overlap <=0) {
    return false;
  }

  if(x_overlap < y_overlap) {
    if(n.x < 0) {
      c->normal = (Vector){ .x=-1, .y=0 };
    } else {
      c->normal = (Vector){ .x=0, .y=0 };
    }

    c->penetration = x_overlap;
    return true;
  } else {
    if(n.y < 0) {
      c->normal = (Vector){ .x=0, .y=-1 };
    } else {
      c->normal = (Vector){ .x=0, .y=1 };
    }
    c->penetration = y_overlap;
    return true;
  }
  return false;
}

// https://learnopengl.com/In-Practice/2D-Game/Collisions/Collision-detection
bool AABB_vs_circle(RigidBody* aabb, RigidBody* circle, Collision* c) {
  // get vector from center of circle to center of aabb
  Vector distance_between_bodies = subtract_vectors(circle->pos, aabb->pos);

  // get the half-size dimensions of the aabb
  float aabb_half_x_extent = aabb->collider_shape.aabb.width / 2;
  float aabb_half_y_extent = aabb->collider_shape.aabb.height / 2;

  // clamp vector between body centers to the bounds of the aabb
  Vector clamped = distance_between_bodies;
  clamped.x = clamp(clamped.x, aabb_half_x_extent, -aabb_half_x_extent);
  clamped.y = clamp(clamped.y, aabb_half_y_extent, -aabb_half_y_extent);

  Vector closest_point_on_aabb = add_vectors(clamped, aabb->pos);
  Vector circle_pos_to_aabb_point = subtract_vectors(circle->pos, closest_point_on_aabb);
  float circle_pos_to_aabb_point_distance_squared = get_magnitude_squared(circle_pos_to_aabb_point);
  float circle_radius = circle->collider_shape.circle.radius;

  bool collided = circle_pos_to_aabb_point_distance_squared > (circle_radius*circle_radius);
  if(!collided)
  {
    return false;
  }

  float m = sqrtf(circle_pos_to_aabb_point_distance_squared);
  c->penetration = circle_radius - m;
  c->normal = negate_vector(divide_vector(circle_pos_to_aabb_point, m));
  return true;
}

bool circle_vs_circle_optimized(RigidBody* a, RigidBody* b, Collision* c) {
  Vector n = subtract_vectors(b->pos, a->pos);
  float normal_magnitude_squared = get_magnitude_squared(n);
  float r = a->collider_shape.circle.radius + b->collider_shape.circle.radius;
  r *= r;

  if(normal_magnitude_squared > r) {
    return false;
  }
  // circles collided!

  // sqrt is expensive, try and not to it unless we need
  // which is why we get the squared magnitude first
  float normal_magnitude = sqrtf(normal_magnitude_squared);

  if(normal_magnitude != 0) {
    c->penetration = r - normal_magnitude;
    //normalize the vector directly since we've already done the needed sqrt calc
    c->normal = divide_vector(n, normal_magnitude);
  }
  // circles are on same position
  else {
    c->penetration = a->collider_shape.circle.radius;
    c->normal = (Vector){ .x=1, .y=0 };
  }

  return true;
}

void collide(RigidBody* a, RigidBody* b, Vector normal) {
  Vector relative_velocity = subtract_vectors(b->velocity, a->velocity);

  // put it in terms of the collision normal direction
  float velocity_along_normal = dot_product(relative_velocity, normal);

  // object are not approaching each other
  if(velocity_along_normal > 0) {
    return;
  }

  // calculate restitution
  float e = fminf(a->restitution, b->restitution);

  // calculate impulse scalar
  float j = -(1 + e) * velocity_along_normal;
  j /= a->inv_mass + b->inv_mass;

  // apply impulse
  Vector impulse = multiply_vector(normal, j);
  a->velocity = subtract_vectors(a->velocity, multiply_vector(impulse, a->inv_mass));
  b->velocity = add_vectors(b->velocity, multiply_vector(impulse, b->inv_mass));
}
