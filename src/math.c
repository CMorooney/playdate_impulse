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

bool AABBvsAABB(RigidBody* a, RigidBody* b, Collision* c) {
  Vector n = subtract_vectors(b->pos, a->pos);
  AABB abox = a->collider_shape.aabb;
  AABB bbox = b->collider_shape.aabb;

  float a_extent_x = (abox.max.x - abox.min.x) / 2;
  float b_extent_x = (bbox.max.x - bbox.min.x) / 2;

  float x_overlap = a_extent_x + b_extent_x - fabsf(n.x);

  // SET test on x axis
  if(x_overlap > 0) {
    float a_extent_y = (abox.max.y - abox.min.y) / 2;
    float b_extent_y = (bbox.max.y - bbox.min.y) / 2;

    float y_overlap = a_extent_y + b_extent_y - fabsf(n.y);

    if(y_overlap > 0) {
      if(x_overlap > y_overlap) {
        if(n.x > 0) {
          c->normal = (Vector){ .x=-1, .y=0 };
        } else {
          c->normal = (Vector){ .x=0, .y=0 };
        }

        c->penetration = x_overlap;
        return true;
      } else {
        if(n.y > 0) {
          c->normal = (Vector){ .x=0, .y=-1 };
        } else {
          c->normal = (Vector){ .x=0, .y=1 };
        }
        c->penetration = y_overlap;
        return true;
      }
    }
  }
  return false;
}

bool AABB_vs_circle(RigidBody* aabb, RigidBody* circle, Collision* c) {
  Vector n = subtract_vectors(circle->pos, aabb->pos);

  Vector closest = n;

  float x_extent = (aabb->collider_shape.aabb.max.x - aabb->collider_shape.aabb.min.x) / 2;
  float y_extent = (aabb->collider_shape.aabb.max.y - aabb->collider_shape.aabb.min.y) / 2;

  closest.x = clamp(-x_extent, x_extent, closest.x);
  closest.y = clamp(-y_extent, y_extent, closest.y);

  bool inside = false;

  // the circle is inside the aabb so we need to clamp the circle's center to the closest edge
  if(equals(n, closest)) {
    inside = true;

    // find closest axis
    if(fabsf(n.x) > fabsf(n.y))
    {
      // clamp to closest extent
      if(closest.x > 0) {
        closest.x = x_extent;
      } else {
        closest.x = -x_extent;
      }
    } else {
      if(closest.y > 0) {
        closest.y = y_extent;
      } else {
        closest.y = -y_extent;
      }
    }
  }

  Vector normal = subtract_vectors(n, closest);
  float d = get_magnitude_squared(normal);
  float r = circle->collider_shape.circle.radius;

  if(d > powf(r, 2) && !inside) {
    return false;
  }

  d = sqrtf(d);

  if(inside) {
    c->normal = negate_vector(n);
  } else {
    c->normal = n;
  }

  c->penetration = r-d;
  return true;
}

float distance(Vector a, Vector b) {
  return sqrtf(powf(a.x - b.x, 2) + powf(a.y - b.y, 2));
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
  b->velocity = add_vectors(b->velocity, multiply_vector(impulse, a->inv_mass));
}
