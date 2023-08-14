#include "math.h"
#include "pd_api.h"
#include <math.h>

float clamp(float x, float upper, float lower) {
  return fminf(upper, fmaxf(x, lower));
}

bool vector_equals(Vector a, Vector b) {
  return a.x == b.x && a.y == b.y;
}

Vector add_vectors(Vector v1, Vector v2) {
  return (Vector) { .x = v1.x + v2.x, .y = v1.y + v2.y};
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

