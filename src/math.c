#include "math.h"
#include "pd_api.h"
#include <math.h>

float clamp(float x, float upper, float lower) {
  return fminf(upper, fmaxf(x, lower));
}

float cross_prodcut_vectors(Vector v1, Vector v2) {
  return v1.x * v2.y - v1.y * v2.x;
}

Vector cross_product_vector_scalar(Vector v, float s) {
  return (Vector) { .x = s * v.y, .y = -s * v.x };
}

Vector cross_product_scalar_vector(float s, Vector v) {
  return (Vector) { .x = -s * v.y, .y = s * v.x };
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

float distance_squared(Vector v1, Vector v2) {
  return powf((v2.x - v1.x), 2) + powf(v2.y - v1.y, 2);
}

float distance(Vector v1, Vector v2) {
  return sqrtf(distance_squared(v1, v2));
}

//https://stackoverflow.com/a/68116732/3102451
bool is_point_on_line(Vector l1, Vector l2, Vector p) {
  float dx = (p.x - l1.x) / (l2.x - l1.x);
  float dy = (p.y - l1.y) / (l2.y - l1.y);
  bool onLine = dx == dy;

  // Check on or within x and y bounds
  bool betweenX = 0 <= dx && dx <= 1;
  bool betweenY = 0 <= dy && dy <= 1;

  return onLine && betweenX && betweenY;
}
