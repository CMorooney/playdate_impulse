#ifndef MATH_HEADER
#define MATH_HEADER

#include "types.h"
#include <stdbool.h>

float clamp(float x, float upper, float lower);

bool vector_equals(Vector a, Vector b);
Vector subtract_vectors(Vector v1, Vector v2);
Vector divide_vector(Vector v, float by);
Vector multiply_vector(Vector v, float by);

Vector normalize_vector(Vector v);
float get_magnitude_squared(Vector v);
float get_magnitude(Vector v);

Vector add_vectors(Vector v1, Vector v2);
float dot_product(Vector v1, Vector v2);
float cross_product_vectors(Vector v1, Vector v2);
Vector cross_product_vector_scalar(Vector v, float s);
Vector cross_product_scalar_vector(float s, Vector v);

float distance_squared(Vector v1, Vector v2);
float distance(Vector v1, Vector v2);

bool is_point_on_line(Vector l1, Vector l2, Vector p);

#endif
