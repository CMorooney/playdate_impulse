#ifndef MATH_HEADER
#define MATH_HEADER

#include "types.h"

Vector add_vectors(Vector v1, Vector v2);
Vector subtract_vectors(Vector v1, Vector v2);
Vector divide_vector(Vector v, float by);
Vector multiply_vector(Vector v, float by);
Vector invert_vector(Vector v);
Vector normalize_vector(Vector v);
Vector mirror_vector(Vector v, Vector normalizedVector);
float get_magnitude(Vector v);
float dot_product(Vector v1, Vector v2);

#endif
