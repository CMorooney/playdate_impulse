#include "collision.h"

PlaydateAPI* c_pd;

void init_collision(PlaydateAPI* playdate) {
  c_pd = playdate;
}

bool AABB_vs_AABB(RigidBody* a, RigidBody* b, Collision* c) {
  AABB abox = a->collider_shape.aabb;
  AABB bbox = b->collider_shape.aabb;
  LCDRect rect_a = LCDRect_from_AABB(abox, a->pos);
  LCDRect rect_b = LCDRect_from_AABB(bbox, b->pos);

  bool no_collide =  rect_a.right < rect_b.left ||
                     rect_a.left > rect_b.right ||
                     rect_a.bottom < rect_b.top ||
                     rect_a.top > rect_b.bottom;
  if(no_collide) {
    return false;
  }

  float y_overlap = 0.0f;
  Vector y_vector = (Vector){ .x=0, .y=0 };
  // a is above b
  if(a->pos.y < b->pos.y) {
    y_vector = (Vector){ .x=0, .y=-1 };
    y_overlap = rect_a.bottom - rect_b.top;
  }
  // a is below b
  else {
    y_vector = (Vector){ .x=0, .y=1 };
    y_overlap = rect_a.top - rect_b.top;
  }

  float x_overlap = 0.0f;
  Vector x_vector = (Vector){ .x=0, .y=0 };
  // a is to right of b
  if(a->pos.x < b->pos.x) {
    x_vector = (Vector){ .x=-1, .y=0 };
    x_overlap = rect_a.right - rect_b.left;
  }
  // a is to left of b
  else {
    x_vector = (Vector){ .x=1, .y=0 };
    x_overlap = rect_a.left - rect_b.right;
  }

  if(y_overlap < x_overlap) {
    c->normal = y_vector;
    c->penetration = y_overlap;
    return true;
  } else {
    c->normal = x_vector;
    c->penetration = x_overlap;
    return true;
  }
}

// https://learnopengl.com/In-Practice/2D-Game/Collisions/Collision-detection
bool AABB_vs_circle(RigidBody* aabb, RigidBody* circle, Collision* c) {
  // get vector from center of circle to center of aabb
  Vector vector_between_bodies = subtract_vectors(circle->pos, aabb->pos);

  // get the half-size dimensions of the aabb
  float aabb_half_x_extent = aabb->collider_shape.aabb.width / 2.0f;
  float aabb_half_y_extent = aabb->collider_shape.aabb.height / 2.0f;

  // clamp vector between body centers to the bounds of the aabb
  Vector clamped = vector_between_bodies;
  clamped.x = clamp(clamped.x, aabb_half_x_extent, -aabb_half_x_extent);
  clamped.y = clamp(clamped.y, aabb_half_y_extent, -aabb_half_y_extent);

  Vector closest_point_on_aabb = add_vectors(clamped, aabb->pos);
  Vector circle_pos_to_aabb_point = subtract_vectors(closest_point_on_aabb, circle->pos);
  float circle_pos_to_aabb_point_distance_squared = get_magnitude_squared(circle_pos_to_aabb_point);
  float circle_radius = circle->collider_shape.circle.radius;

  bool collided = circle_pos_to_aabb_point_distance_squared < (circle_radius*circle_radius);
  if(!collided)
  {
    return false;
  }

  float m = sqrtf(circle_pos_to_aabb_point_distance_squared);
  c->penetration = circle_radius - m;
  c->normal = divide_vector(circle_pos_to_aabb_point, m);
  return true;
}

bool circle_vs_circle(RigidBody* a, RigidBody* b, Collision* c) {
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
  float inv_mass_sum = a->inv_mass + b-> inv_mass;
  if(inv_mass_sum == 0) {
    a->velocity = (Vector){ .x=0, .y=0 };
    b->velocity = (Vector){ .x=0, .y=0 };
    return;
  }

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
  j /= inv_mass_sum;

  // apply impulse
  Vector impulse = multiply_vector(normal, j);
  a->velocity = subtract_vectors(a->velocity, multiply_vector(impulse, a->inv_mass));
  b->velocity = add_vectors(b->velocity, multiply_vector(impulse, b->inv_mass));

  //BEGIN FRICTION CALC

  // Re-calculate relative velocity after normal impulse
  // is applied (impulse from first article, this code comes
  // directly thereafter in the same resolve function)
  relative_velocity = subtract_vectors(b->velocity, a->velocity);

  // Solve for the tangent vector
  float dot = dot_product(relative_velocity, normal);
  Vector dot_x_normal = multiply_vector(normal, dot);
  if(vector_equals(relative_velocity, dot_x_normal)) {
    return;
  }
  Vector tangent = subtract_vectors(relative_velocity, dot_x_normal);
  tangent = normalize_vector(tangent);

  // Solve for magnitude to apply along the friction vector
  float jt = -dot_product(relative_velocity, tangent);
  jt /= inv_mass_sum;

  // PythagoreanSolve = A^2 + B^2 = C^2, solving for C given A and B
  // Use to approximate mu given friction coefficients of each body
  float mu = sqrtf(powf(a->static_friction, 2) + powf(b->static_friction, 2));

  // Clamp magnitude of friction and create impulse vector
  Vector friction_impulse = (Vector){.x = 0, .y = 0 };
  if(fabsf(jt) < j * mu) {
    friction_impulse = multiply_vector(tangent, jt);
  } else {
    float dynamicFriction = sqrtf(powf(a->dynamic_friction, 2) + powf(b->dynamic_friction, 2));
    friction_impulse = multiply_vector(tangent, -j * dynamicFriction);
  }

  // Apply
  a->velocity = subtract_vectors(a->velocity, multiply_vector(friction_impulse, a->inv_mass));
  b->velocity = add_vectors(b->velocity, multiply_vector(friction_impulse, b->inv_mass));
}

bool AABB_vs_triangle(RigidBody* aabb, RigidBody* triangle, Collision* out_c) {
  Vector half_size = (Vector){ .x=aabb->collider_shape.aabb.width/2, .y=aabb->collider_shape.aabb.height/2 };

  float min_x = aabb->pos.x - half_size.x;
  float max_x = aabb->pos.x + half_size.x;
  float min_y = aabb->pos.y - half_size.y;
  float max_y = aabb->pos.y + half_size.y;

  Triangle t_shape = triangle->collider_shape.triangle;

  // check if triangle points are within rectangle
  // this is not sufficient
  Vector points[3] = { t_shape.p1, t_shape.p2, t_shape.p3 };
  for(int i = 0; i < 3; i++) {
    Vector p = points[i];
    Vector rel = add_vectors(p, triangle->pos);
    if(rel.x > min_x &&
       rel.x < max_x &&
       rel.y > min_y &&
       rel.y < max_y) {
      // todo: actually calculate normal
      out_c->normal = (Vector) { .x=0, .y=-1 };
      return true;
    }
  }

  out_c->normal = (Vector) { .x=0, .y=0 };

  return false;
}

//mostly using the edge detection from
//http://www.phatcode.net/articles.php?id=459
bool circle_vs_triangle(RigidBody* circle, RigidBody* triangle, Collision* out_c) {
  Circle c_shape = circle->collider_shape.circle;
  Triangle t_shape = triangle->collider_shape.triangle;

  // get absolute position of triangle vectors
  Vector vertices[3] = { add_vectors(triangle->pos, t_shape.p1),
                         add_vectors(triangle->pos, t_shape.p2),
                         add_vectors(triangle->pos, t_shape.p3)
  };

  // get absolute edges of triangle
  Vector edges[3][2] = { { vertices[0], vertices[1] },
                         { vertices[1], vertices[2] },
                         { vertices[2], vertices[0] }
  };

  for(int i = 0; i < 3; i++) {
    Vector p1 = edges[i][0];
    Vector p2 = edges[i][1];
    Vector dist = subtract_vectors(p1, p2);

    float len_squared = get_magnitude_squared(dist);

    float dot = (((circle->pos.x - p1.x) * (p2.x - p1.x)) + ((circle->pos.y - p1.y) * (p2.y - p1.y)))/len_squared;

    Vector closest_point = (Vector) {
      .x = p1.x + (dot * (p2.x - p1.x)),
      .y = p1.y + (dot * (p2.y - p1.y))
    };

    // check if closest_point is on the actual line <p1, p2>
    if (is_point_on_line(p1, p2, closest_point)){
      // ok, it's on the line
      Vector circle_to_closest_point_on_line = subtract_vectors(closest_point, circle->pos);
      float final_dist_squared = powf(circle_to_closest_point_on_line.x, 2) + powf(circle_to_closest_point_on_line.y, 2);
      if(final_dist_squared <= powf(c_shape.radius, 2)) {
        Vector cross = p1.x < p2.x ?
                        normalize_vector((Vector){ .x = dist.y, .y = -dist.x })
                        : normalize_vector((Vector){ .x = -dist.y, .y = dist.x });
        out_c->normal = cross;
        return true;
      }
    }
  }
  return false;
}

SpriteCollisionResponseType circle_collider_handler(LCDSprite* sprite, LCDSprite* other) {
  ColliderShapeType collider_type = c_pd->sprite->getTag(other);
  RigidBody* body = c_pd->sprite->getUserdata(sprite);
  RigidBody* other_body = c_pd->sprite->getUserdata(other);

  Collision* c = c_pd->system->realloc(NULL, sizeof(Collision));

  bool collided = false;
  switch(collider_type)
  {
    case circle:
      collided = circle_vs_circle(body, other_body, c);
      break;
    case aabb:
      collided = AABB_vs_circle(other_body, body, c);
      break;
    case triangle:
      collided  = circle_vs_triangle(body, other_body, c);
  }

  if(collided && c != NULL) {
    collide(body, other_body, c->normal);
  }
  // free the collision object
  c_pd->system->realloc(c, 0);
  return kCollisionTypeOverlap;
}

SpriteCollisionResponseType rect_collider_handler(LCDSprite* sprite, LCDSprite* other) {
  ColliderShapeType collider_type = c_pd->sprite->getTag(other);
  RigidBody* body = c_pd->sprite->getUserdata(sprite);
  RigidBody* other_body = c_pd->sprite->getUserdata(other);

  Collision* c = c_pd->system->realloc(NULL, sizeof(Collision));

  bool collided = false;
  switch(collider_type)
  {
    case circle:
      collided = AABB_vs_circle(body, other_body, c);
      break;
    case aabb:
      collided = AABB_vs_AABB(body, other_body, c);
      break;
    case triangle:
      collided = AABB_vs_triangle(body, other_body, c);
      break;
  }

  if(collided && c != NULL) {
    collide(other_body, body, c->normal);
  }
  // free the collision object
  c_pd->system->realloc(c, 0);
  return kCollisionTypeOverlap;
}

SpriteCollisionResponseType triangle_collider_handler(LCDSprite* sprite, LCDSprite* other) {
  ColliderShapeType collider_type = c_pd->sprite->getTag(other);
  RigidBody* body = c_pd->sprite->getUserdata(sprite);
  RigidBody* other_body = c_pd->sprite->getUserdata(other);

  Collision* c = c_pd->system->realloc(NULL, sizeof(Collision));

  bool collided = false;
  switch(collider_type)
  {
    case circle:
      collided = circle_vs_triangle(other_body, body, c);
      break;
    case aabb:
      collided = AABB_vs_triangle(other_body, body, c);
      break;
    case triangle:
      break;
  }

  if(collided && c != NULL) {
    collide(other_body, body, c->normal);
  }
  // free the collision object
  c_pd->system->realloc(c, 0);
  return kCollisionTypeOverlap;
}

