#include "types.h"

LCDRect LCDRect_from_AABB(AABB aabb, Vector position) {
  float half_height = aabb.height/2.0f;
  float half_width = aabb.width/2.0f;
  return (LCDRect) {
    .left = position.x - half_width,
    .right = position.x + half_width,
    .top = position.y - half_height,
    .bottom = position.y + half_height
  };
}
