#include "AABB.h"
#include <algorithm>

AABB::AABB(glm::vec3 min, glm::vec3 max)
    : min(min - glm::vec3(1e-5)), max(max + glm::vec3(1e-5))
{}

AABB::AABB(const AABB &box1, const AABB &box2) {
    min.x = std::min(box1.min.x, box2.min.x) - 1e-5;
    min.y = std::min(box1.min.y, box2.min.y) - 1e-5;
    min.z = std::min(box1.min.z, box2.min.z) - 1e-5;

    max.x = std::max(box1.max.x, box2.max.x) + 1e-5;
    max.y = std::max(box1.max.y, box2.max.y) + 1e-5;
    max.z = std::max(box1.max.z, box2.max.z) + 1e-5;
}
