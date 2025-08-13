#pragma once
#include "glm/glm.hpp"

struct AABB {
    AABB() = default;
    AABB(glm::vec3 min, glm::vec3 max);
    AABB(const AABB &box1, const AABB &box2);

    glm::vec3 min, max;
};
