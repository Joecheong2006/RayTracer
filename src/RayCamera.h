#pragma once

#include "glm/glm.hpp"
#include "util.h"

class TraceableObject;
class Material;

struct RayCamera {
    glm::vec3 position = { 0, 0, 0 };
    f32 yaw = 90, pitch = 0, fov = 45;

    glm::vec3 up = glm::vec3(0, 1, 0),
              right = glm::vec3(1, 0, 0),
              forward = glm::vec3(0, 0, 1);

    glm::ivec2 resolution = { 1024, 1024 };
    i32 bounces = 5, rayPerPixel = 1;

    void updateDirection() {
        glm::vec3 direction;
        direction.x = cos(glm::radians(180 + yaw)) * cos(glm::radians(pitch));
        direction.y = sin(glm::radians(pitch));
        direction.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
        forward = glm::normalize(direction);
        right = normalize(-glm::cross(forward, glm::vec3(0, 1, 0)));
        up = glm::cross(-right, forward);
    }
};

