#pragma once

#include "glm/glm.hpp"
#include "util.h"

struct Material {
    glm::vec3 emissionColor = { 0, 0, 0 };
    f32 emissionStrength = 0;

    glm::vec3 albedo = { 1, 1, 1 };
    f32 subsurface = 0;
    f32 roughness = 1.0;
    f32 metallic = 0;
    f32 specular = 0.5;
    f32 specularTint = 0;

};

