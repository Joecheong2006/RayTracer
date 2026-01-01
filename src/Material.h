#pragma once

#include "glm/glm.hpp"
#include "util.h"

using GVec3 = glm::vec<3, f32, glm::packed>;

struct MaterialTexture {
    i32 normalTexture;
    f32 normalScale;
    i32 baseColorTexture;
    i32 metallicRoughnessTexture;

};

struct Material {
    GVec3 emissionColor = { 0, 0, 0 };
    f32 emissionStrength = 0;

    GVec3 albedo = { 1, 1, 1 };
    f32 subsurface = 0;
    f32 roughness = 1.0;
    f32 metallic = 0;
    f32 specular = 0.5;
    f32 specularTint = 0;

    f32 transmission = 0;
    f32 ior = 1.5;

    MaterialTexture texture;

};

