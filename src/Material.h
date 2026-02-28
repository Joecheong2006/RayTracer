#pragma once

#include "glm/glm.hpp"
#include "util.h"

#include "gpu/Serializable.h"

using GVec3 = glm::vec<3, f32, glm::packed>;

struct MaterialTexture {
    i32 normalTexture = -1;
    i32 baseColorTexture = -1;
    i32 metallicRoughnessTexture = -1;
    i32 emissiveTexture = -1;
    i32 transmissionTexture = -1;
    i32 occlusionTexture = -1;

};

struct Material : public gpu::Serializable {
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

    f32 alphaCut = 0;

    f32 normalScale = -1;
    f32 occlusionStrength = 1.0;

    MaterialTexture texture;

    virtual void serialize(gpu::Buffer &buffer) const {
        buffer.push(texture.normalTexture);
        buffer.push(texture.baseColorTexture);
        buffer.push(texture.metallicRoughnessTexture);
        buffer.push(texture.emissiveTexture);
        buffer.push(texture.transmissionTexture);
        buffer.push(texture.occlusionTexture);

        buffer.push(emissionColor);
        buffer.push(emissionStrength);
        buffer.push(albedo);
        buffer.push(subsurface);
        buffer.push(roughness);
        buffer.push(metallic);
        buffer.push(specular);
        buffer.push(specularTint);
        buffer.push(transmission);
        buffer.push(ior);

        buffer.push(alphaCut);
        buffer.push(normalScale);
        buffer.push(occlusionStrength);
    }

};

