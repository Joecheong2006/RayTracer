#pragma once

#include "gpu/Serializable.h"
#include "Material.h"
#include "glm/glm.hpp"

#include <vector> // std::vector
#include <string> // std::string

struct MeshData {
    struct Vertex {
        glm::vec3 position;
        glm::vec3 normal;
        glm::vec2 uv;
    };

    std::vector<Vertex> vertices;

    struct Identifier : public gpu::Serializable {
        glm::ivec3 index;
        i32 materialIndex;
        bool hasTextures = false;
        virtual void serialize(gpu::Buffer &buffer) const override;
    };

    std::vector<Identifier> identifiers;
    std::vector<Material> materials;

    std::vector<Identifier> lightIdentifiers;

    struct Texture : public gpu::Serializable {
        i32 width, height, channels, channelSize, wrapS, wrapT;
        std::vector<f32> data;
        virtual void serialize(gpu::Buffer &buffer) const override;
    };

    std::vector<Texture> textures;

    i32 lightSourcesCount = 0;

    static MeshData LoadMeshData(std::string modelPath);
};

