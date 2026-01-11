#pragma once

#include "Material.h"
#include "glm/glm.hpp"

#include <vector> // std::vector
#include <string> // std::string

struct MeshData {
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::vector<glm::vec2> UVs;

    struct Identifier {
        glm::ivec3 index;
        i32 materialIndex;
    };

    std::vector<Identifier> identifiers;
    std::vector<Material> materials;

    struct Texture {
        i32 width, height, channels, channelSize, wrapS, wrapT;
        std::vector<f32> data;
    };

    std::vector<Texture> textures;

    static MeshData LoadMeshData(std::string modelPath);
};

