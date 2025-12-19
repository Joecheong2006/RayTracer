#pragma once

#include "Material.h"
#include "glm/glm.hpp"

#include <vector> // std::vector
                  //
using GVec3 = glm::vec<3, f32, glm::packed>;
using GVec3I = glm::vec<3, u32, glm::packed>;

struct MeshData {
    std::vector<GVec3> vertices;
    std::vector<GVec3> normals;

    struct Identifier {
        GVec3I indices;
        i32 materialIndex;
    };

    std::vector<Identifier> identifiers;
    std::vector<Material> materials;

    static MeshData LoadMeshData(std::string modelPath);
};

