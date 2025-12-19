#pragma once

#include "util.h"
#include "glm/glm.hpp"

#include "AABB.h"

#include <vector>

using GVec3 = glm::vec<3, f32, glm::packed>;
using GVec3I = glm::vec<3, u32, glm::packed>;

enum class TraceableType {
    Sphere,
    Quad,
    Triangle,
    Model,
    Count
};

class RayScene;
class TraceableObject {
    friend class RayScene;
private:
    i32 m_materialIndex = -1;
    TraceableType m_type = TraceableType::Count;

protected:
    void writeHeader(std::vector<f32> &buffer) const;
    AABB boundingBox;

public:
    explicit TraceableObject(TraceableType type)
        : m_type(type)
    {}

    virtual ~TraceableObject() = default;

    virtual void write(std::vector<f32> &buffer) const = 0;
    virtual bool inAABB(const AABB &box) const = 0;

    inline i32 getMaterialIndex() const { return m_materialIndex; }
    inline virtual AABB getAABB() const { return boundingBox; }

};

struct Sphere : public TraceableObject {
    Sphere(GVec3 center, f32 radius);

    virtual void write(std::vector<f32> &buffer) const override;
    virtual bool inAABB(const AABB &box) const override;

    GVec3 center = { 0, 0, 0 };
    f32 radius = 1.0;

};

struct Quad : public TraceableObject {
    Quad(GVec3 q, GVec3 u, GVec3 v, bool cullFace = false);

    virtual void write(std::vector<f32> &buffer) const override;
    virtual bool inAABB(const AABB &box) const override;

    GVec3 q, u, v;
    bool cullFace;
};

struct Triangle : public TraceableObject {
    Triangle(GVec3 posA, GVec3 posB, GVec3 posC,
             GVec3 normA = {}, GVec3 normB = {}, GVec3 normC = {});

    virtual void write(std::vector<f32> &buffer) const override;
    virtual bool inAABB(const AABB &box) const override;

    GVec3 posA, posB, posC;
    GVec3 normA, normB, normC;
};

namespace tinygltf {
    class TinyGLTF;
    class Model;
}

#include "Material.h"
struct MeshData {
    std::vector<GVec3> vertices;
    std::vector<GVec3> normals;

    struct Identifier {
        GVec3I indices;
        i32 materialIndex;
    };

    std::vector<Identifier> identifiers;
    std::vector<Material> materials;

    inline static Triangle GetTriangleFromIdentifier(const MeshData &meshData, int index) {
        GVec3I idx = meshData.identifiers[index].indices;
        return Triangle {
            meshData.vertices[idx.x], meshData.vertices[idx.y], meshData.vertices[idx.z],
            meshData.normals[idx.x], meshData.normals[idx.y], meshData.normals[idx.z]
        };
    }
};

#include "BVHTree.h"
struct Model : public TraceableObject {
    static MeshData LoadMeshData(std::string modelPath);

    Model(std::string modelPath);

    virtual void write(std::vector<f32> &buffer) const override;
    virtual bool inAABB(const AABB &box) const override;

    MeshData meshData;
    BVHTree bvh;

};

