#pragma once

#include "util.h"
#include "glm/glm.hpp"

#include "AABB.h"

#include <vector> // std::vector
#include <string> // std::string
#include <memory> // std::unique_ptr

#include "gpu/Serializable.h"

enum class TraceableType {
    Sphere,
    Quad,
    Triangle,
    Model,
    Count
};

class RayScene;
class TraceableObject : public gpu::Serializable {
    friend class RayScene;
private:
    i32 m_materialIndex = -1;
    TraceableType m_type = TraceableType::Count;

protected:
    AABB boundingBox;

public:
    explicit TraceableObject(TraceableType type)
        : m_type(type)
    {}

    virtual ~TraceableObject() = default;
    virtual std::unique_ptr<TraceableObject> clone() const = 0;

    virtual bool inAABB(const AABB &box) const = 0;

    inline TraceableType getType() const { return m_type; }
    inline i32 getMaterialIndex() const { return m_materialIndex; }
    inline virtual AABB getAABB() const { return boundingBox; }

};

struct Sphere : public TraceableObject {
    Sphere(glm::vec3 center, f32 radius);

    virtual bool inAABB(const AABB &box) const override;
    virtual void serialize(gpu::Buffer &buffer) const override;

    glm::vec3 center = { 0, 0, 0 };
    f32 radius = 1.0;

    virtual std::unique_ptr<TraceableObject> clone() const override {
        return std::make_unique<Sphere>(*this);
    }

};

struct Quad : public TraceableObject {
    Quad(glm::vec3 q, glm::vec3 u, glm::vec3 v, bool cullFace = false);

    virtual bool inAABB(const AABB &box) const override;
    virtual void serialize(gpu::Buffer &buffer) const override;

    glm::vec3 q, u, v;
    bool cullFace;

    virtual std::unique_ptr<TraceableObject> clone() const override {
        return std::make_unique<Quad>(*this);
    }

};

struct Triangle : public TraceableObject {
    Triangle(glm::vec3 posA, glm::vec3 posB, glm::vec3 posC,
             glm::vec3 normA = {}, glm::vec3 normB = {}, glm::vec3 normC = {});

    virtual bool inAABB(const AABB &box) const override;
    virtual void serialize(gpu::Buffer &buffer) const override;

    glm::vec3 posA, posB, posC;
    glm::vec3 normA, normB, normC;

    virtual std::unique_ptr<TraceableObject> clone() const override {
        return std::make_unique<Triangle>(*this);
    }

};

namespace tinygltf {
    class TinyGLTF;
    class Model;
}

#include "BVHTree.h"
#include "MeshData.h"

struct Model : public TraceableObject {
    inline Triangle getTriangleFromIdentifier(int idenIndex) {
        glm::ivec3 idx = meshData.identifiers[idenIndex].index;
        return Triangle {
            meshData.vertices[idx.x], meshData.vertices[idx.y], meshData.vertices[idx.z],
            meshData.normals[idx.x], meshData.normals[idx.y], meshData.normals[idx.z]
        };
    }

    inline Triangle getTriangleFromIdentifier(const MeshData::Identifier &iden) {
        glm::ivec3 idx = iden.index;
        return Triangle {
            meshData.vertices[idx.x], meshData.vertices[idx.y], meshData.vertices[idx.z],
            meshData.normals[idx.x], meshData.normals[idx.y], meshData.normals[idx.z]
        };
    }

    Model(std::string modelPath);
    Model(Model&&) = default;

    Model(const Model &model)
        : TraceableObject(TraceableType::Model)
        , meshData(model.meshData)
        , bvh(model.bvh)
    {}

    virtual bool inAABB(const AABB &box) const override;
    virtual void serialize(gpu::Buffer &buffer) const override;

    MeshData meshData;
    BVHTree bvh;

    virtual std::unique_ptr<TraceableObject> clone() const override {
        return std::make_unique<Model>(*this);
    }

};

