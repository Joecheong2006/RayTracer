#pragma once

#include "util.h"
#include "glm/glm.hpp"

#include "AABB.h"

#include <vector>

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
    void writeHeader(std::vector<glm::vec4> &buffer) const;
    AABB boundingBox;

public:
    explicit TraceableObject(TraceableType type)
        : m_type(type)
    {}

    virtual ~TraceableObject() = default;

    virtual void write(std::vector<glm::vec4> &buffer) const = 0;
    virtual bool inAABB(const AABB &box) const = 0;

    inline i32 getMaterialIndex() const { return m_materialIndex; }
    inline virtual AABB getAABB() const { return boundingBox; }

};

struct Sphere : public TraceableObject {
    Sphere(glm::vec3 center, f32 radius);

    virtual void write(std::vector<glm::vec4> &buffer) const override;
    virtual bool inAABB(const AABB &box) const override;

    glm::vec3 center = { 0, 0, 0 };
    f32 radius = 1.0;

};

struct Quad : public TraceableObject {
    Quad(glm::vec3 q, glm::vec3 u, glm::vec3 v, bool cullFace = false);

    virtual void write(std::vector<glm::vec4> &buffer) const override;
    virtual bool inAABB(const AABB &box) const override;

    glm::vec3 q, u, v;
    bool cullFace;
};

struct Triangle : public TraceableObject {
    Triangle(glm::vec3 posA, glm::vec3 posB, glm::vec3 posC,
             glm::vec3 normA = {}, glm::vec3 normB = {}, glm::vec3 normC = {});

    virtual void write(std::vector<glm::vec4> &buffer) const override;
    virtual bool inAABB(const AABB &box) const override;

    glm::vec3 posA, posB, posC;
    glm::vec3 normA, normB, normC;
};

#include "BVHTree.h"
struct Model : public TraceableObject {
    Model(const std::vector<Triangle> &triangles);

    virtual void write(std::vector<glm::vec4> &buffer) const override;
    virtual bool inAABB(const AABB &box) const override;

    BVHTree bvh;

};

