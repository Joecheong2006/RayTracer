#pragma once

#include "util.h"
#include "glm/glm.hpp"

#include <vector>

enum class TraceableType {
    Sphere,
    Quad,
    Triangle,
    Model,
    Count
};

struct AABB {
    AABB() = default;

    AABB(glm::vec3 min, glm::vec3 max)
        : min(min - glm::vec3(1e-5)), max(max + glm::vec3(1e-5))
    {}

    AABB(const AABB &box1, const AABB &box2) {
        min.x = std::min(box1.min.x, box2.min.x) - 1e-5;
        min.y = std::min(box1.min.y, box2.min.y) - 1e-5;
        min.z = std::min(box1.min.z, box2.min.z) - 1e-5;

        max.x = std::max(box1.max.x, box2.max.x) + 1e-5;
        max.y = std::max(box1.max.y, box2.max.y) + 1e-5;
        max.z = std::max(box1.max.z, box2.max.z) + 1e-5;
    }

    glm::vec3 min, max;
};

class RayScene;
class TraceableObject {
    friend class RayScene;
private:
    i32 m_materialIndex = -1;
    TraceableType m_type = TraceableType::Count;

protected:
    void writeHeader(std::vector<glm::vec4> &buffer) const;
    virtual void write(std::vector<glm::vec4> &buffer) const = 0;

public:
    explicit TraceableObject(TraceableType type)
        : m_type(type)
    {}

    virtual ~TraceableObject() = default;

    inline i32 getMaterialIndex() const { return m_materialIndex; }

    virtual AABB getAABB() const = 0;
    virtual bool inAABB(const AABB &box) const = 0;

};

struct Sphere : public TraceableObject {
    Sphere(glm::vec3 center, f32 radius);

    virtual void write(std::vector<glm::vec4> &buffer) const override;
    virtual AABB getAABB() const override;
    virtual bool inAABB(const AABB &box) const override;

    glm::vec3 center = { 0, 0, 0 };
    f32 radius = 1.0;

};

struct Quad : public TraceableObject {
    Quad(glm::vec3 q, glm::vec3 u, glm::vec3 v, bool cullFace = false);

    virtual void write(std::vector<glm::vec4> &buffer) const override;
    virtual AABB getAABB() const override;
    virtual bool inAABB(const AABB &box) const override;

    glm::vec3 q, u, v;
    bool cullFace;
};

struct Triangle : public TraceableObject {
    Triangle(glm::vec3 posA, glm::vec3 posB, glm::vec3 posC);

    virtual void write(std::vector<glm::vec4> &buffer) const override;
    virtual AABB getAABB() const override;
    virtual bool inAABB(const AABB &box) const override;

    glm::vec3 posA, posB, posC;
};

struct Model : public TraceableObject {
    Model(std::vector<Triangle> triangles);

    virtual void write(std::vector<glm::vec4> &buffer) const override;
    virtual AABB getAABB() const override;
    virtual bool inAABB(const AABB &box) const override;

    std::vector<Triangle> triangles;
    int endIndex;
    AABB aabb;
};

