#pragma once

#include "util.h"
#include "glm/glm.hpp"

#include <vector>

enum class TraceableType {
    Sphere,
    Quad,
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
    virtual void write(std::vector<glm::vec4> &buffer) const = 0;

public:
    explicit TraceableObject(TraceableType type)
        : m_type(type)
    {}

    virtual ~TraceableObject() = default;

    inline i32 getMaterialIndex() const { return m_materialIndex; }

};

struct Sphere : public TraceableObject {
    Sphere(glm::vec3 center, f32 radius)
        : TraceableObject(TraceableType::Sphere)
        , center(center)
        , radius(radius)
    {}

    virtual void write(std::vector<glm::vec4> &buffer) const override;

    glm::vec3 center = { 0, 0, 0 };
    f32 radius = 1.0;

};

struct Quad : public TraceableObject {
    Quad(glm::vec3 q, glm::vec3 u, glm::vec3 v, bool cullFace = false)
        : TraceableObject(TraceableType::Quad)
        , q(q), u(u), v(v), cullFace(cullFace)
    {}

    virtual void write(std::vector<glm::vec4> &buffer) const override;

    glm::vec3 q, u, v;
    bool cullFace;
};

