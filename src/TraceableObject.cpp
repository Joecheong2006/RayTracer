#include "TraceableObject.h"
#include <algorithm>

void TraceableObject::writeHeader(std::vector<glm::vec4> &buffer) const {
    buffer.push_back({
            m_type,             // int for type declaration
            m_materialIndex,    // int for material's index
            0, 0                // 2 float are unused currently
        });
}

void Sphere::write(std::vector<glm::vec4> &buffer) const {
    writeHeader(buffer);
    buffer.push_back({
            center,         // vec3 for center
            radius          // 1 float for radius
        });
}

AABB Sphere::getAABB() const {
    return AABB(
            center - glm::vec3(radius),
            center + glm::vec3(radius)
        );
}

void Quad::write(std::vector<glm::vec4> &buffer) const {
    writeHeader(buffer);
    buffer.push_back({ q, cullFace });
    buffer.push_back({ u, 0 });
    buffer.push_back({ v, 0 });
}

AABB Quad::getAABB() const {
    glm::vec3 p0 = q;          // corner
    glm::vec3 p1 = q + u;      // corner + edge u
    glm::vec3 p2 = q + v;      // corner + edge v
    glm::vec3 p3 = q + u + v;  // corner + edge u + edge v
    return AABB(
            glm::min(glm::min(p0, p1), glm::min(p2, p3)),
            glm::max(glm::max(p0, p1), glm::max(p2, p3))
        );
}

void Triangle::write(std::vector<glm::vec4> &buffer) const {
    writeHeader(buffer);
    buffer.push_back({ posA, 0 });
    buffer.push_back({ posB, 0 });
    buffer.push_back({ posC, 0 });
}

AABB Triangle::getAABB() const {
    return AABB(
            glm::min(posA, glm::min(posB, posC)),
            glm::max(posA, glm::max(posB, posC))
        );
}

void Model::write(std::vector<glm::vec4> &buffer) const {
    writeHeader(buffer);
    buffer.push_back({ aabb.min, endIndex });
    buffer.push_back({ aabb.max, 0});
}

AABB Model::getAABB() const {
    if (triangles.size() == 0) {
        return {};
    }

    AABB box = triangles.front().getAABB();

    for (auto &triangle : triangles) {
        box = AABB(box, triangle.getAABB());
    }

    return box;
}

