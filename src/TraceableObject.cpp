#include "TraceableObject.h"

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
    return AABB();
}

void Quad::write(std::vector<glm::vec4> &buffer) const {
    writeHeader(buffer);
    buffer.push_back({ q, cullFace });
    buffer.push_back({ u, 0 });
    buffer.push_back({ v, 0 });
}

AABB Quad::getAABB() const {
    return AABB();
}

void Triangle::write(std::vector<glm::vec4> &buffer) const {
    writeHeader(buffer);
    buffer.push_back({ posA, 0 });
    buffer.push_back({ posB, 0 });
    buffer.push_back({ posC, 0 });
}

AABB Triangle::getAABB() const {
    AABB box;

    box.min.x = std::min({posA.x, posB.x, posC.x});
    box.min.y = std::min({posA.y, posB.y, posC.y});
    box.min.z = std::min({posA.z, posB.z, posC.z});

    box.max.x = std::max({posA.x, posB.x, posC.x});
    box.max.y = std::max({posA.y, posB.y, posC.y});
    box.max.z = std::max({posA.z, posB.z, posC.z});

    return box;
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

