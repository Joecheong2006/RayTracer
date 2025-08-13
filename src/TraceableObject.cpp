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
    return AABB(
            center - glm::vec3(radius),
            center + glm::vec3(radius)
        );
}

bool Sphere::inAABB(const AABB &box) const {
    return (center.x - radius >= box.min.x && center.x + radius <= box.max.x) &&
           (center.y - radius >= box.min.y && center.y + radius <= box.max.y) &&
           (center.z - radius >= box.min.z && center.z + radius <= box.max.z);
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

bool Quad::inAABB(const AABB &box) const {
    glm::vec3 p0 = q;          // corner
    glm::vec3 p1 = q + u;      // corner + edge u
    glm::vec3 p2 = q + v;      // corner + edge v
    glm::vec3 p3 = q + u + v;  // corner + edge u + edge v

    glm::vec3 quadMin = glm::min(glm::min(p0, p1), glm::min(p2, p3));
    glm::vec3 quadMax = glm::max(glm::max(p0, p1), glm::max(p2, p3));

    return (quadMax.x >= box.min.x && quadMin.x <= box.max.x) &&
           (quadMax.y >= box.min.y && quadMin.y <= box.max.y) &&
           (quadMax.z >= box.min.z && quadMin.z <= box.max.z);
}

void Triangle::write(std::vector<glm::vec4> &buffer) const {
    writeHeader(buffer);
    buffer.push_back({ posA, 0 });
    buffer.push_back({ posB, 0 });
    buffer.push_back({ posC, 0 });
}

bool Triangle::inAABB(const AABB &box) const {
    for (auto v : {posA, posB, posC}) {
        if (v.x < box.min.x || v.x > box.max.x ||
            v.y < box.min.y || v.y > box.max.y ||
            v.z < box.min.z || v.z > box.max.z) {
            return false;
        }
    }
    return true;
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

bool Model::inAABB(const AABB &box) const {
    glm::vec3 clampedMin = glm::max(aabb.min, box.min);
    glm::vec3 clampedMax = glm::min(aabb.max, box.max);
    return clampedMin == aabb.min && clampedMax == aabb.max;
}

