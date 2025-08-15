#include "TraceableObject.h"

void TraceableObject::writeHeader(std::vector<glm::vec4> &buffer) const {
    buffer.push_back({
            m_type,             // int for type declaration
            m_materialIndex,    // int for material's index
            0, 0                // 2 float are unused currently
        });
}

Sphere::Sphere(glm::vec3 center, f32 radius)
    : TraceableObject(TraceableType::Sphere)
    , center(center)
    , radius(radius)
{
    boundingBox = AABB(
            center - glm::vec3(radius),
            center + glm::vec3(radius)
        );
}

void Sphere::write(std::vector<glm::vec4> &buffer) const {
    writeHeader(buffer);
    buffer.push_back({
            center,         // vec3 for center
            radius          // 1 float for radius
        });
}

bool Sphere::inAABB(const AABB &box) const {
    return (center.x - radius >= box.min.x && center.x + radius <= box.max.x) &&
           (center.y - radius >= box.min.y && center.y + radius <= box.max.y) &&
           (center.z - radius >= box.min.z && center.z + radius <= box.max.z);
}

Quad::Quad(glm::vec3 q, glm::vec3 u, glm::vec3 v, bool cullFace)
    : TraceableObject(TraceableType::Quad)
    , q(q), u(u), v(v), cullFace(cullFace)
{
    glm::vec3 p0 = q;          // corner
    glm::vec3 p1 = q + u;      // corner + edge u
    glm::vec3 p2 = q + v;      // corner + edge v
    glm::vec3 p3 = q + u + v;  // corner + edge u + edge v
    boundingBox = AABB(
            glm::min(glm::min(p0, p1), glm::min(p2, p3)),
            glm::max(glm::max(p0, p1), glm::max(p2, p3))
        );
}

void Quad::write(std::vector<glm::vec4> &buffer) const {
    writeHeader(buffer);
    buffer.push_back({ q, cullFace });
    buffer.push_back({ u, 0 });
    buffer.push_back({ v, 0 });
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

Triangle::Triangle(glm::vec3 posA, glm::vec3 posB, glm::vec3 posC, glm::vec3 normA, glm::vec3 normB, glm::vec3 normC)
    : TraceableObject(TraceableType::Triangle)
    , posA(posA), posB(posB), posC(posC)
    , normA(normA), normB(normB), normC(normC)
{
    boundingBox = AABB(
            glm::min(posA, glm::min(posB, posC)),
            glm::max(posA, glm::max(posB, posC))
        );
}

void Triangle::write(std::vector<glm::vec4> &buffer) const {
    writeHeader(buffer);
    buffer.push_back({ posA, normA.x });
    buffer.push_back({ posB, normA.y });
    buffer.push_back({ posC, normA.z });
    buffer.push_back({ normB, 0 });
    buffer.push_back({ normC, 0 });
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

Model::Model(const std::vector<Triangle> &triangles)
    : TraceableObject(TraceableType::Model)
    , bvh(triangles)
{
    if (triangles.size() > 0) {
        boundingBox = triangles.front().getAABB();
        for (auto &triangle : triangles) {
            boundingBox = AABB(boundingBox, triangle.getAABB());
        }
    }
}

void Model::write(std::vector<glm::vec4> &buffer) const {
    writeHeader(buffer);

    const auto &nodes = bvh.getNodes();
    const auto &triangles = bvh.getTriangles();

    buffer.push_back({ boundingBox.min, triangles.size() });
    buffer.push_back({ boundingBox.max, nodes.size() });

    for (auto &node : nodes) {
        buffer.push_back({ node.box.min, 0 });
        buffer.push_back({ node.box.max, 0 });
        buffer.push_back({ node.leftIndex, node.rightIndex, node.isLeaf, 0 });
    }
}

bool Model::inAABB(const AABB &box) const {
    glm::vec3 clampedMin = glm::max(boundingBox.min, box.min);
    glm::vec3 clampedMax = glm::min(boundingBox.max, box.max);
    return clampedMin == boundingBox.min && clampedMax == boundingBox.max;
}

