#include "TraceableObject.h"

void TraceableObject::writeHeader(std::vector<f32> &buffer) const {
    buffer.push_back(static_cast<f32>(m_type));
    buffer.push_back(static_cast<f32>(m_materialIndex));
}

Sphere::Sphere(GVec3 center, f32 radius)
    : TraceableObject(TraceableType::Sphere)
    , center(center)
    , radius(radius)
{
    boundingBox = AABB(
            center - GVec3(radius),
            center + GVec3(radius)
        );
}

void Sphere::write(std::vector<f32> &buffer) const {
    writeHeader(buffer);
    buffer.insert(buffer.end(), &center.x, &center.x + 3);
    buffer.push_back(radius);
}

bool Sphere::inAABB(const AABB &box) const {
    return (center.x - radius >= box.min.x && center.x + radius <= box.max.x) &&
           (center.y - radius >= box.min.y && center.y + radius <= box.max.y) &&
           (center.z - radius >= box.min.z && center.z + radius <= box.max.z);
}

Quad::Quad(GVec3 q, GVec3 u, GVec3 v, bool cullFace)
    : TraceableObject(TraceableType::Quad)
    , q(q), u(u), v(v), cullFace(cullFace)
{
    GVec3 p0 = q;          // corner
    GVec3 p1 = q + u;      // corner + edge u
    GVec3 p2 = q + v;      // corner + edge v
    GVec3 p3 = q + u + v;  // corner + edge u + edge v
    boundingBox = AABB(
            glm::min(glm::min(p0, p1), glm::min(p2, p3)),
            glm::max(glm::max(p0, p1), glm::max(p2, p3))
        );
}

void Quad::write(std::vector<f32> &buffer) const {
    writeHeader(buffer);
    buffer.insert(buffer.end(), &q.x, &q.x + 3);
    buffer.insert(buffer.end(), &u.x, &u.x + 3);
    buffer.insert(buffer.end(), &v.x, &v.x + 3);
    buffer.push_back(static_cast<f32>(cullFace));
}

bool Quad::inAABB(const AABB &box) const {
    GVec3 p0 = q;          // corner
    GVec3 p1 = q + u;      // corner + edge u
    GVec3 p2 = q + v;      // corner + edge v
    GVec3 p3 = q + u + v;  // corner + edge u + edge v

    GVec3 quadMin = glm::min(glm::min(p0, p1), glm::min(p2, p3));
    GVec3 quadMax = glm::max(glm::max(p0, p1), glm::max(p2, p3));

    return (quadMax.x >= box.min.x && quadMin.x <= box.max.x) &&
           (quadMax.y >= box.min.y && quadMin.y <= box.max.y) &&
           (quadMax.z >= box.min.z && quadMin.z <= box.max.z);
}

Triangle::Triangle(GVec3 posA, GVec3 posB, GVec3 posC, GVec3 normA, GVec3 normB, GVec3 normC)
    : TraceableObject(TraceableType::Triangle)
    , posA(posA), posB(posB), posC(posC)
    , normA(normA), normB(normB), normC(normC)
{
    boundingBox = AABB(
            glm::min(posA, glm::min(posB, posC)),
            glm::max(posA, glm::max(posB, posC))
        );
}

void Triangle::write(std::vector<f32> &buffer) const {
    writeHeader(buffer);
    buffer.insert(buffer.end(), &posA.x, &posA.x + 3);
    buffer.insert(buffer.end(), &posB.x, &posB.x + 3);
    buffer.insert(buffer.end(), &posC.x, &posC.x + 3);
    buffer.insert(buffer.end(), &normA.x, &normA.x + 3);
    buffer.insert(buffer.end(), &normB.x, &normB.x + 3);
    buffer.insert(buffer.end(), &normC.x, &normC.x + 3);
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

void Model::write(std::vector<f32> &buffer) const {
    writeHeader(buffer);

    const auto &nodes = bvh.getNodes();
    const auto &triangles = bvh.getTriangles();
    buffer.insert(buffer.end(), &boundingBox.min.x, &boundingBox.min.x + 3);
    buffer.insert(buffer.end(), &boundingBox.max.x, &boundingBox.max.x + 3);
    buffer.push_back(triangles.size());
    buffer.push_back(nodes.size());

    for (auto &node : nodes) {
        buffer.insert(buffer.end(), &node.box.min.x, &node.box.min.x + 3);
        buffer.insert(buffer.end(), &node.box.max.x, &node.box.max.x + 3);
        buffer.push_back(node.leftIndex);
        buffer.push_back(node.rightIndex);
        buffer.push_back(static_cast<bool>(node.isLeaf));
    }
}

bool Model::inAABB(const AABB &box) const {
    GVec3 clampedMin = glm::max(boundingBox.min, box.min);
    GVec3 clampedMax = glm::min(boundingBox.max, box.max);
    return clampedMin == boundingBox.min && clampedMax == boundingBox.max;
}

