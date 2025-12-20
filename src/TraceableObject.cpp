#include "TraceableObject.h"

#include <iostream>
#include <stack>

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

Model::Model(std::string modelPath)
    : TraceableObject(TraceableType::Model)
    , meshData(MeshData::LoadMeshData(modelPath))
    , bvh(*this)
{
    if (meshData.identifiers.size() > 0) {
        boundingBox = getTriangleFromIdentifier(0).getAABB();
        for (int i = 1; i < meshData.identifiers.size(); ++i) {
            boundingBox = AABB(boundingBox, getTriangleFromIdentifier(i).getAABB());
        }
    }

    // Height Info
    i32 maxHeight = 0;
    i32 minHeight = INT_MAX;
    i32 totalHeight = 0;

    // Leaf Node Info
    i32 leafNodeCount = 0;
    i32 emptyLeaf = 0;
    i32 maxTri = INT_MIN;
    i32 minTri = INT_MAX;

    auto &m_nodes = bvh.getNodes();

    std::stack<glm::ivec2> s;
    s.push({ 0, 1 });
    while (!s.empty()) {
        auto track = s.top();
        auto currentIndex = track[0];
        s.pop();

        maxHeight = std::max(maxHeight, track[1]);
        minHeight = std::min(minHeight, track[1]);

        auto current = m_nodes[currentIndex];
        if (current.isLeaf) {
            leafNodeCount++;
            emptyLeaf += current.leftIndex == current.rightIndex;
            totalHeight += track[1];

            maxTri = std::max(maxTri, current.rightIndex - current.leftIndex);
            minTri = std::min(minTri, current.rightIndex - current.leftIndex);

            for (i32 i = current.leftIndex; i < current.rightIndex; ++i) {
                if (!getTriangleFromIdentifier(i).inAABB(current.box)) {
                    std::cout << "Invalid BVH\n";
                    return;
                }
            }
            continue;
        }

        s.push({ current.rightIndex, track[1] + 1 });
        s.push({ current.leftIndex, track[1] + 1 });
    }

    glm::ivec2 nodesUsage = { m_nodes.size() * sizeof(BVHNode), m_nodes.size() * 3 * sizeof(glm::vec4) };
    glm::ivec2 trianglesUsage = { meshData.identifiers.size() * sizeof(Triangle) / 3, meshData.identifiers.size() * 6 * sizeof(glm::vec4) };

    glm::ivec2 bvhUsage = nodesUsage + trianglesUsage;

    const char *gaps = "\t";

    std::cout
        << "\nBVH Constructed Successfully!\n"
        << "\tNodes Count: " << gaps << gaps <<m_nodes.size() << '\n'
            << "\t\tCPU Usage: " << gaps << nodesUsage[0] / 1024.0f << " (KB)\n"
            << "\t\tGPU Usage: " << gaps << nodesUsage[1] / 1024.0f << " (KB)\n"

        << "\tTriangles Count: " << gaps << meshData.identifiers.size() << '\n'
            << "\t\tCPU Usage: " << gaps << trianglesUsage[0] / 1024.0f << " (KB)\n"
            << "\t\tGPU Usage: " << gaps << trianglesUsage[1] / 1024.0f << " (KB)\n"

        << "\tBVH Memory Usage: " << '\n'
            << "\t\tCPU Usage: "  << gaps << bvhUsage[0] / 1024.0f<< " (KB)\n"
            << "\t\tGPU Usage: "  << gaps << bvhUsage[1] / 1024.0f << " (KB)\n"

        << "\tBVH Min Height: " << gaps << minHeight << '\n'
        << "\tBVH Max Height: " << gaps << maxHeight << '\n'
        << "\tBVH Avg Height: " << gaps << totalHeight / (f32)leafNodeCount << '\n'

        << "\tMin Triangles Leaf: " << gaps << minTri << '\n'
        << "\tMax Triangles Leaf: " << gaps << maxTri << '\n'
        << "\tAvg Triangles Leaf: " << gaps << meshData.identifiers.size() / (f32)leafNodeCount << '\n'
        << "\tEmpty Leaf: " << gaps << gaps << emptyLeaf  << "\n\n";
}

void Model::write(std::vector<f32> &buffer) const {
    const auto &nodes = bvh.getNodes();
    buffer.push_back(meshData.identifiers.size());
    buffer.push_back(meshData.vertices.size());
    buffer.push_back(nodes.size());

    std::cout << "Wrote nodesCount: " << nodes.size() << std::endl;
    std::cout << "Wrote verticesCount: " << meshData.vertices.size() << std::endl;
    std::cout << "Wrote identifiersCount: " << meshData.identifiers.size() << std::endl;
    for (const auto &node : nodes) {
        buffer.insert(buffer.end(), &node.box.min.x, &node.box.min.x + 3);
        buffer.insert(buffer.end(), &node.box.max.x, &node.box.max.x + 3);
        buffer.push_back(node.leftIndex);
        buffer.push_back(node.rightIndex);
        buffer.push_back(static_cast<bool>(node.isLeaf));
    }

    for (const auto &iden : meshData.identifiers) {
        buffer.insert(buffer.end(), &iden.indices.x, &iden.indices.x + 3);
        buffer.push_back(iden.materialIndex);
    }

    for (const auto &vertex : meshData.vertices) {
        buffer.insert(buffer.end(), &vertex.x, &vertex.x + 3);
    }

    for (const auto &normal : meshData.normals) {
        buffer.insert(buffer.end(), &normal.x, &normal.x + 3);
    }
}

bool Model::inAABB(const AABB &box) const {
    GVec3 clampedMin = glm::max(boundingBox.min, box.min);
    GVec3 clampedMax = glm::min(boundingBox.max, box.max);
    return clampedMin == boundingBox.min && clampedMax == boundingBox.max;
}

