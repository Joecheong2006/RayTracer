#include "BVHTree.h"
#include "TraceableObject.h"

#include <algorithm>

static void construct_bvh(std::vector<BVHNode> &m_nodes, std::vector<MeshData::Identifier> &identifiers, Model &model, i32 axis, i32 start, i32 end) {
    AABB box = model.getTriangleFromIdentifier(start).getAABB();
    for (i32 i = start + 1; i < end; ++i) {
        box = AABB(box, model.getTriangleFromIdentifier(i).getAABB());
    }

    BVHNode node;
    node.box = box;

    i32 currentIndex = m_nodes.size();
    m_nodes.emplace_back(node);

    if (end - start <= 2) {
        m_nodes[currentIndex].leftIndex = start;
        m_nodes[currentIndex].rightIndex = end;
        m_nodes[currentIndex].isLeaf = true;
        identifiers.insert(identifiers.end(),
                model.meshData.identifiers.begin() + start, model.meshData.identifiers.begin() + end);
        return;
    }

    axis = (axis + 1) % 3;

    i32 mid = start + (end - start) * 0.5f;
    std::nth_element(model.meshData.identifiers.begin() + start, model.meshData.identifiers.begin() + mid, model.meshData.identifiers.begin() + end,
            [&model, axis](const auto &iden1, const auto &iden2) {
                Triangle tri1 = model.getTriangleFromIdentifier(iden1);
                Triangle tri2 = model.getTriangleFromIdentifier(iden2);
                glm::vec3 c1 = (tri1.posA + tri1.posB + tri1.posC) / 3.0f;
                glm::vec3 c2 = (tri2.posA + tri2.posB + tri2.posC) / 3.0f;
                return c1[axis] < c2[axis];
            }
        );

    m_nodes[currentIndex].leftIndex = currentIndex + 1;

    construct_bvh( // Construct Left Node
            m_nodes, identifiers, model, axis, start, mid);

    m_nodes[currentIndex].rightIndex = m_nodes.size();

    construct_bvh( // Construct Right Node
            m_nodes, identifiers, model, axis, mid, end);
}

BVHTree::BVHTree(Model &model) {
    MeshData &meshData = model.meshData;
    if (meshData.identifiers.size() == 0) {
        return;
    }

    m_nodes.reserve(meshData.identifiers.size() * 2);

    AABB box = model.getTriangleFromIdentifier(0).getAABB();
    for (int i = 1; i < meshData.identifiers.size(); ++i) {
        box = AABB(box, model.getTriangleFromIdentifier(i).getAABB());
    }

    i32 axis;
    f32 xRange = box.max.x - box.min.x;
    f32 yRange = box.max.y - box.min.y;
    f32 zRange = box.max.z - box.min.z;
    f32 maxRange = std::max({ xRange, yRange, zRange });
    if (maxRange == xRange) {
        axis = 0;
    }
    else if (maxRange == yRange) {
        axis = 1;
    }
    else {
        axis = 2;
    }

    std::vector<MeshData::Identifier> identifiers;

    construct_bvh(m_nodes, identifiers, model, axis, 0, meshData.identifiers.size());

    // Update the identifiers
    meshData.identifiers = identifiers;
}

