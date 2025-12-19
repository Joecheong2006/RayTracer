#pragma once
#include "util.h"
#include "AABB.h"
#include <vector>

struct BVHNode {
    AABB box;
    i32 leftIndex = -1, rightIndex = -1;
    bool isLeaf = false;

    BVHNode() = default;
};

class Triangle;
struct MeshData;
class BVHTree {
    std::vector<BVHNode> m_nodes;

public:
    BVHTree(MeshData &meshData);

    const std::vector<BVHNode>& getNodes() const { return m_nodes; }

};

