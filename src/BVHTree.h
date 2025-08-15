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
class BVHTree {
    void construct_bvh(std::vector<Triangle> &triangles, i32 axis, i32 start, i32 end);

    std::vector<BVHNode> m_nodes;
    std::vector<Triangle> m_triangles;

public:
    BVHTree(const std::vector<Triangle>& triangles);

    const std::vector<BVHNode>& getNodes() const { return m_nodes; }
    const std::vector<Triangle>& getTriangles() const { return m_triangles; }

};

