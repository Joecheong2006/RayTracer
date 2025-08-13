#pragma once
#include "TraceableObject.h"

struct BVHNode {
    AABB box;
    i32 leftIndex = -1, rightIndex = -1;
    bool isLeaf = false;

    BVHNode() = default;
};

class BVHTree {
    void construct_bvh(std::vector<Triangle> &triangles, i32 depth, i32 axis, i32 start, i32 end);

    std::vector<BVHNode> m_nodes;
    std::vector<Triangle> m_triangles;

public:
    BVHTree(const std::vector<Triangle>& triangles);

};

