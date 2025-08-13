#include "BVHTree.h"
#include "TraceableObject.h"

#include <algorithm>
#include <stack>
#include <iostream>

void BVHTree::construct_bvh(std::vector<Triangle> &triangles, i32 depth, i32 axis, i32 start, i32 end) {
    AABB box;
    for (auto &triangle : triangles) {
        box = AABB(box, triangle.getAABB());
    }

    if (axis < 0) {
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
    }
    else {
        axis = (axis + 1) % 3;
    }

    BVHNode node;
    node.box = box;

    if (depth == 0) {
        m_triangles.insert(m_triangles.end(), triangles.begin(), triangles.end());
        node.leftIndex = start;
        node.rightIndex = end;
        node.isLeaf = true;
        m_nodes.emplace_back(node);
        return;
    }
    else {
        node.leftIndex = m_nodes.size() + 1;
        node.rightIndex = m_nodes.size() + std::pow(2, depth);
        m_nodes.emplace_back(node);
    }

    i32 mid = (end + start) / 2;;

    std::sort(triangles.begin(), triangles.end(),
            [axis](const auto &tri1, const auto &tri2) {
                return tri1.posA[axis] < tri2.posA[axis];
            }
        );

    i32 rMid = triangles.size() / 2;

    auto t = std::vector<Triangle>(triangles.begin(), triangles.begin() + rMid);
    construct_bvh( // Construct Left Node
            t, depth - 1, axis, start, mid);

    t = std::vector<Triangle>(triangles.begin() + rMid, triangles.end());
    construct_bvh( // Construct Right Node
            t, depth - 1, axis, mid, end);
}

BVHTree::BVHTree(const std::vector<Triangle>& triangles) {
    i32 depth = std::log2(triangles.size());

    m_triangles.reserve(triangles.size());
    m_nodes.reserve(std::pow(2, depth) * 2 - 1);

    auto tris = std::vector<Triangle>(triangles);
    construct_bvh(tris, depth, -1, 0, tris.size());

    std::stack<BVHNode*> s;
    s.push(&m_nodes.front());
    while (!s.empty()) {
        auto *current = s.top();
        s.pop();

        if (current->isLeaf) {
            std::cout << '[' << current->leftIndex << ' ' << current->rightIndex << ") ";
            for (i32 i = current->leftIndex; i < current->rightIndex; ++i) {
                if (!m_triangles[i].inAABB(current->box)) {
                    std::cout << "Invalid BVH\n";
                    return;
                }
            }
            continue;
        }

        s.push(&m_nodes[current->rightIndex]);
        s.push(&m_nodes[current->leftIndex]);
    }

    std::cout
        << "\nBVH Generated!\n"
        << "\tNodes Count: " << m_nodes.size() << '\n'
        << "\tTriangles Count: " << m_triangles.size() << '\n'
        << "\tBVH Height: " << depth << '\n'
        << "\tAvg Triangles Per Leaf Node: " << tris.size() / std::pow(2, depth) << '\n';
}

