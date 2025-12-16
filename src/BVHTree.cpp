#include "BVHTree.h"
#include "TraceableObject.h"

#include <algorithm>
#include <stack>
#include <iostream>

void BVHTree::construct_bvh(std::vector<Triangle> &triangles, i32 axis, i32 start, i32 end) {
    AABB box = triangles[start].getAABB();
    for (i32 i = start + 1; i < end; ++i) {
        box = AABB(box, triangles[i].getAABB());
    }

    BVHNode node;
    node.box = box;

    i32 currentIndex = m_nodes.size();
    m_nodes.emplace_back(node);

    if (end - start <= 2) {
        m_triangles.insert(m_triangles.end(), triangles.begin() + start, triangles.begin() + end);
        m_nodes[currentIndex].leftIndex = start;
        m_nodes[currentIndex].rightIndex = end;
        m_nodes[currentIndex].isLeaf = true;
        return;
    }

    axis = (axis + 1) % 3;

    i32 mid = start + (end - start) * 0.5f;
    // std::nth_element(triangles.begin() + start, triangles.begin() + mid, triangles.begin() + end,
    std::sort(triangles.begin() + start, triangles.begin() + end,
            [axis](const auto &tri1, const auto &tri2) {
                glm::vec3 c1 = (tri1.posA + tri1.posB + tri1.posC) / 3.0f;
                glm::vec3 c2 = (tri2.posA + tri2.posB + tri2.posC) / 3.0f;
                return c1[axis] < c2[axis];
            }
        );

    m_nodes[currentIndex].leftIndex = currentIndex + 1;

    construct_bvh( // Construct Left Node
            triangles, axis, start, mid);

    m_nodes[currentIndex].rightIndex = m_nodes.size();

    construct_bvh( // Construct Right Node
            triangles, axis, mid, end);
}

BVHTree::BVHTree(const MeshData &meshData) {
    auto &triangles = meshData.triangles;
    if (triangles.size() == 0) {
        return;
    }

    m_triangles.reserve(triangles.size());
    m_nodes.reserve(triangles.size() * 2);

    AABB box = triangles.front().getAABB();
    for (auto &tri : triangles) {
        box = AABB(box, tri.getAABB());
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

    auto tris = std::vector<Triangle>(triangles);
    construct_bvh(tris, axis, 0, tris.size());

    // Height Info
    i32 maxHeight = 0;
    i32 minHeight = INT_MAX;
    i32 totalHeight = 0;

    // Leaf Node Info
    i32 leafNodeCount = 0;
    i32 emptyLeaf = 0;
    i32 maxTri = INT_MIN;
    i32 minTri = INT_MAX;

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
                if (!m_triangles[i].inAABB(current.box)) {
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
    glm::ivec2 trianglesUsage = { m_triangles.size() * sizeof(Triangle), m_triangles.size() * 6 * sizeof(glm::vec4) };

    glm::ivec2 bvhUsage = nodesUsage + trianglesUsage;

    const char *gaps = "\t";

    std::cout
        << "\nBVH Constructed Successfully!\n"
        << "\tNodes Count: " << gaps << gaps <<m_nodes.size() << '\n'
            << "\t\tCPU Usage: " << gaps << nodesUsage[0] / 1024.0f << " (KB)\n"
            << "\t\tGPU Usage: " << gaps << nodesUsage[1] / 1024.0f << " (KB)\n"

        << "\tTriangles Count: " << gaps << m_triangles.size() << '\n'
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
        << "\tAvg Triangles Leaf: " << gaps << tris.size() / (f32)leafNodeCount << '\n'
        << "\tEmpty Leaf: " << gaps << gaps << emptyLeaf  << "\n\n";
}

