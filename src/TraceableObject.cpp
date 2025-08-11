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

void Quad::write(std::vector<glm::vec4> &buffer) const {
    writeHeader(buffer);
    buffer.push_back({ q, cullFace });
    buffer.push_back({ u, 0 });
    buffer.push_back({ v, 0 });
}

void Triangle::write(std::vector<glm::vec4> &buffer) const {
    writeHeader(buffer);
    buffer.push_back({ posA, 0 });
    buffer.push_back({ posB, 0 });
    buffer.push_back({ posC, 0 });
}

