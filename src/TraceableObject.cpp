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

