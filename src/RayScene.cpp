#include "RayScene.h"

#include "glUtilities/TextureBuffer.h"
#include "TraceableObject.h"
#include <glad/glad.h>

void Camera::updateDirection() {
    glm::vec3 direction;
    direction.x = cos(glm::radians(180 + yaw)) * cos(glm::radians(pitch));
    direction.y = sin(glm::radians(pitch));
    direction.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    forward = glm::normalize(direction);
    right = normalize(-glm::cross(forward, glm::vec3(0, 1, 0)));
    up = glm::cross(-right, forward);
}

void RayScene::load_material(i32 materialIndex) {
    auto &material = m_materials[materialIndex];
    m_materialsBuffer.push_back({
            material.emissionColor, material.emissionStrength
        });

    m_materialsBuffer.push_back({
            material.albedo, material.subsurface
        });

    m_materialsBuffer.push_back({
            material.roughness,
            material.metallic,
            material.specular,
            material.specularTint
        });
}

void RayScene::load_objects() {
    for (auto &e : m_traceableObjects) {
        e->write(m_objectsBuffer);
        load_material(e->getMaterialIndex());
    }
}

void RayScene::initialize(Camera &camera) {
    m_camera = camera;
    m_objectsTexBuffer = std::make_unique<gl::TextureBuffer>(
            nullptr, 0, GL_STATIC_DRAW, GL_RGBA32F);

    m_materialsTexBuffer = std::make_unique<gl::TextureBuffer>(
            nullptr, 0, GL_STATIC_DRAW, GL_RGBA32F);
}

void RayScene::submit() {
    m_materialsBuffer.clear();
    m_objectsBuffer.clear();

    load_objects();
    m_objectsTexBuffer->setBuffer(m_objectsBuffer.data(), m_objectsBuffer.size() * sizeof(glm::vec4));
    m_materialsTexBuffer->setBuffer(m_materialsBuffer.data(), m_materialsBuffer.size() * sizeof(glm::vec4));
}

void RayScene::bindObjects(i32 slot) const {
    m_objectsTexBuffer->bind(slot);
}

void RayScene::bindMaterials(i32 slot) const {
    m_materialsTexBuffer->bind(slot);
}

