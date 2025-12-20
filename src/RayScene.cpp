#include "RayScene.h"

#include "glUtilities/TextureBuffer.h"
#include "TraceableObject.h"
#include <glad/glad.h>

#include <iostream>

void RayCamera::updateDirection() {
    glm::vec3 direction;
    direction.x = cos(glm::radians(180 + yaw)) * cos(glm::radians(pitch));
    direction.y = sin(glm::radians(pitch));
    direction.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    forward = glm::normalize(direction);
    right = normalize(-glm::cross(forward, glm::vec3(0, 1, 0)));
    up = glm::cross(-right, forward);
}

#include <tinygltf/tiny_gltf.h>
#include <tinygltf/stb_image.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp> // for glm::mat4_cast

#include <glm/gtc/type_ptr.hpp>

void RayScene::load_material(const Material &material) {
    m_materialsBuffer.insert(m_materialsBuffer.end(),
            &material.emissionColor.x, &material.emissionColor.x + 3);
    m_materialsBuffer.push_back(material.emissionStrength);
    m_materialsBuffer.insert(m_materialsBuffer.end(),
            &material.albedo.x, &material.albedo.x + 3);
    m_materialsBuffer.push_back(material.subsurface);
    m_materialsBuffer.push_back(material.roughness);
    m_materialsBuffer.push_back(material.metallic);
    m_materialsBuffer.push_back(material.specular);
    m_materialsBuffer.push_back(material.specularTint);
    m_materialsBuffer.push_back(material.transmission);
    m_materialsBuffer.push_back(material.ior);
}

void RayScene::initialize() {
    m_objectsTexBuffer = std::make_unique<gl::TextureBuffer>(
            nullptr, 0, GL_STATIC_DRAW, GL_R32F);

    m_modelObjectsTexBuffer = std::make_unique<gl::TextureBuffer>(
            nullptr, 0, GL_STATIC_DRAW, GL_R32F);

    m_materialsTexBuffer = std::make_unique<gl::TextureBuffer>(
            nullptr, 0, GL_STATIC_DRAW, GL_R32F);

    // Added default material
    Material defaultMat;
    m_materials.push_back(defaultMat);
    load_material(defaultMat);
}

void RayScene::submit() {
    m_objectsTexBuffer->setBuffer(m_objectsBuffer.data(), m_objectsBuffer.size() * sizeof(f32));
    m_modelObjectsTexBuffer->setBuffer(m_modelObjectsBuffer.data(), m_modelObjectsBuffer.size() * sizeof(f32));
    m_materialsTexBuffer->setBuffer(m_materialsBuffer.data(), m_materialsBuffer.size() * sizeof(f32));
}

void RayScene::bindObjects(i32 slot) const {
    m_objectsTexBuffer->bind(slot);
}

void RayScene::bindModelObjects(i32 slot) const {
    m_modelObjectsTexBuffer->bind(slot);
}

void RayScene::bindMaterials(i32 slot) const {
    m_materialsTexBuffer->bind(slot);
}

void RayScene::setSkyColor(glm::vec3 skyColor) {
    m_skyColor = skyColor;
}

glm::vec3 RayScene::getSkyColor() const {
    return m_skyColor;
}

void RayScene::addModel(const std::string &modelPath) {
    {
        Model localModel = Model(modelPath);

        if (localModel.meshData.vertices.empty()) {
            return;
        }

        m_modelObjects.push_back(std::make_unique<Model>(std::move(localModel)));
    }
    auto &model = m_modelObjects.back();
    MeshData &meshData = model->meshData;

    // Compute unqiue material index
    for (auto &identifier : meshData.identifiers) {
        identifier.materialIndex = 
            meshData.materials.size() > 0 ? m_materials.size() + identifier.materialIndex : 0;
    }

    m_materials.insert(m_materials.end(), meshData.materials.begin(), meshData.materials.end());
    for (auto &material : meshData.materials) {
        load_material(material);
    }

    model->write(m_modelObjectsBuffer);

    std::cout << "identifiers: " << meshData.identifiers.size() << std::endl;
    std::cout << "\tTriangles Count: " << meshData.identifiers.size() << '\n';
    std::cout << "\tMaterials Count: " << meshData.materials.size() << '\n';
}

