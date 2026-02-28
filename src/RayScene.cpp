#include "RayScene.h"

#include "glUtilities/ShaderProgram.h"
#include "TraceableObject.h"

#include <iostream>
#include <glm/glm.hpp>

#include "gpu/FloatBuffer.h"
#include "gpu/TBOStorage.h"

void RayScene::initialize() {
    primitiveBuffer = std::make_unique<gpu::FloatBuffer>();
    primitiveGPUStorage = std::make_unique<gpu::TBOStorage>();

    modelBuffer = std::make_unique<gpu::FloatBuffer>();
    modelGPUStorage = std::make_unique<gpu::TBOStorage>();

    textureBuffer = std::make_unique<gpu::FloatBuffer>();
    textureGPUStorage = std::make_unique<gpu::TBOStorage>();

    materialBuffer = std::make_unique<gpu::FloatBuffer>();
    materialGPUStorage = std::make_unique<gpu::TBOStorage>();

    // Added default material
    Material defaultMat;
    m_materials.push_back(defaultMat);
}

void RayScene::bindShader(gl::ShaderProgram &shader) const {
    // Bind objects
    primitiveGPUStorage->bindToUnit(1);
    shader.setUniform1i("objectsBuffer", 1);

    // Bind materials
    materialGPUStorage->bindToUnit(2);
    shader.setUniform1i("materialsBuffer", 2);

    // Bind models
    modelGPUStorage->bindToUnit(4);
    shader.setUniform1i("modelObjectsBuffer", 4);

    // Bind textures
    textureGPUStorage->bindToUnit(5);
    shader.setUniform1i("texturesBuffer", 5);

}

void RayScene::submit() {
    for (const auto &primitive : m_traceableObjects) {
        primitiveBuffer->push(static_cast<i32>(primitive->getType()));
        primitiveBuffer->push(static_cast<i32>(primitive->getMaterialIndex()));
        primitive->serialize(*primitiveBuffer);
    }
    primitiveGPUStorage->upload(*primitiveBuffer);

    for (const auto &model : m_modelObjects) {
        model->serialize(*modelBuffer);
    }
    modelGPUStorage->upload(*modelBuffer);

    for (const auto &model : m_modelObjects) {
        for (const auto &texture : model->meshData.textures) {
            texture.serialize(*textureBuffer);
        }
    }
    textureGPUStorage->upload(*textureBuffer);

    for (const auto &material : m_materials) {
        material.serialize(*materialBuffer);
    }
    materialGPUStorage->upload(*materialBuffer);
}

void RayScene::setSkyColor(glm::vec3 skyColor) {
    m_skyColor = skyColor;
    m_linearSkyColor = glm::pow(m_skyColor, glm::vec3(2.2));
}

glm::vec3 RayScene::getSkyColor() const {
    return m_skyColor;
}

glm::vec3 RayScene::getLinearSkyColor() const {
    return m_linearSkyColor;
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

    std::cout << "identifiers: " << meshData.identifiers.size() << std::endl;
    std::cout << "\tTriangles Count: " << meshData.identifiers.size() << '\n';
    std::cout << "\tMaterials Count: " << meshData.materials.size() << '\n';
}

void RayScene::addObject(const TraceableObject &obj, const Material &material) {
    m_traceableObjects.push_back(obj.clone());

    auto &object = m_traceableObjects.back();
    object->m_materialIndex = m_materials.size();
    m_materials.push_back(material);
}

void RayScene::addObject(const TraceableObject &obj, int materialIndex) {
    ASSERT(materialIndex >= 0 && materialIndex < m_materials.size());
    m_traceableObjects.push_back(obj.clone());

    auto &object = m_traceableObjects.back();
    object->m_materialIndex = materialIndex;
}

