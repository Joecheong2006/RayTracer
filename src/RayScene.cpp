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

static void load_meshdata_texture(std::vector<f32> &buffer, MeshData::Texture &texture) {
    buffer.push_back(texture.width);
    buffer.push_back(texture.height);
    buffer.push_back(texture.channels);

    buffer.push_back(texture.wrapS);
    buffer.push_back(texture.wrapT);
    buffer.insert(buffer.end(), texture.data.begin(), texture.data.end());
}

// TODO: changed this to static hepler function
void RayScene::load_material(const Material &material) {
    // Load material texture
    m_materialTexturesBuffer.push_back(material.texture.normalTexture);
    m_materialTexturesBuffer.push_back(material.texture.baseColorTexture);
    m_materialTexturesBuffer.push_back(material.texture.metallicRoughnessTexture);
    m_materialTexturesBuffer.push_back(material.texture.emissiveTexture);
    m_materialTexturesBuffer.push_back(material.texture.transmissionTexture);
    m_materialTexturesBuffer.push_back(material.texture.occlusionTexture);

    // Load material
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

    m_materialsBuffer.push_back(material.alphaCut);
    m_materialsBuffer.push_back(material.normalScale);
    m_materialsBuffer.push_back(material.occlusionStrength);
}

void RayScene::initialize() {
    m_objectsTexBuffer = std::make_unique<gl::TextureBuffer>(
            nullptr, 0, GL_STATIC_DRAW, GL_R32F);

    m_modelObjectsTexBuffer = std::make_unique<gl::TextureBuffer>(
            nullptr, 0, GL_STATIC_DRAW, GL_R32F);

    m_texturesTexBuffer = std::make_unique<gl::TextureBuffer>(
            nullptr, 0, GL_STATIC_DRAW, GL_R32F);

    m_materialTexturesTexBuffer = std::make_unique<gl::TextureBuffer>(
            nullptr, 0, GL_STATIC_DRAW, GL_R32I);

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
    m_texturesTexBuffer->setBuffer(m_texturesBuffer.data(), m_texturesBuffer.size() * sizeof(f32));
    m_materialTexturesTexBuffer->setBuffer(m_materialTexturesBuffer.data(), m_materialTexturesBuffer.size() * sizeof(i32));
    m_materialsTexBuffer->setBuffer(m_materialsBuffer.data(), m_materialsBuffer.size() * sizeof(f32));
}

void RayScene::bindObjects(i32 slot) const {
    m_objectsTexBuffer->bind(slot);
}

void RayScene::bindModelObjects(i32 slot) const {
    m_modelObjectsTexBuffer->bind(slot);
}

void RayScene::bindTextures(i32 slot) const {
    m_texturesTexBuffer->bind(slot);
}

void RayScene::bindMaterials(i32 slot) const {
    m_materialsTexBuffer->bind(slot);
}

void RayScene::bindMaterialTextures(i32 slot) const {
    m_materialTexturesTexBuffer->bind(slot);
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

    std::vector<i32> indexLocationMap(meshData.textures.size());
    for (size_t i = 0; i < indexLocationMap.size(); ++i) {
        indexLocationMap[i] = m_texturesBuffer.size();
        load_meshdata_texture(m_texturesBuffer, meshData.textures[i]);
    }

    for (auto &material : meshData.materials) {
        if (material.texture.normalTexture != -1) {
            std::cout << "Load normalTexture index: " << material.texture.normalTexture;
            material.texture.normalTexture
                = indexLocationMap[material.texture.normalTexture];
            std::cout << " at " << material.texture.normalTexture << std::endl;
        }

        if (material.texture.baseColorTexture != -1) {
            std::cout << "Load baseColorTexture index: " << material.texture.baseColorTexture;
            material.texture.baseColorTexture
                = indexLocationMap[material.texture.baseColorTexture];
            std::cout << " at " << material.texture.baseColorTexture << std::endl;
        }

        if (material.texture.metallicRoughnessTexture != -1) {
            std::cout << "Load metallicRoughnessTexture index: " << material.texture.metallicRoughnessTexture;
            material.texture.metallicRoughnessTexture
                = indexLocationMap[material.texture.metallicRoughnessTexture];
            std::cout << " at " << material.texture.metallicRoughnessTexture << std::endl;
        }

        if (material.texture.emissiveTexture != -1) {
            std::cout << "Load emissiveTexture index: " << material.texture.emissiveTexture;
            material.texture.emissiveTexture
                = indexLocationMap[material.texture.emissiveTexture];
            std::cout << " at " << material.texture.emissiveTexture << std::endl;
        }

        if (material.texture.transmissionTexture != -1) {
            std::cout << "Load transmissionTexture index: " << material.texture.transmissionTexture;
            material.texture.transmissionTexture
                = indexLocationMap[material.texture.transmissionTexture];
            std::cout << " at " << material.texture.transmissionTexture << std::endl;
        }

        if (material.texture.occlusionTexture != -1) {
            std::cout << "Load occlusionTexture index: " << material.texture.occlusionTexture;
            material.texture.occlusionTexture
                = indexLocationMap[material.texture.occlusionTexture];
            std::cout << " at " << material.texture.occlusionTexture << std::endl;
        }

        load_material(material);
    }
    m_materials.insert(m_materials.end(), meshData.materials.begin(), meshData.materials.end());

    model->write(m_modelObjectsBuffer);

    std::cout << "identifiers: " << meshData.identifiers.size() << std::endl;
    std::cout << "\tTriangles Count: " << meshData.identifiers.size() << '\n';
    std::cout << "\tMaterials Count: " << meshData.materials.size() << '\n';
}

