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

static Material process_material(tinygltf::Model &model, tinygltf::Material &material) {
    Material outMat;

    auto pbr = material.pbrMetallicRoughness;
    auto textureInfo = pbr.baseColorTexture;
    std::cout << material.name << std::endl;
    if (textureInfo.index >= 0) {
        std::cout << "textureInfo.index: " << textureInfo.index << std::endl;
        std::cout << "textureInfo.texCoord: " << textureInfo.texCoord << std::endl;
        auto &tex = model.textures[textureInfo.index];
        std::cout << "tex.source: " << tex.source << std::endl;
        auto &image = model.images[tex.source];
        std::cout << "image.name: " << image.name << std::endl;
        if (!image.uri.empty()) {
            std::cout << "Image URI: " << image.uri << std::endl;
        } else if (image.bufferView >= 0) {
            auto &view = model.bufferViews[image.bufferView];
            auto &buffer = model.buffers[view.buffer];

            const unsigned char* imageData = buffer.data.data() + view.byteOffset;
            size_t imageSize = view.byteLength;

            int width, height, channels;
            unsigned char* pixels = stbi_load_from_memory(
                imageData,
                imageSize,
                &width,
                &height,
                &channels,
                0
            );

            if (pixels) {
                std::cout << "Decoded image: " << width << "x" << height << std::endl;
                stbi_image_free(pixels);
            }

            int internalFormat;
            switch (channels) {
                case 1:
                    internalFormat = GL_R8;
                    std::cout << "internalFormat: GL_R8\n";
                    break;
                case 2:
                    internalFormat = GL_RG8;
                    std::cout << "internalFormat: GL_RG8\n";
                    break;
                case 3:
                    internalFormat = GL_RGB8;
                    std::cout << "internalFormat: GL_RGB8\n";
                    break;
                case 4:
                    internalFormat = GL_RGBA8;
                    std::cout << "internalFormat: GL_RGBR8\n";
                    break;
            }
        } else {
            std::cerr << "No image data available!" << std::endl;
        }
    }

    auto it = material.values.find("baseColorFactor");
    if (it != material.values.end()) {
        outMat.albedo = {
            it->second.number_array[0],
            it->second.number_array[1],
            it->second.number_array[2]
        };
    }

    auto extIt = material.extensions.find("KHR_materials_volume");
    if (extIt != material.extensions.end()) {
        const tinygltf::Value &ext = extIt->second;

        auto tIt = ext.Get("thicknessFactor");
        if (tIt.IsNumber()) {
            outMat.subsurface = static_cast<float>(tIt.GetNumberAsDouble());
        }
    }

    // Default metallic from glTF PBR spec
    outMat.metallic = 1;
    auto itMetal = material.values.find("metallicFactor");
    if (itMetal != material.values.end()) {
        outMat.metallic = static_cast<float>(itMetal->second.number_value);
    }

    // Default roughness from glTF PBR spec
    outMat.roughness = 1;
    auto itRough = material.values.find("roughnessFactor");
    if (itRough != material.values.end()) {
        outMat.roughness = static_cast<float>(itRough->second.number_value);
    }

    it = material.additionalValues.find("emissiveFactor");
    if (it != material.additionalValues.end() && it->second.number_array.size() == 3) {
        outMat.emissionColor = glm::vec3(
            it->second.number_array[0],
            it->second.number_array[1],
            it->second.number_array[2]
        );
    }

    // emissiveStrength  (GLTF extension)
    extIt = material.extensions.find("KHR_materials_emissive_strength");
    if (extIt != material.extensions.end()) {
        const tinygltf::Value &ext = extIt->second;
        auto sIt = ext.Get("emissiveStrength");
        if (sIt.IsNumber()) {
            outMat.emissionStrength = static_cast<float>(sIt.GetNumberAsDouble());
        }
    }

    // transmissionFactor (GLTF extension)
    extIt = material.extensions.find("KHR_materials_transmission");
    if (extIt != material.extensions.end()) {
        const tinygltf::Value &ext = extIt->second;
        auto sIt = ext.Get("transmissionFactor");
        if (sIt.IsNumber()) {
            outMat.transmission = static_cast<float>(sIt.GetNumberAsDouble());
        }
    }

    extIt = material.extensions.find("KHR_materials_ior");
    if (extIt != material.extensions.end()) {
        const tinygltf::Value &ext = extIt->second;
        if (ext.Has("ior")) {
            outMat.ior = static_cast<float>(ext.Get("ior").Get<double>());
        }
    }
    return outMat;
}

MeshData RayScene::load_model(std::string modelPath) {
    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err;
    std::string warn;

    bool ret = loader.LoadBinaryFromFile(&model, &err, &warn, modelPath);

    if (!warn.empty()) std::cout << "Warn: " << warn << '\n';
    if (!err.empty()) std::cerr << "Err: " << err << '\n';
    if (!ret) {
        std::cerr << "Failed to load " << modelPath << '\n';
        std::cout << "You may try the models in res/models.7z" << '\n';
        return {};
    }

    std::cout << "Load " << modelPath << " successfully!\n";

    MeshData meshData = Model::LocalMeshData(loader, model);

    if (meshData.vertices.empty()) {
        return {};
    }

    // Generate triangle
    for (const auto &identifier : meshData.identifiers) {
        u32 idx0 = identifier.indices[0];
        u32 idx1 = identifier.indices[1];
        u32 idx2 = identifier.indices[2];

        Triangle triangle {
                    meshData.vertices[idx0], meshData.vertices[idx1], meshData.vertices[idx2],
                    meshData.normals[idx0], meshData.normals[idx1], meshData.normals[idx2]
                };
        triangle.m_materialIndex = model.materials.size() > 0 ? m_materials.size() + identifier.mateiralsIndices : 0;
        meshData.triangles.push_back(triangle);
    }

    std::cout << "identifiers: " << meshData.identifiers.size() << std::endl;

    for (auto &material : model.materials) {
        Material m = process_material(model, material);
        m_materials.push_back(m);
        load_material(m);
    }

    std::cout << "\tTriangles Count: " << meshData.triangles.size() << '\n';
    std::cout << "\tMaterials Count: " << model.materials.size() << '\n';

    return meshData;
}

void RayScene::load_material(const Material &material) {
    m_materialsBuffer.insert(m_materialsBuffer.end(),
            reinterpret_cast<const f32*>(&material),
            reinterpret_cast<const f32*>(&material) + sizeof(Material) / sizeof(f32));
}

void RayScene::initialize() {
    m_objectsTexBuffer = std::make_unique<gl::TextureBuffer>(
            nullptr, 0, GL_STATIC_DRAW, GL_R32F);

    m_modelMeshesDataTexBuffer = std::make_unique<gl::TextureBuffer>(
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
    m_modelMeshesDataTexBuffer->setBuffer(m_modelMeshesDataBuffer.data(), m_modelMeshesDataBuffer.size() * sizeof(f32));
    m_modelObjectsTexBuffer->setBuffer(m_modelObjectsBuffer.data(), m_modelObjectsBuffer.size() * sizeof(f32));
    m_materialsTexBuffer->setBuffer(m_materialsBuffer.data(), m_materialsBuffer.size() * sizeof(f32));
}

void RayScene::bindObjects(i32 slot) const {
    m_objectsTexBuffer->bind(slot);
}

void RayScene::bindModelMeshesData(i32 slot) const {
    m_modelMeshesDataTexBuffer->bind(slot);
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

void RayScene::addModel(std::string modelPath) {
    MeshData tris = load_model(modelPath);
    if (tris.triangles.empty()) {
        return;
    }

    m_modelObjects.push_back(std::make_unique<Model>(tris));

    auto &model = m_modelObjects.back();
    model->write(m_modelObjectsBuffer);

    auto &buffer = m_modelMeshesDataBuffer;
    auto &triangles = model->bvh.getTriangles();
    for (auto &triangle : triangles) {
        buffer.push_back(static_cast<f32>(triangle.m_materialIndex));
        triangle.write(buffer);
    }
}

