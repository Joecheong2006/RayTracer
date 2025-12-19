#include "MeshData.h"

#include "glUtilities/util.h"

#include <tinygltf/tiny_gltf.h>
#include <tinygltf/stb_image.h>
#include <glm/gtc/type_ptr.hpp>
#include <glad/glad.h>
#include <iostream>

static glm::mat4 get_local_transform(const tinygltf::Node& node) {
    glm::mat4 transform(1.0f);
    
    if (node.matrix.size() == 16) {
        transform = glm::make_mat4(node.matrix.data());
    } else {
        if (node.translation.size() == 3) {
            transform = glm::translate(transform, glm::vec3(
                node.translation[0],
                node.translation[1],
                node.translation[2]
            ));
        }
        
        if (node.rotation.size() == 4) {
            glm::quat rotation(
                node.rotation[3],
                node.rotation[0],
                node.rotation[1],
                node.rotation[2]
            );
            transform *= glm::mat4_cast(rotation);
        }
        
        if (node.scale.size() == 3) {
            transform = glm::scale(transform, glm::vec3(
                node.scale[0],
                node.scale[1],
                node.scale[2]
            ));
        }
    }
    
    return transform;
}

static void load_mesh_data_gltf(MeshData &meshData, const tinygltf::Primitive &primitive, const tinygltf::Model &model, const glm::mat4 &worldTransform) {
    glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(worldTransform)));

    // ================= Load positions =================
    const tinygltf::Accessor &accessor = model.accessors[primitive.attributes.at("POSITION")];
    const tinygltf::BufferView &view = model.bufferViews[accessor.bufferView];
    const tinygltf::Buffer &buffer = model.buffers[view.buffer];
    size_t vertexCount = accessor.count;

    const float* positions = reinterpret_cast<const float*>(
        buffer.data.data() + view.byteOffset + accessor.byteOffset
    );

    // Reserve to vertexCount
    size_t verticesOffset = meshData.vertices.size();
    meshData.vertices.reserve(verticesOffset + vertexCount);

    for (size_t i = 0; i < vertexCount; ++i) {
        // Transform all positions once
        meshData.vertices.push_back(GVec3(worldTransform *
            glm::vec4(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2], 1.0f)));
    }

    // ================= Load normals =================
    meshData.normals.reserve(meshData.normals.size() + vertexCount);
    if (primitive.attributes.find("NORMAL") != primitive.attributes.end()) {
        const tinygltf::Accessor &accessor = model.accessors[primitive.attributes.at("NORMAL")];
        const tinygltf::BufferView &view = model.bufferViews[accessor.bufferView];
        const tinygltf::Buffer &buffer = model.buffers[view.buffer];
        const float* normals = reinterpret_cast<const float*>(
            buffer.data.data() + view.byteOffset + accessor.byteOffset
        );
        for (size_t i = 0; i < vertexCount; ++i) {
            meshData.normals.push_back(glm::normalize(normalMatrix *
                glm::vec3(normals[i * 3 + 0], normals[i * 3 + 1], normals[i * 3 + 2])));
        }
    }
    else {
        // Default normals if not provided
        for (size_t i = 0; i < vertexCount; ++i) {
            meshData.normals.push_back(glm::normalize(normalMatrix * glm::vec3(0, 1, 0)));
        }
    }

    // ================= Load indices =================
    if (primitive.indices >= 0) {
        const tinygltf::Accessor &accessor = model.accessors[primitive.indices];
        const tinygltf::BufferView &view = model.bufferViews[accessor.bufferView];
        const tinygltf::Buffer &buffer = model.buffers[view.buffer];

        const unsigned char* dataPtr = buffer.data.data() + view.byteOffset + accessor.byteOffset;

        u32 indicesCount = accessor.count / 3;
        meshData.identifiers.reserve(meshData.identifiers.size() + indicesCount);
        int materialIndex = primitive.material < 0 ? 0 : primitive.material;

        // ================= Generate identifiers =================
        for (u32 i = 0; i < indicesCount; ++i) {
            GVec3I index;
            switch (accessor.componentType) {
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: {
                    const u8* buf = reinterpret_cast<const uint8_t*>(dataPtr);
                    index = GVec3I(buf[i * 3 + 0], buf[i * 3 + 1], buf[i * 3 + 2]);
                    break;
                }
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
                    const u16* buf = reinterpret_cast<const uint16_t*>(dataPtr);
                    index = GVec3I(buf[i * 3 + 0], buf[i * 3 + 1], buf[i * 3 + 2]);
                    break;
                }
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT: {
                    const u32* buf = reinterpret_cast<const uint32_t*>(dataPtr);
                    index = GVec3I(buf[i * 3 + 0], buf[i * 3 + 1], buf[i * 3 + 2]);
                    break;
                }
                default:
                    ASSERT(false); // Unsupported index component type
            }
            index += GVec3I(verticesOffset);
            meshData.identifiers.push_back({ index, materialIndex });
        }
    }
}

inline static void process_node(const tinygltf::Model &model, int nodeIndex, const glm::mat4 &parentTransform, MeshData &meshData) {
    const tinygltf::Node &node = model.nodes[nodeIndex];
    glm::mat4 localTransform = get_local_transform(node);
    glm::mat4 worldTransform = parentTransform * localTransform;

    if (node.mesh >= 0) {
        const tinygltf::Mesh &mesh = model.meshes[node.mesh];
        for (const tinygltf::Primitive &primitive : mesh.primitives) {
            if (primitive.mode != TINYGLTF_MODE_TRIANGLES) {
                std::cout << "Only TRIANGLES mode supported\n";
            }
            load_mesh_data_gltf(meshData, primitive, model, worldTransform);
        }
    }

    for (int childIndex : node.children) {
        process_node(model, childIndex, worldTransform, meshData);
    }
}

static Material process_material(const tinygltf::Model &model, const tinygltf::Material &material) {
    Material outMat;

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

MeshData MeshData::LoadMeshData(std::string modelPath) {
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
    MeshData meshData;

    int sceneIndex = model.defaultScene > -1 ? model.defaultScene : 0;
    const tinygltf::Scene &scene = model.scenes[sceneIndex];
    for (int nodeIndex : scene.nodes) {
        glm::mat4 transform = glm::mat4(1.0);
        for (int nodeIndex : scene.nodes) {
            process_node(model, nodeIndex, transform, meshData);
        }
    }

    meshData.materials.reserve(model.materials.size());
    for (const auto &material : model.materials) {
        meshData.materials.push_back(process_material(model, material));
    }

    return meshData;
}
