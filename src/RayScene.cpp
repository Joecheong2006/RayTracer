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

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp> // for glm::mat4_cast

// Build parent map: parent[childIndex] = parentIndex or -1 if root
static std::vector<int> BuildParentMap(const tinygltf::Model &model) {
    std::vector<int> parent(model.nodes.size(), -1);
    for (size_t i = 0; i < model.nodes.size(); ++i) {
        for (int child : model.nodes[i].children) {
            if (child >= 0 && child < (int)model.nodes.size())
                parent[child] = (int)i;
        }
    }
    return parent;
}

static glm::mat4 GetLocalTransform(const tinygltf::Node &node) {
    // If matrix present -> use it (glTF stores matrix in column-major order)
    if (!node.matrix.empty()) {
        glm::mat4 m(1.0f);
        for (int i = 0; i < 16; ++i) {
            // column = i / 4, row = i % 4  => m[column][row]
            m[i / 4][i % 4] = static_cast<float>(node.matrix[i]);
        }
        return m;
    }

    glm::vec3 T(0.0f);
    if (!node.translation.empty())
        T = glm::vec3(
            static_cast<float>(node.translation[0]),
            static_cast<float>(node.translation[1]),
            static_cast<float>(node.translation[2])
        );

    glm::quat R(1.0f, 0.0f, 0.0f, 0.0f); // w,x,y,z
    if (!node.rotation.empty())
        R = glm::quat(
            static_cast<float>(node.rotation[3]), // w
            static_cast<float>(node.rotation[0]), // x
            static_cast<float>(node.rotation[1]), // y
            static_cast<float>(node.rotation[2])  // z
        );

    glm::vec3 S(1.0f);
    if (!node.scale.empty())
        S = glm::vec3(
            static_cast<float>(node.scale[0]),
            static_cast<float>(node.scale[1]),
            static_cast<float>(node.scale[2])
        );

    return glm::translate(glm::mat4(1.0f), T) * glm::mat4_cast(R) * glm::scale(glm::mat4(1.0f), S);
}

// recursive with memoization
static glm::mat4 ComputeWorldTransformCached(
    const tinygltf::Model &model,
    int nodeIndex,
    const std::vector<int> &parentMap,
    std::vector<char> &visited,
    std::vector<glm::mat4> &cache
) {
    if (visited[nodeIndex]) return cache[nodeIndex];

    int p = parentMap[nodeIndex];
    glm::mat4 parentMat = (p == -1) ? glm::mat4(1.0f)
                                    : ComputeWorldTransformCached(model, p, parentMap, visited, cache);

    glm::mat4 local = GetLocalTransform(model.nodes[nodeIndex]);
    cache[nodeIndex] = parentMat * local; // parent * local
    visited[nodeIndex] = 1;
    return cache[nodeIndex];
}

// public helper: returns world transform for nodeIndex
static glm::mat4 GetWorldTransform(const tinygltf::Model &model, int nodeIndex) {
    auto parentMap = BuildParentMap(model);
    std::vector<char> visited(model.nodes.size(), 0);
    std::vector<glm::mat4> cache(model.nodes.size(), glm::mat4(1.0f));
    return ComputeWorldTransformCached(model, nodeIndex, parentMap, visited, cache);
}

void RayScene::load_triangles_gltf(std::vector<Triangle> &triangles, const tinygltf::Model &model, int nodeIndex, const glm::mat4 &parentTransform) {
    const tinygltf::Node &node = model.nodes[nodeIndex];
    const glm::mat4 worldTransform = GetWorldTransform(model, nodeIndex);

    if (node.mesh >= 0) {
        glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(worldTransform)));

        const tinygltf::Mesh &mesh = model.meshes[node.mesh];

        for (const tinygltf::Primitive &primitive : mesh.primitives) {
            if (primitive.mode != TINYGLTF_MODE_TRIANGLES) {
                std::cout << "Only TRIANGLES mode supported\n";
                continue;
            }

            // ---- Load positions ----
            const tinygltf::Accessor &posAccessor = model.accessors[primitive.attributes.at("POSITION")];
            const tinygltf::BufferView &posView = model.bufferViews[posAccessor.bufferView];
            const tinygltf::Buffer &posBuffer = model.buffers[posView.buffer];

            const float* positions = reinterpret_cast<const float*>(
                &posBuffer.data[posView.byteOffset + posAccessor.byteOffset]
            );

            size_t vertexCount = posAccessor.count;

            // Transform all positions once
            std::vector<glm::vec3> worldPositions(vertexCount);
            for (size_t i = 0; i < vertexCount; ++i) {
                worldPositions[i] = glm::vec3(worldTransform *
                    glm::vec4(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2], 1.0f));
            }

            // ---- Load or compute normals ----
            std::vector<glm::vec3> vertexNormals(vertexCount, glm::vec3(0.0f));
            bool hasNormals = primitive.attributes.count("NORMAL") > 0;

            if (hasNormals) {
                const tinygltf::Accessor &normAccessor = model.accessors[primitive.attributes.at("NORMAL")];
                const tinygltf::BufferView &normView = model.bufferViews[normAccessor.bufferView];
                const tinygltf::Buffer &normBuffer = model.buffers[normView.buffer];
                const float* normals = reinterpret_cast<const float*>(
                    &normBuffer.data[normView.byteOffset + normAccessor.byteOffset]
                );
                for (size_t i = 0; i < vertexCount; ++i) {
                    vertexNormals[i] = glm::normalize(normalMatrix *
                        glm::vec3(normals[i * 3 + 0], normals[i * 3 + 1], normals[i * 3 + 2]));
                }
            }

            // ---- Load indices----
            if (primitive.indices < 0) {
                std::cerr << "Primitive has no indices\n";
                continue;
            }

            const tinygltf::Accessor &indexAccessor = model.accessors[primitive.indices];
            const tinygltf::BufferView &indexView = model.bufferViews[indexAccessor.bufferView];
            const tinygltf::Buffer &indexBuffer = model.buffers[indexView.buffer];

            const unsigned char* dataPtr = indexBuffer.data.data() + indexView.byteOffset + indexAccessor.byteOffset;

            std::vector<uint32_t> indices;
            indices.reserve(indexAccessor.count);

            switch (indexAccessor.componentType) {
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: {
                    const uint8_t* buf = reinterpret_cast<const uint8_t*>(dataPtr);
                    for (size_t i = 0; i < indexAccessor.count; ++i) {
                        indices.push_back(static_cast<uint32_t>(buf[i]));
                    }
                    break;
                }
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
                    const uint16_t* buf = reinterpret_cast<const uint16_t*>(dataPtr);
                    for (size_t i = 0; i < indexAccessor.count; ++i) {
                        indices.push_back(static_cast<uint32_t>(buf[i]));
                    }
                    break;
                }
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT: {
                    const uint32_t* buf = reinterpret_cast<const uint32_t*>(dataPtr);
                    for (size_t i = 0; i < indexAccessor.count; ++i) {
                        indices.push_back(buf[i]);
                    }
                    break;
                }
                default:
                    ASSERT(false); // Unsupported index component type
            }

            // ---- If no normals, accumulate face normals ----
            if (!hasNormals) {
                for (size_t tri = 0; tri < indices.size(); tri += 3) {
                    uint32_t i0 = indices[tri + 0];
                    uint32_t i1 = indices[tri + 1];
                    uint32_t i2 = indices[tri + 2];

                    glm::vec3 faceNormal = glm::normalize(glm::cross(
                        worldPositions[i1] - worldPositions[i0],
                        worldPositions[i2] - worldPositions[i0]));

                    vertexNormals[i0] += faceNormal;
                    vertexNormals[i1] += faceNormal;
                    vertexNormals[i2] += faceNormal;
                }
                for (auto &n : vertexNormals) {
                    n = glm::normalize(n);
                }
            }

            // ---- Laod materials ====
            int materialIndex = primitive.material;
            for (size_t tri = 0; tri < indices.size(); tri += 3) {
                uint32_t idx0 = indices[tri + 0];
                uint32_t idx1 = indices[tri + 1];
                uint32_t idx2 = indices[tri + 2];

                Triangle triangle {
                            worldPositions[idx0], worldPositions[idx1], worldPositions[idx2],
                            vertexNormals[idx0], vertexNormals[idx1], vertexNormals[idx2]
                        };
                triangle.m_materialIndex = materialIndex < 0 ? 0 : m_materials.size() + materialIndex;

                triangles.push_back(triangle);
            }
        }
    }

    // Load children
    for (int childIndex : node.children) {
        load_triangles_gltf(triangles, model, childIndex, worldTransform);
    }
}

std::vector<Triangle> RayScene::load_model(std::string modelPath) {
    std::vector<Triangle> result;

    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err;
    std::string warn;

    bool ret = loader.LoadBinaryFromFile(&model, &err, &warn, modelPath);

    if (!warn.empty()) std::cout << "Warn: " << warn << '\n';
    if (!err.empty()) std::cerr << "Err: " << err << '\n';
    if (!ret) {
        std::cerr << "Failed to load " << modelPath << '\n';
        ASSERT(false);
    }

    std::cout << "Load " << modelPath << " successfully!\n";

    const tinygltf::Scene &scene = model.scenes[model.defaultScene > -1 ? model.defaultScene : 0];
    for (int nodeIndex : scene.nodes) {
        load_triangles_gltf(result, model, nodeIndex, glm::mat4(1.0f));
    }

    for (auto &material : model.materials) {
        Material m;
        auto it = material.values.find("baseColorFactor");
        if (it != material.values.end()) {
            m.albedo = {
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
                m.subsurface = static_cast<float>(tIt.GetNumberAsDouble());
            }
        }

        auto itMetal = material.values.find("metallicFactor");
        if (itMetal != material.values.end()) {
            m.metallic = static_cast<float>(itMetal->second.number_value);
        }

        auto itRough = material.values.find("roughnessFactor");
        if (itRough != material.values.end()) {
            m.roughness = static_cast<float>(itRough->second.number_value);
        }

        it = material.additionalValues.find("emissiveFactor");
        if (it != material.additionalValues.end() && it->second.number_array.size() == 3) {
            m.emissionColor = glm::vec3(
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
                m.emissionStrength = static_cast<float>(sIt.GetNumberAsDouble());
            }
        }

        // transmissionFactor (GLTF extension)
        extIt = material.extensions.find("KHR_materials_transmission");
        if (extIt != material.extensions.end()) {
            const tinygltf::Value &ext = extIt->second;
            auto sIt = ext.Get("transmissionFactor");
            if (sIt.IsNumber()) {
                m.transmission = static_cast<float>(sIt.GetNumberAsDouble());
            }
        }

        extIt = material.extensions.find("KHR_materials_ior");
        if (extIt != material.extensions.end()) {
            const tinygltf::Value &ext = extIt->second;
            if (ext.Has("ior")) {
                m.ior = static_cast<float>(ext.Get("ior").Get<double>());
            }
        }

        m_materials.push_back(m);
        load_material(m);
    }

    std::cout << "\tTriangles Count: " << result.size() << '\n';
    std::cout << "\tMaterials Count: " << model.materials.size() << '\n';

    return result;
}

void RayScene::load_material(const Material &material) {
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

    m_materialsBuffer.push_back({
            material.transmission, material.ior, 0, 0
        });
}

void RayScene::initialize() {
    m_objectsTexBuffer = std::make_unique<gl::TextureBuffer>(
            nullptr, 0, GL_STATIC_DRAW, GL_RGBA32F);

    m_modelObjectsTexBuffer = std::make_unique<gl::TextureBuffer>(
            nullptr, 0, GL_STATIC_DRAW, GL_RGBA32F);

    m_materialsTexBuffer = std::make_unique<gl::TextureBuffer>(
            nullptr, 0, GL_STATIC_DRAW, GL_RGBA32F);

    // Added default material
    Material defaultMat;
    m_materials.push_back(defaultMat);
    load_material(defaultMat);
}

void RayScene::submit() {
    m_objectsTexBuffer->setBuffer(m_objectsBuffer.data(), m_objectsBuffer.size() * sizeof(glm::vec4));
    m_modelObjectsTexBuffer->setBuffer(m_modelObjectsBuffer.data(), m_modelObjectsBuffer.size() * sizeof(glm::vec4));
    m_materialsTexBuffer->setBuffer(m_materialsBuffer.data(), m_materialsBuffer.size() * sizeof(glm::vec4));
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

void RayScene::addModel(std::string modelPath) {
    m_traceableObjects.push_back(
                std::make_unique<Model>(load_model(modelPath))
            );
    auto &object = m_traceableObjects.back();
    object->m_materialIndex = 0; // Default material
    object->write(m_objectsBuffer);

    auto model = static_cast<Model&>(*m_traceableObjects.back().get());
    auto &triangles = model.bvh.getTriangles();
    for (auto &triangle : triangles) {
        triangle.write(m_modelObjectsBuffer);
    }
}

