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

static void LoadWorldSpaceTriangle(std::vector<Triangle> &triangles, const tinygltf::Model &model, int nodeIndex) {
    const tinygltf::Node &node = model.nodes[nodeIndex];

    if (node.mesh < 0) {
        std::cout << "Node has no mesh\n";
        return;
    }

    glm::mat4 worldTransform = GetWorldTransform(model, nodeIndex);

    const tinygltf::Mesh &mesh = model.meshes[node.mesh];

    for (const tinygltf::Primitive &primitive : mesh.primitives) {
        if (primitive.mode != TINYGLTF_MODE_TRIANGLES) {
            std::cout << "Only TRIANGLES mode supported\n";
            continue;
        }

        // Positions
        const tinygltf::Accessor &posAccessor = model.accessors[primitive.attributes.at("POSITION")];
        const tinygltf::BufferView &posView = model.bufferViews[posAccessor.bufferView];
        const tinygltf::Buffer &posBuffer = model.buffers[posView.buffer];

        const float* positions = reinterpret_cast<const float*>(
            &posBuffer.data[posView.byteOffset + posAccessor.byteOffset]
        );

        // Indices
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

        // Print triangles in world space
        for (size_t tri = 0; tri < indices.size(); tri += 3) {
            uint32_t idx0 = indices[tri + 0];
            uint32_t idx1 = indices[tri + 1];
            uint32_t idx2 = indices[tri + 2];

            glm::vec4 v0 = worldTransform * glm::vec4(
                positions[idx0 * 3 + 0], positions[idx0 * 3 + 1], positions[idx0 * 3 + 2], 1.0f);
            glm::vec4 v1 = worldTransform * glm::vec4(
                positions[idx1 * 3 + 0], positions[idx1 * 3 + 1], positions[idx1 * 3 + 2], 1.0f);
            glm::vec4 v2 = worldTransform * glm::vec4(
                positions[idx2 * 3 + 0], positions[idx2 * 3 + 1], positions[idx2 * 3 + 2], 1.0f);
            triangles.push_back(Triangle{ v0, v1, v2 });
        }
    }
}

std::vector<Triangle> RayScene::LoadModel(std::string modelPath) {
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

    LoadWorldSpaceTriangle(result, model, 0);
    
    std::cout << "\tTriangle Count: " << result.size() << '\n';

    return result;
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
        i32 materialIndex = e->getMaterialIndex();

        // Material uses 3 vec4 buffers
        if (materialIndex * 3 >= m_materialsBuffer.size())
            load_material(materialIndex);
    }
}

void RayScene::initialize(const RayCamera &camera) {
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

void RayScene::setSkyColor(glm::vec3 skyColor) {
    m_skyColor = skyColor;
}

glm::vec3 RayScene::getSkyColor() const {
    return m_skyColor;
}

