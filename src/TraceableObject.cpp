#include "TraceableObject.h"

void TraceableObject::writeHeader(std::vector<f32> &buffer) const {
    buffer.push_back(static_cast<f32>(m_type));
    buffer.push_back(static_cast<f32>(m_materialIndex));
}

Sphere::Sphere(GVec3 center, f32 radius)
    : TraceableObject(TraceableType::Sphere)
    , center(center)
    , radius(radius)
{
    boundingBox = AABB(
            center - GVec3(radius),
            center + GVec3(radius)
        );
}

void Sphere::write(std::vector<f32> &buffer) const {
    buffer.insert(buffer.end(), &center.x, &center.x + 3);
    buffer.push_back(radius);
}

bool Sphere::inAABB(const AABB &box) const {
    return (center.x - radius >= box.min.x && center.x + radius <= box.max.x) &&
           (center.y - radius >= box.min.y && center.y + radius <= box.max.y) &&
           (center.z - radius >= box.min.z && center.z + radius <= box.max.z);
}

Quad::Quad(GVec3 q, GVec3 u, GVec3 v, bool cullFace)
    : TraceableObject(TraceableType::Quad)
    , q(q), u(u), v(v), cullFace(cullFace)
{
    GVec3 p0 = q;          // corner
    GVec3 p1 = q + u;      // corner + edge u
    GVec3 p2 = q + v;      // corner + edge v
    GVec3 p3 = q + u + v;  // corner + edge u + edge v
    boundingBox = AABB(
            glm::min(glm::min(p0, p1), glm::min(p2, p3)),
            glm::max(glm::max(p0, p1), glm::max(p2, p3))
        );
}

void Quad::write(std::vector<f32> &buffer) const {
    buffer.insert(buffer.end(), &q.x, &q.x + 3);
    buffer.insert(buffer.end(), &u.x, &u.x + 3);
    buffer.insert(buffer.end(), &v.x, &v.x + 3);
    buffer.push_back(static_cast<f32>(cullFace));
}

bool Quad::inAABB(const AABB &box) const {
    GVec3 p0 = q;          // corner
    GVec3 p1 = q + u;      // corner + edge u
    GVec3 p2 = q + v;      // corner + edge v
    GVec3 p3 = q + u + v;  // corner + edge u + edge v

    GVec3 quadMin = glm::min(glm::min(p0, p1), glm::min(p2, p3));
    GVec3 quadMax = glm::max(glm::max(p0, p1), glm::max(p2, p3));

    return (quadMax.x >= box.min.x && quadMin.x <= box.max.x) &&
           (quadMax.y >= box.min.y && quadMin.y <= box.max.y) &&
           (quadMax.z >= box.min.z && quadMin.z <= box.max.z);
}

Triangle::Triangle(GVec3 posA, GVec3 posB, GVec3 posC, GVec3 normA, GVec3 normB, GVec3 normC)
    : TraceableObject(TraceableType::Triangle)
    , posA(posA), posB(posB), posC(posC)
    , normA(normA), normB(normB), normC(normC)
{
    boundingBox = AABB(
            glm::min(posA, glm::min(posB, posC)),
            glm::max(posA, glm::max(posB, posC))
        );
}

void Triangle::write(std::vector<f32> &buffer) const {
    buffer.insert(buffer.end(), &posA.x, &posA.x + 3);
    buffer.insert(buffer.end(), &posB.x, &posB.x + 3);
    buffer.insert(buffer.end(), &posC.x, &posC.x + 3);
    buffer.insert(buffer.end(), &normA.x, &normA.x + 3);
    buffer.insert(buffer.end(), &normB.x, &normB.x + 3);
    buffer.insert(buffer.end(), &normC.x, &normC.x + 3);
}

bool Triangle::inAABB(const AABB &box) const {
    for (auto v : {posA, posB, posC}) {
        if (v.x < box.min.x || v.x > box.max.x ||
            v.y < box.min.y || v.y > box.max.y ||
            v.z < box.min.z || v.z > box.max.z) {
            return false;
        }
    }
    return true;
}

#include "glUtilities/util.h"

#include <tinygltf/tiny_gltf.h>
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

static void process_node(const tinygltf::Model &model, int nodeIndex, const glm::mat4 &parentTransform, MeshData &meshData) {
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

MeshData Model::LocalMeshData(const tinygltf::TinyGLTF &loader, const tinygltf::Model &model) {
    MeshData meshData;

    int sceneIndex = model.defaultScene > -1 ? model.defaultScene : 0;
    const tinygltf::Scene &scene = model.scenes[sceneIndex];
    for (int nodeIndex : scene.nodes) {
        glm::mat4 transform = glm::mat4(1.0);
        for (int nodeIndex : scene.nodes) {
            process_node(model, nodeIndex, transform, meshData);
        }
    }

    return meshData;
}

Model::Model(const MeshData &meshData)
    : TraceableObject(TraceableType::Model)
    , bvh(meshData)
{
    if (meshData.triangles.size() > 0) {
        boundingBox = meshData.triangles.front().getAABB();
        for (auto &triangle : meshData.triangles) {
            boundingBox = AABB(boundingBox, triangle.getAABB());
        }
    }
}

void Model::write(std::vector<f32> &buffer) const {
    const auto &nodes = bvh.getNodes();
    const auto &triangles = bvh.getTriangles();
    buffer.insert(buffer.end(), &boundingBox.min.x, &boundingBox.min.x + 3);
    buffer.insert(buffer.end(), &boundingBox.max.x, &boundingBox.max.x + 3);
    buffer.push_back(triangles.size());
    buffer.push_back(nodes.size());

    for (auto &node : nodes) {
        buffer.insert(buffer.end(), &node.box.min.x, &node.box.min.x + 3);
        buffer.insert(buffer.end(), &node.box.max.x, &node.box.max.x + 3);
        buffer.push_back(node.leftIndex);
        buffer.push_back(node.rightIndex);
        buffer.push_back(static_cast<bool>(node.isLeaf));
    }
}

bool Model::inAABB(const AABB &box) const {
    GVec3 clampedMin = glm::max(boundingBox.min, box.min);
    GVec3 clampedMax = glm::min(boundingBox.max, box.max);
    return clampedMin == boundingBox.min && clampedMax == boundingBox.max;
}

