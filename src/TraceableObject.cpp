#include "TraceableObject.h"

#include "glUtilities/util.h"

#include <tinygltf/tiny_gltf.h>
#include <tinygltf/stb_image.h>
#include <glm/gtc/type_ptr.hpp>
#include <glad/glad.h>
#include <iostream>

#include <stack>

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

MeshData Model::LoadMeshData(std::string modelPath) {
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

Model::Model(std::string modelPath)
    : TraceableObject(TraceableType::Model)
    , meshData(Model::LoadMeshData(modelPath))
    , bvh(meshData)
{
    if (meshData.identifiers.size() > 0) {
        boundingBox = MeshData::GetTriangleFromIdentifier(meshData, 0).getAABB();
        for (int i = 1; i < meshData.identifiers.size(); ++i) {
            boundingBox = AABB(boundingBox, MeshData::GetTriangleFromIdentifier(meshData, i).getAABB());
        }
    }

    // Height Info
    i32 maxHeight = 0;
    i32 minHeight = INT_MAX;
    i32 totalHeight = 0;

    // Leaf Node Info
    i32 leafNodeCount = 0;
    i32 emptyLeaf = 0;
    i32 maxTri = INT_MIN;
    i32 minTri = INT_MAX;

    auto &m_nodes = bvh.getNodes();

    std::stack<glm::ivec2> s;
    s.push({ 0, 1 });
    while (!s.empty()) {
        auto track = s.top();
        auto currentIndex = track[0];
        s.pop();

        maxHeight = std::max(maxHeight, track[1]);
        minHeight = std::min(minHeight, track[1]);

        auto current = m_nodes[currentIndex];
        if (current.isLeaf) {
            leafNodeCount++;
            emptyLeaf += current.leftIndex == current.rightIndex;
            totalHeight += track[1];

            maxTri = std::max(maxTri, current.rightIndex - current.leftIndex);
            minTri = std::min(minTri, current.rightIndex - current.leftIndex);

            for (i32 i = current.leftIndex; i < current.rightIndex; ++i) {
                if (!MeshData::GetTriangleFromIdentifier(meshData, i).inAABB(current.box)) {
                    std::cout << "Invalid BVH\n";
                    return;
                }
            }
            continue;
        }

        s.push({ current.rightIndex, track[1] + 1 });
        s.push({ current.leftIndex, track[1] + 1 });
    }

    glm::ivec2 nodesUsage = { m_nodes.size() * sizeof(BVHNode), m_nodes.size() * 3 * sizeof(glm::vec4) };
    glm::ivec2 trianglesUsage = { meshData.identifiers.size() * sizeof(Triangle) / 3, meshData.identifiers.size() * 6 * sizeof(glm::vec4) };

    glm::ivec2 bvhUsage = nodesUsage + trianglesUsage;

    const char *gaps = "\t";

    std::cout
        << "\nBVH Constructed Successfully!\n"
        << "\tNodes Count: " << gaps << gaps <<m_nodes.size() << '\n'
            << "\t\tCPU Usage: " << gaps << nodesUsage[0] / 1024.0f << " (KB)\n"
            << "\t\tGPU Usage: " << gaps << nodesUsage[1] / 1024.0f << " (KB)\n"

        << "\tTriangles Count: " << gaps << meshData.identifiers.size() << '\n'
            << "\t\tCPU Usage: " << gaps << trianglesUsage[0] / 1024.0f << " (KB)\n"
            << "\t\tGPU Usage: " << gaps << trianglesUsage[1] / 1024.0f << " (KB)\n"

        << "\tBVH Memory Usage: " << '\n'
            << "\t\tCPU Usage: "  << gaps << bvhUsage[0] / 1024.0f<< " (KB)\n"
            << "\t\tGPU Usage: "  << gaps << bvhUsage[1] / 1024.0f << " (KB)\n"

        << "\tBVH Min Height: " << gaps << minHeight << '\n'
        << "\tBVH Max Height: " << gaps << maxHeight << '\n'
        << "\tBVH Avg Height: " << gaps << totalHeight / (f32)leafNodeCount << '\n'

        << "\tMin Triangles Leaf: " << gaps << minTri << '\n'
        << "\tMax Triangles Leaf: " << gaps << maxTri << '\n'
        << "\tAvg Triangles Leaf: " << gaps << meshData.identifiers.size() / (f32)leafNodeCount << '\n'
        << "\tEmpty Leaf: " << gaps << gaps << emptyLeaf  << "\n\n";
}

void Model::write(std::vector<f32> &buffer) const {
    const auto &nodes = bvh.getNodes();
    buffer.push_back(meshData.identifiers.size());
    buffer.push_back(meshData.vertices.size());
    buffer.push_back(nodes.size());

    std::cout << "Wrote nodesCount: " << nodes.size() << std::endl;
    std::cout << "Wrote verticesCount: " << meshData.vertices.size() << std::endl;
    std::cout << "Wrote identifiersCount: " << meshData.identifiers.size() << std::endl;
    for (const auto &node : nodes) {
        buffer.insert(buffer.end(), &node.box.min.x, &node.box.min.x + 3);
        buffer.insert(buffer.end(), &node.box.max.x, &node.box.max.x + 3);
        buffer.push_back(node.leftIndex);
        buffer.push_back(node.rightIndex);
        buffer.push_back(static_cast<bool>(node.isLeaf));
    }

    for (const auto &iden : meshData.identifiers) {
        buffer.insert(buffer.end(), &iden.indices.x, &iden.indices.x + 3);
        buffer.push_back(iden.materialIndex);
    }

    for (const auto &vertex : meshData.vertices) {
        buffer.insert(buffer.end(), &vertex.x, &vertex.x + 3);
    }

    for (const auto &normal : meshData.normals) {
        buffer.insert(buffer.end(), &normal.x, &normal.x + 3);
    }
}

bool Model::inAABB(const AABB &box) const {
    GVec3 clampedMin = glm::max(boundingBox.min, box.min);
    GVec3 clampedMax = glm::min(boundingBox.max, box.max);
    return clampedMin == boundingBox.min && clampedMax == boundingBox.max;
}

