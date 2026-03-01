#include "RayScene.h"

#include "glUtilities/ShaderProgram.h"
#include "TraceableObject.h"

#include <iostream>
#include <glm/glm.hpp>

#include "gpu/FloatBuffer.h"
#include "gpu/TBOStorage.h"

void RayScene::initialize() {
    primitiveBuffer = std::make_unique<gpu::FloatBuffer>();
    modelBuffer = std::make_unique<gpu::FloatBuffer>();
    textureBuffer = std::make_unique<gpu::FloatBuffer>();
    materialBuffer = std::make_unique<gpu::FloatBuffer>();

    primitiveGPUStorage = std::make_unique<gpu::TBOStorage>();
    modelGPUStorage = std::make_unique<gpu::TBOStorage>();
    textureGPUStorage = std::make_unique<gpu::TBOStorage>();
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

std::string RayScene::genShaderCode() {
    return R"(
struct AABB {
    vec3 min, max;
};

struct TextureInfo {
    int width, height, channels, wrapS, wrapT, index;
};

struct Sphere {
    float radius;
    vec3 center;
    int materialIndex;
};

struct Quad {
    vec3 q, u, v;
    bool cullFace;
    int materialIndex;
};

struct Triangle {
    vec3[3] vertices, normals;
    vec2[3] UVs;
    int materialIndex;
};

struct BVHNode {
    AABB boundingBox;
    int leftIndex, rightIndex;
    bool isLeaf;
};

struct Identifier {
    ivec3 index;
    int materialIndex;
};

struct Model {
    int identifiersCount, verticesCount, UVsCount, nodesCount;
    int materialIndex;
};

uniform samplerBuffer objectsBuffer;
uniform samplerBuffer modelObjectsBuffer;
uniform samplerBuffer texturesBuffer;
uniform samplerBuffer materialsBuffer;

float srgb_to_linear_c(float c) {
    if (c <= 0.04045)
        return c / 12.92;
    else
        return pow((c + 0.055) / 1.055, 2.4);
}

vec3 srgb_to_linear(vec3 srgb) {
    return vec3(srgb_to_linear_c(srgb.x), srgb_to_linear_c(srgb.y), srgb_to_linear_c(srgb.z));
}

// === Buffer Utilities ===
float samplerLoadFloat(samplerBuffer buffer, inout int index) {
    float x = texelFetch(buffer, index).r;
    index++;
    return x;
}

int samplerLoadFloatInt(samplerBuffer buffer, inout int index) {
    float x = samplerLoadFloat(buffer, index);
    return floatBitsToInt(x);
}

ivec3 vec3BitsToIVec3(vec3 v) {
    return ivec3(
            floatBitsToInt(v.x),
            floatBitsToInt(v.y),
            floatBitsToInt(v.z)
        );
}

vec2 samplerLoadVec2(samplerBuffer buffer, inout int index) {
    float x = texelFetch(buffer, index + 0).r;
    float y = texelFetch(buffer, index + 1).r;
    index += 2;
    return vec2(x, y);
}

vec3 samplerLoadVec3(samplerBuffer buffer, inout int index) {
    float x = texelFetch(buffer, index + 0).r;
    float y = texelFetch(buffer, index + 1).r;
    float z = texelFetch(buffer, index + 2).r;
    index += 3;
    return vec3(x, y, z);
}

vec3[3] loadVec3FromIndices(samplerBuffer buffer, ivec3 indices, int offset) {
    vec3 result[3];
    int index = offset + indices[0] * 3;
    result[0] = samplerLoadVec3(buffer, index);
    index = offset + indices[1] * 3;
    result[1] = samplerLoadVec3(buffer, index);
    index = offset + indices[2] * 3;
    result[2] = samplerLoadVec3(buffer, index);
    return result;
}

vec2[3] loadVec2FromIndices(samplerBuffer buffer, ivec3 indices, int offset) {
    vec2 result[3];
    int index = offset + indices[0] * 2;
    result[0] = samplerLoadVec2(buffer, index);
    index = offset + indices[1] * 2;
    result[1] = samplerLoadVec2(buffer, index);
    index = offset + indices[2] * 2;
    result[2] = samplerLoadVec2(buffer, index);
    return result;
}

// TextureInfo
TextureInfo loadTextureInfo(int textureInfoIndex) {
    TextureInfo info;
    info.width = samplerLoadFloatInt(texturesBuffer, textureInfoIndex);
    info.height = samplerLoadFloatInt(texturesBuffer, textureInfoIndex);
    info.channels = samplerLoadFloatInt(texturesBuffer, textureInfoIndex);
    info.wrapS = samplerLoadFloatInt(texturesBuffer, textureInfoIndex);
    info.wrapT  = samplerLoadFloatInt(texturesBuffer, textureInfoIndex);
    info.index = textureInfoIndex;
    return info;
}

vec2 getTextureUV(in TextureInfo info, vec2 uv) {
    // Apply horizontal wrap (S/U)
    if (info.wrapS == 10497) {
        // REPEAT
        uv.x = fract(uv.x);
    } else if (info.wrapS == 33071) {
        // CLAMP_TO_EDGE
        uv.x = clamp(uv.x, 0.0, 1.0);
    } else if (info.wrapS == 33648) {
        // MIRRORED_REPEAT
        float t = fract(uv.x * 0.5) * 2.0;
        uv.x = t > 1.0 ? 2.0 - t : t;
    }

    // Apply vertical wrap (T/V)
    if (info.wrapT == 10497) {
        // REPEAT
        uv.y = fract(uv.y);
    } else if (info.wrapT == 33071) {
        // CLAMP_TO_EDGE
        uv.y = clamp(uv.y, 0.0, 1.0);
    } else if (info.wrapT == 33648) {
        // MIRRORED_REPEAT
        float t = fract(uv.y * 0.5) * 2.0;
        uv.y = t > 1.0 ? 2.0 - t : t;
    }

    return clamp(uv, vec2(0.0), vec2(0.999999));
}

int getTextureItemIndex(in TextureInfo info, vec2 uv) {
    return info.index + (int(uv.x * info.width) + int(uv.y * info.height) * info.width) * info.channels;
}

// Material
Material loadMaterial(int materialIndex) {
    Material result;

    // Offset = (byteof Material / byteof f32) -> how many floats from Material
    int offset = materialIndex * (92 / 4);
    result.texture.normalTexture = samplerLoadFloatInt(materialsBuffer, offset);
    result.texture.baseColorTexture = samplerLoadFloatInt(materialsBuffer, offset);
    result.texture.metallicRoughnessTexture = samplerLoadFloatInt(materialsBuffer, offset);
    result.texture.emissiveTexture = samplerLoadFloatInt(materialsBuffer, offset);
    result.texture.transmissionTexture = samplerLoadFloatInt(materialsBuffer, offset);
    result.texture.occlusionTexture = samplerLoadFloatInt(materialsBuffer, offset);

    result.emissionColor = samplerLoadVec3(materialsBuffer, offset);
    result.emissionStrength = samplerLoadFloat(materialsBuffer, offset);

    result.albedo = samplerLoadVec3(materialsBuffer, offset);
    result.subsurface = samplerLoadFloat(materialsBuffer, offset);

    result.roughness = samplerLoadFloat(materialsBuffer, offset);
    result.metallic = samplerLoadFloat(materialsBuffer, offset);
    result.specular = samplerLoadFloat(materialsBuffer, offset);
    result.specularTint = samplerLoadFloat(materialsBuffer, offset);

    result.transmission = samplerLoadFloat(materialsBuffer, offset);
    result.ior = samplerLoadFloat(materialsBuffer, offset);

    result.alphaCut = samplerLoadFloat(materialsBuffer, offset);

    result.normalScale = samplerLoadFloat(materialsBuffer, offset);
    result.occlusionStrength = samplerLoadFloat(materialsBuffer, offset);

    return result;
}

float RayBoundingBoxDst(in Ray r, in AABB box, float t) {
    vec3 invDir = 1.0 / r.direction;
    vec3 tMin = (box.min - r.origin) * invDir;
    vec3 tMax = (box.max - r.origin) * invDir;

    vec3 t1 = min(tMin, tMax);
    vec3 t2 = max(tMin, tMax);

    float near = max(max(t1.x, t1.y), t1.z);
    if (near > t) return 1e20;

    float far = min(min(t2.x, t2.y), t2.z);

    return far >= near && far > 0 ? near : 1e20;
}

// Sphere Functions
Sphere loadSphere(in samplerBuffer buffer, inout int objectIndex) {
    Sphere result;
    result.center = samplerLoadVec3(buffer, objectIndex);
    result.radius = samplerLoadFloat(buffer, objectIndex);
    return result;
}

bool hitSphere(in Sphere sphere, in Ray r, float max, inout HitInfo info) {
    vec3 dir = sphere.center - r.origin;
    float a = dot(r.direction, r.direction);
    float h = dot(r.direction, dir);
    float c = dot(dir, dir) - sphere.radius * sphere.radius;
    float discriminant = h * h - a * c;
    if (discriminant < 0) {
        return false;
    }

    float sqrtd = sqrt(discriminant);

    float t = (h - sqrtd) / a;
    if (t <= 1e-8 || t >= max || t >= info.t) {
        t = (h + sqrtd) / a;
        if (t <= 1e-8 || t >= max || t >= info.t) {
            return false;
        }
    }

    info.t = t;
    info.point = rayAt(r, t);
    info.normal = normalize((info.point - sphere.center) / sphere.radius);
    info.front_face = dot(r.direction, info.normal) < 0;
    return true;
}

// Quad Functions
Quad loadQuad(in samplerBuffer buffer, inout int objectIndex) {
    Quad result;
    result.q = samplerLoadVec3(buffer, objectIndex);
    result.u = samplerLoadVec3(buffer, objectIndex);
    result.v = samplerLoadVec3(buffer, objectIndex);
    result.cullFace = bool(samplerLoadFloat(buffer, objectIndex));
    return result;
}

bool hitQuad(in Quad quad, in Ray r, float max, inout HitInfo info) {
    // Precompute normal and its squared length
    vec3 normal = cross(quad.u, quad.v);
    float denom = dot(normal, r.direction);
    float nn = dot(normal, normal); // avoid redundant computation

    // Backface cull or skip parallel rays
    if (abs(denom) < MIN_DENOMINATOR) return false;

    // Solve plane equation: dot(N, X) = dot(N, P)
    float t = dot(normal, quad.q - r.origin) / denom;
    if (t < 1e-8 || t > max || t >= info.t) return false;

    vec3 hitPos = rayAt(r, t);
    vec3 rel = hitPos - quad.q;

    // Use barycentric-style check in plane coordinates
    float alpha = dot(normal, cross(rel, quad.v)) / nn;
    float beta  = dot(normal, cross(quad.u, rel)) / nn;

    // Bounds check inside the quad (0 ≤ alpha, beta ≤ 1)
    if (alpha < 0.0 || alpha > 1.0 || beta < 0.0 || beta > 1.0) return false;

    // Populate hit info
    info.t = t;
    info.point = hitPos;
    info.normal = denom < 0.0 ? normalize(normal) : -normalize(normal);

    info.front_face = dot(r.direction, info.normal) < 0;
    return true;
}

// Triangle Function
Triangle loadTriangle(in samplerBuffer buffer, inout int objectIndex) {
    Triangle result;

    result.vertices[0] = samplerLoadVec3(buffer, objectIndex);
    result.vertices[1] = samplerLoadVec3(buffer, objectIndex);
    result.vertices[2] = samplerLoadVec3(buffer, objectIndex);

    result.normals[0] = samplerLoadVec3(buffer, objectIndex);
    result.normals[1] = samplerLoadVec3(buffer, objectIndex);
    result.normals[2] = samplerLoadVec3(buffer, objectIndex);

    return result;
}

bool hitTriangle(in Triangle tri, in Ray r, float max, inout HitInfo info) {
    vec3 edgeAB = tri.vertices[1] - tri.vertices[0];
    vec3 edgeAC = tri.vertices[2] - tri.vertices[0];
    vec3 normal = cross(edgeAB, edgeAC);

    // if (dot(normal, r.direction) >= 0) return false;

    float determinant = -dot(r.direction, normal);
    if (abs(determinant) < 1e-8) return false; // parallel

    vec3 ao = r.origin - tri.vertices[0];
    vec3 dao = cross(ao, r.direction);

    float invDet = 1.0 / determinant;

    float t = dot(ao, normal) * invDet;
    if (t < 0 || t > max || t >= info.t) return false;

    float u =  dot(edgeAC, dao) * invDet;
    float v = -dot(edgeAB, dao) * invDet;
    if (u < 0.0 || v < 0.0 || u + v > 1.0) return false;

    info.mat = loadMaterial(tri.materialIndex);
    if (info.mat.texture.baseColorTexture != -1 && info.mat.alphaCut > 0) {
        vec3 e0 = tri.vertices[1] - tri.vertices[0];
        vec3 e1 = tri.vertices[2] - tri.vertices[0];
        vec3 vp = rayAt(r, info.t)  - tri.vertices[0];

        float d00 = dot(e0, e0);
        float d01 = dot(e0, e1);
        float d11 = dot(e1, e1);
        float d20 = dot(vp, e0);
        float d21 = dot(vp, e1);

        float denom = d00 * d11 - d01 * d01;

        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0 - v - w;

        info.uv = u * tri.UVs[0] + v * tri.UVs[1] + w * tri.UVs[2];

        TextureInfo texInfo = loadTextureInfo(info.mat.texture.baseColorTexture);
        vec2 uv = getTextureUV(texInfo, info.uv);
        info.mat.texture.baseColorTexture = getTextureItemIndex(texInfo, uv);
        info.mat.albedo = samplerLoadVec3(texturesBuffer, info.mat.texture.baseColorTexture);
        info.mat.albedo = srgb_to_linear(info.mat.albedo);
        float a = samplerLoadFloat(texturesBuffer, info.mat.texture.baseColorTexture);
        if (a < info.mat.alphaCut) {
            return false;
        }
    }

    info.t = t;
    info.point = rayAt(r, t);

    if (dot(tri.normals[0], tri.normals[0]) > 0) {
        float w = 1.0 - u - v;
        vec3 smoothNormal = normalize(
              tri.normals[0] * w +
              tri.normals[1] * u +
              tri.normals[2] * v
        );
        info.normal = smoothNormal;
    }
    else {
        info.normal = normalize(normal);
    }

    info.front_face = dot(r.direction, info.normal) < 0;

    // === Tangent calculation ===
    vec3 tangent;

    // Try UV-based calculation first
    vec2 deltaUV1 = tri.UVs[1] - tri.UVs[0];
    vec2 deltaUV2 = tri.UVs[2] - tri.UVs[0];
    float uvDet = deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y;

    if (abs(uvDet) > MIN_DENOMINATOR) {
        float f = 1.0 / uvDet;

        tangent = vec3(
            f * (deltaUV2.y * edgeAB.x - deltaUV1.y * edgeAC.x),
            f * (deltaUV2.y * edgeAB.y - deltaUV1.y * edgeAC.y),
            f * (deltaUV2.y * edgeAB.z - deltaUV1.y * edgeAC.z)
        );

        // Gram-Schmidt orthogonalize
        tangent = tangent - dot(tangent, info.normal) * info.normal;

        if (length(tangent) > MIN_DENOMINATOR) {
            info.tangent = normalize(tangent);
            info.bitangent = cross(info.normal, info.tangent);
        }
    }

    return true;
}

// Model Function
Model loadModel(samplerBuffer buffer, inout int objectIndex) {
    Model result;

    result.identifiersCount = samplerLoadFloatInt(buffer, objectIndex);
    result.verticesCount = samplerLoadFloatInt(buffer, objectIndex);
    result.UVsCount = samplerLoadFloatInt(buffer, objectIndex);
    result.nodesCount = samplerLoadFloatInt(buffer, objectIndex);

    return result;
}

BVHNode loadBVHNodeAt(samplerBuffer buffer, int objectIndex) {
    BVHNode result;

    result.boundingBox.min = samplerLoadVec3(buffer, objectIndex);
    result.boundingBox.max = samplerLoadVec3(buffer, objectIndex);

    result.leftIndex = samplerLoadFloatInt(buffer, objectIndex);
    result.rightIndex = samplerLoadFloatInt(buffer, objectIndex);
    result.isLeaf = bool(samplerLoadFloat(buffer, objectIndex));

    return result;
}

void loadVertexByIndex(int offset, int index, inout vec3 position, inout vec3 normal, inout vec2 uv) {
    // NOTE: position (3) + normal (3) + uv (2) = 8 (f32)
    int location = offset + index * 8;
    position = samplerLoadVec3(modelObjectsBuffer, location);
    normal = samplerLoadVec3(modelObjectsBuffer, location);
    uv = samplerLoadVec2(modelObjectsBuffer, location);
}

void loadTriangleByIndex(int offset, ivec3 index, inout Triangle triangle) {
    for (int i = 0; i < 3; ++i) {
        loadVertexByIndex(offset, index[i], triangle.vertices[i], triangle.normals[i], triangle.UVs[i]);
    }
}

Identifier loadIdentifier(int index) {
    Identifier iden;
    iden.index = vec3BitsToIVec3(samplerLoadVec3(modelObjectsBuffer, index));
    iden.materialIndex = samplerLoadFloatInt(modelObjectsBuffer, index);
    return iden;
}

bool hitModel(in Model model, in Ray r, float max, inout HitInfo info, int objectIndex, inout Triangle triangle) {
    int stack[32];
    int stackIndex = 0;
    stack[stackIndex++] = 0;

    HitInfo hInfo;
    hInfo.t = 1e20;

    while (stackIndex > 0) {
        int nodeIndex = stack[--stackIndex];
        BVHNode node = loadBVHNodeAt(modelObjectsBuffer, objectIndex + nodeIndex * 9);

        if (node.isLeaf) {
            for (int offset = node.leftIndex; offset < node.rightIndex; ++offset) {
                // Looking for identifiers
                int index = objectIndex + model.nodesCount * 9 + offset * 4;

                Triangle tri;

                Identifier iden = loadIdentifier(index);
                tri.materialIndex = iden.materialIndex;

                loadTriangleByIndex(
                        objectIndex + model.nodesCount * 9 + model.identifiersCount * 4,
                        iden.index, tri);

                if (hitTriangle(tri, r, max, hInfo)) {
                    max = hInfo.t;
                    hInfo.materialIndex = tri.materialIndex;
                    triangle = tri;
                }
            }
            continue;
        }

        BVHNode leftNode = loadBVHNodeAt(modelObjectsBuffer, objectIndex + node.leftIndex * 9);
        BVHNode rightNode = loadBVHNodeAt(modelObjectsBuffer, objectIndex + node.rightIndex * 9);

        float leftDst = RayBoundingBoxDst(r, leftNode.boundingBox, hInfo.t);
        float rightDst = RayBoundingBoxDst(r, rightNode.boundingBox, hInfo.t);

        if (leftDst < rightDst) {
            if (rightDst < hInfo.t) stack[stackIndex++] = node.rightIndex;
            if (leftDst < hInfo.t) stack[stackIndex++] = node.leftIndex;
        }
        else {
            if (leftDst < hInfo.t) stack[stackIndex++] = node.leftIndex;
            if (rightDst < hInfo.t) stack[stackIndex++] = node.rightIndex;
        }
    }
    info = hInfo;
    return info.t < 1e20;
}

void hitModels(in Ray r, inout HitInfo track) {
    HitInfo tmp = track;

    float closest = tmp.t;
    float startClosest = closest;

    int modelObjectIndex = 0;

    Triangle tri;
    for (int i = 0; i < modelsCount; ++i) {
        bool hitted = false;

        Model model = loadModel(modelObjectsBuffer, modelObjectIndex);
        hitted = hitModel(model, r, closest, tmp, modelObjectIndex, tri);
        modelObjectIndex += model.nodesCount * 9
                        + model.identifiersCount * 4 // For identifiers
                        + model.verticesCount * 6 // For vertices and normals
                        + model.UVsCount * 2; // For UVs

        if (hitted) {
            closest = tmp.t;
            track = tmp;
        }
        track.tests++;
    }

    if (startClosest > closest) {
        track.mat = loadMaterial(track.materialIndex);
        vec3 e0 = tri.vertices[1] - tri.vertices[0];
        vec3 e1 = tri.vertices[2] - tri.vertices[0];
        vec3 vp = rayAt(r, track.t)  - tri.vertices[0];

        float d00 = dot(e0, e0);
        float d01 = dot(e0, e1);
        float d11 = dot(e1, e1);
        float d20 = dot(vp, e0);
        float d21 = dot(vp, e1);

        float denom = d00 * d11 - d01 * d01;

        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0 - v - w;

        track.uv = u * tri.UVs[0] + v * tri.UVs[1] + w * tri.UVs[2];

        if (track.mat.texture.baseColorTexture != -1) {
            TextureInfo texInfo = loadTextureInfo(track.mat.texture.baseColorTexture);

            vec2 uv = getTextureUV(texInfo, track.uv);
            track.mat.texture.baseColorTexture = getTextureItemIndex(texInfo, uv);
            track.mat.albedo = samplerLoadVec3(texturesBuffer, track.mat.texture.baseColorTexture);
            track.mat.albedo = srgb_to_linear(track.mat.albedo);
            float a = samplerLoadFloat(texturesBuffer, track.mat.texture.baseColorTexture);
            track.mat.transmission *= 1.0 - a;
        }

        if (track.mat.texture.metallicRoughnessTexture != -1) {
            TextureInfo texInfo = loadTextureInfo(track.mat.texture.metallicRoughnessTexture);

            vec2 uv = getTextureUV(texInfo, track.uv);
            track.mat.texture.metallicRoughnessTexture = getTextureItemIndex(texInfo, uv);
            vec3 metallicRoughness = samplerLoadVec3(texturesBuffer, track.mat.texture.metallicRoughnessTexture);
            track.mat.roughness *= metallicRoughness.g;
            track.mat.metallic *= metallicRoughness.b;
        }

        if (track.mat.texture.normalTexture != -1) {
            TextureInfo texInfo = loadTextureInfo(track.mat.texture.normalTexture);

            vec2 uv = getTextureUV(texInfo, track.uv);
            track.mat.texture.normalTexture = getTextureItemIndex(texInfo, uv);
            vec3 tangentNormal = samplerLoadVec3(texturesBuffer, track.mat.texture.normalTexture);
            tangentNormal = tangentNormal * 2.0 - 1.0;
            tangentNormal.xy *= track.mat.normalScale;
            tangentNormal = normalize(tangentNormal);

            mat3 TBN = mat3(track.tangent, track.bitangent, track.normal);
            track.normal = normalize(TBN * tangentNormal);
            track.front_face = dot(r.direction, track.normal) < 0;
        }

        if (track.mat.texture.emissiveTexture != -1) {
            TextureInfo texInfo = loadTextureInfo(track.mat.texture.emissiveTexture);

            vec2 uv = getTextureUV(texInfo, track.uv);
            track.mat.texture.emissiveTexture = getTextureItemIndex(texInfo, uv);
            vec3 textureColor = samplerLoadVec3(texturesBuffer, track.mat.texture.emissiveTexture);
            track.mat.emissionColor *= textureColor;
        }

        if (track.mat.texture.transmissionTexture != -1) {
            TextureInfo texInfo = loadTextureInfo(track.mat.texture.transmissionTexture);

            vec2 uv = getTextureUV(texInfo, track.uv);
            track.mat.texture.transmissionTexture = getTextureItemIndex(texInfo, uv);
            vec3 textureColor = samplerLoadVec3(texturesBuffer, track.mat.texture.transmissionTexture);
            track.mat.transmission *= textureColor.r;
        }

        if (track.mat.texture.occlusionTexture != -1) {
            TextureInfo texInfo = loadTextureInfo(track.mat.texture.occlusionTexture);

            vec2 uv = getTextureUV(texInfo, track.uv);
            track.mat.texture.occlusionTexture = getTextureItemIndex(texInfo, uv);
            vec3 textureColor = samplerLoadVec3(texturesBuffer, track.mat.texture.occlusionTexture);
            track.mat.transmission *= 1.0 - (1.0 - textureColor.r) * (1.0 - track.mat.occlusionStrength);
        }
    }
}

void hit(in Ray r, inout HitInfo track) {
    HitInfo tmp = track;

    float closest = tmp.t;
    float startClosest = closest;

    int objectIndex = 0;

    for (int i = 0; i < objectCount; ++i) {
        int type = samplerLoadFloatInt(objectsBuffer, objectIndex);
        tmp.materialIndex = samplerLoadFloatInt(objectsBuffer, objectIndex);

        bool hitted = false;

        switch (type) {
            case 0:
                Sphere sphere = loadSphere(objectsBuffer, objectIndex);
                sphere.materialIndex = tmp.materialIndex;
                hitted = hitSphere(sphere, r, closest, tmp);
                break;
            case 1:
                Quad quad = loadQuad(objectsBuffer, objectIndex);

                if (quad.cullFace && dot(r.direction, cross(quad.u, quad.v)) > 0) {
                    break;
                }

                quad.materialIndex = tmp.materialIndex;
                hitted = hitQuad(quad, r, closest, tmp);
                break;
            case 2:
                Triangle tri = loadTriangle(objectsBuffer, objectIndex);
                tri.materialIndex = tmp.materialIndex;
                hitted = hitTriangle(tri, r, closest, tmp);
                break;
            default:
                break;
        }

        if (hitted) {
            closest = tmp.t;
            track = tmp;
        }
        track.tests++;
    }

    if (startClosest > closest) {
        track.mat = loadMaterial(track.materialIndex);
    }

    hitModels(r, track);
}
    )";
}
