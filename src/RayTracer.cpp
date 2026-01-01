#include "RayTracer.h"

#include "glUtilities/ShaderProgram.h"
#include "glUtilities/Texture2D.h"

#include "RayScene.h"

#include <glad/glad.h>

static const char *vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec2 aPos;

void main() {
    gl_Position = vec4(aPos, 0, 1.0);
}
)";

static const char *fragmentShaderSource = R"(
#version 330 core
out vec4 fragColor;

#define SeedType uint
#define MIN_DENOMINATOR 1e-8

const float PI = 3.1415926;
const float INV_PI = 1.0 / PI;
const float RAD = PI / 180.0;

struct Camera {
    float fov;
    vec2 resolution;
    vec3 position;
    vec3 forward, right, up;
    int bounces, rayPerPixel;
};

struct Ray {
    vec3 origin, direction;
};

struct AABB {
    vec3 min, max;
};

struct MaterialTexture {
    int normalTexture;
    float normalScale;
    int baseColorTexture;
    int metallicRoughnessTexture;
    int emissiveTexture;
};

struct Material {
    vec3 emissionColor;
    float emissionStrength;

    vec3 albedo;
    float subsurface;
    float roughness;
    float metallic;
    float specular;
    float specularTint;

    float transmission;
    float ior;

    MaterialTexture texture;
};

struct HitInfo {
    vec3 point, normal, tangent, bitangent;
    float t;
    vec2 uv;
    int materialIndex;
    Material mat;
    bool front_face;
    int tests;
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

struct Model {
    int identifiersCount, verticesCount, UVsCount, nodesCount;
    int materialIndex;
};

uniform sampler2D previousFrame;
uniform samplerBuffer objectsBuffer;
uniform samplerBuffer modelObjectsBuffer;
uniform samplerBuffer texturesBuffer;
uniform samplerBuffer materialsBuffer;

uniform int objectCount;
uniform int modelsCount;
uniform uint frameCount;
uniform vec3 skyColor;

uniform Camera camera;

// === Buffer Utilities ===
float samplerLoadFloat(samplerBuffer buffer, inout int index) {
    float x = texelFetch(buffer, index).r;
    index++;
    return x;
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

// Utility
uint pcg(uint v) {
    uint state = v * uint(747796405) + uint(2891336453);
    uint word = ((state >> ((state >> uint(28)) + uint(4))) ^ state) * uint(277803737);
    return (word >> uint(22)) ^ word;
}

uint hashSeed(uint pixelX, uint pixelY, uint frameIndex, uint sampleIndex) {
    uint h = pixelX * 73856093u ^ pixelY * 19349663u ^ frameIndex * 83492791u ^ sampleIndex * 2654435761u;
    return pcg(h);
}

float rand(inout SeedType seed) {
    seed = pcg(uint(seed));
    return float(seed) / 4294967296.0;
}

float randFloat(inout SeedType seed) {
    return rand(seed);
}

vec3 reflect(in vec3 v, in vec3 n) {
    return v - dot(v, n) * n * 2.0;
}

vec3 perpendicular(in vec3 v) {
    return (abs(v.x) > 0.9) ? vec3(0,1,0) : vec3(1,0,0);
}

vec3 sampleHemisphereCosine(in vec3 N, inout SeedType seed) {
    float r1 = randFloat(seed);
    float r2 = randFloat(seed);

    float phi = 2.0 * PI * r1;
    float cosTheta = sqrt(1.0 - r2);
    float sinTheta = sqrt(r2);

    vec3 local = vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);

    // Transform from tangent space to world space
    vec3 T = normalize(cross(N, perpendicular(N)));
    vec3 B = normalize(cross(N, T));
    return T * local.x + B * local.y + N * local.z;
}

vec3 sampleGGXVNDF_H(in vec3 N, in vec3 V, float roughness, inout SeedType seed) {
    // Transform view to local space
    float a = roughness * roughness;

    float r1 = randFloat(seed);
    float r2 = randFloat(seed);

    float phi = 2.0 * PI * r1;
    float cosTheta = sqrt((1.0 - r2) / (1.0 + (a * a - 1.0) * r2));
    float sinTheta = sqrt(1.0 - cosTheta * cosTheta);

    vec3 T = normalize(cross(N, perpendicular(N)));
    vec3 B = normalize(cross(N, T));
    mat3 TBN = mat3(T, B, N);
    vec3 Vlocal = transpose(TBN) * V;

    vec3 Hlocal = vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
    vec3 H = TBN * Hlocal;
    return H;
}

vec3 sampleGGXVNDF(in vec3 N, in vec3 V, float roughness, inout SeedType seed) {
    vec3 H = sampleGGXVNDF_H(N, V, roughness, seed);

    vec3 L = reflect(-V, H);
    return dot(N, L) > 0.0 ? L : vec3(0.0); // Ensure valid bounce
}

vec3 getAlbedo(in HitInfo info) {
    return info.mat.albedo;
    if (info.mat.texture.baseColorTexture == -1) {
        return info.mat.albedo;
    }
    int width = int(samplerLoadFloat(texturesBuffer, info.mat.texture.baseColorTexture));
    int height = int(samplerLoadFloat(texturesBuffer, info.mat.texture.baseColorTexture));

    info.uv.x = clamp(info.uv.x, 0.0f, 0.999999f);
    info.uv.y = clamp(info.uv.y, 0.0f, 0.999999f);
    info.mat.texture.baseColorTexture += (int(info.uv.x * width) + int(info.uv.y * height) * width) * 3;
    return samplerLoadVec3(texturesBuffer, info.mat.texture.baseColorTexture);
}

vec3 computeF0(in HitInfo info) {
    float specular = clamp(info.mat.specular, 0.0, 1.0);        // user control
    float tintAmount = clamp(info.mat.specularTint, 0.0, 1.0);  // influence of albedo

    vec3 f0 = vec3(0.16 * specular * specular);
    return mix(f0, getAlbedo(info), info.mat.metallic);

    vec3 baseTint = vec3(1.0);
    if (dot(getAlbedo(info), getAlbedo(info)) > 0.0) {
        baseTint = normalize(getAlbedo(info));
    }

    vec3 tint = mix(vec3(1.0), baseTint, tintAmount);  // weighted albedo tint
    vec3 dielectricF0 = 0.08 * specular * tint;        // 0.08 ~ empirical fit

    vec3 metalF0 = clamp(getAlbedo(info), vec3(0.0), vec3(1.0));
    return mix(dielectricF0, metalF0, info.mat.metallic);
}

vec3 fresnelSchlick(float cosTheta, in vec3 F0) {
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

float NDF_GGX(float NoH, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float demon = NoH * NoH * (a2 - 1.0) + 1.0;
    float demon2 = demon * demon;
    return demon2 < MIN_DENOMINATOR ? 1.0 : a2 / demon2 * INV_PI;
}

float geometrySchlickGGX(float NoV, float roughness) {
    float a = roughness * roughness;
    float k = a * 0.5;
    return NoV / max(NoV * (1.0 - k) + k, MIN_DENOMINATOR);
}

float geometrySmith(float NoV, float NoL, float roughness) {
    float ggx1 = geometrySchlickGGX(NoV, roughness);
    float ggx2 = geometrySchlickGGX(NoL, roughness);
    return ggx1 * ggx2;
}

// === Specular ===
float specularPdf(float NoH, float VoH, float roughness) {
    float a = roughness * roughness;
    float D = NDF_GGX(NoH, roughness);
    return D * NoH / max(4.0 * VoH, MIN_DENOMINATOR);
}

vec3 shadeSpecular(in HitInfo info, float NoV, float NoL, float NoH, float VoH) {
    vec3 F0 = computeF0(info);
    vec3 F = fresnelSchlick(VoH, F0);
    float D = NDF_GGX(NoH, info.mat.roughness);
    float G = geometrySmith(NoV, NoL, info.mat.roughness);
    return (D * G * F) / max(4.0 * NoV * NoL, MIN_DENOMINATOR);
}

// === Diffuse ===
vec3 shadeDiffuse(in HitInfo info, float NoL, float NoV, float VoH) {
    vec3 F0 = computeF0(info);
    vec3 F = fresnelSchlick(VoH, F0);
    vec3 kd = (vec3(1.0) - F) * (1.0 - info.mat.metallic);

    float FD90 = 0.5 + 2.0 * dot(F0, vec3(1.0)); // can tweak this
    float FL = fresnelSchlick(NoL, vec3(1.0)).x;
    float FV = fresnelSchlick(NoV, vec3(1.0)).x;

    float fresnelDiffuse = (1.0 + (FD90 - 1.0) * pow(1.0 - NoL, 5.0)) *
                           (1.0 + (FD90 - 1.0) * pow(1.0 - NoV, 5.0));
    return kd * getAlbedo(info) * INV_PI;
}

float diffusePdf(float NoL) {
    return NoL * INV_PI;
}

// === Subsurface (approximate Burley diffusion model) ===
vec3 shadeSubsurface(in HitInfo info, float NoL, float NoV, float LoV) {
    float FL = pow(1.0 - NoL, 5.0);
    float FV = pow(1.0 - NoV, 5.0);
    float Fd90 = 0.5 + 2.0 * LoV * info.mat.roughness;
    float Fd = mix(1.0, Fd90, FL) * mix(1.0, Fd90, FV);

    return getAlbedo(info) * Fd * INV_PI * info.mat.subsurface;
}

// Material
Material loadMaterial(int materialIndex) {
    Material result;

    // Offset = (byteof Material / byteof f32) -> how many floats from Material
    int offset = materialIndex * (76 / 4);

    result.texture.normalTexture = int(samplerLoadFloat(materialsBuffer, offset));
    result.texture.normalScale = samplerLoadFloat(materialsBuffer, offset);
    result.texture.baseColorTexture = int(samplerLoadFloat(materialsBuffer, offset));
    result.texture.metallicRoughnessTexture = int(samplerLoadFloat(materialsBuffer, offset));
    result.texture.emissiveTexture = int(samplerLoadFloat(materialsBuffer, offset));

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

    return result;
}

// Ray Functions
vec3 rayAt(in Ray r, float t) {
    return r.origin + t * r.direction;
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

    // Compute tangent from texture coordinates (for normal mapping)
    vec2 deltaUV1 = tri.UVs[1] - tri.UVs[0];
    vec2 deltaUV2 = tri.UVs[2] - tri.UVs[0];
    
    float f = 1.0 / (deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y);
    
    vec3 tangent = vec3(
        f * (deltaUV2.y * edgeAB.x - deltaUV1.y * edgeAC.x),
        f * (deltaUV2.y * edgeAB.y - deltaUV1.y * edgeAC.y),
        f * (deltaUV2.y * edgeAB.z - deltaUV1.y * edgeAC.z)
    );
    
    // Gram-Schmidt orthogonalize
    tangent = normalize(tangent - dot(tangent, info.normal) * info.normal);
    vec3 bitangent = cross(info.normal, tangent);
    
    info.tangent = tangent;
    info.bitangent = bitangent;

    info.front_face = dot(r.direction, info.normal) < 0;
    return true;
}

// Model Function
Model loadModel(samplerBuffer buffer, inout int objectIndex) {
    Model result;

    result.identifiersCount = int(samplerLoadFloat(buffer, objectIndex));
    result.verticesCount = int(samplerLoadFloat(buffer, objectIndex));
    result.UVsCount = int(samplerLoadFloat(buffer, objectIndex));
    result.nodesCount = int(samplerLoadFloat(buffer, objectIndex));

    return result;
}

BVHNode loadBVHNodeAt(samplerBuffer buffer, int objectIndex) {
    BVHNode result;

    result.boundingBox.min = samplerLoadVec3(buffer, objectIndex);
    result.boundingBox.max = samplerLoadVec3(buffer, objectIndex);

    result.leftIndex = int(samplerLoadFloat(buffer, objectIndex));
    result.rightIndex = int(samplerLoadFloat(buffer, objectIndex));
    result.isLeaf = bool(samplerLoadFloat(buffer, objectIndex));

    return result;
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

                ivec3 idx = ivec3(samplerLoadVec3(modelObjectsBuffer, index));
                tri.materialIndex = int(samplerLoadFloat(modelObjectsBuffer, index));

                // Looking for positions
                index = objectIndex + model.nodesCount * 9 + model.identifiersCount * 4;
                tri.vertices = loadVec3FromIndices(modelObjectsBuffer, idx, index);

                // Looking for normals
                index = objectIndex + model.nodesCount * 9 + model.identifiersCount * 4 + model.verticesCount * 3;
                tri.normals = loadVec3FromIndices(modelObjectsBuffer, idx, index);

                // Looking for UVs
                index = objectIndex + model.nodesCount * 9 + model.identifiersCount * 4 + model.verticesCount * 6;
                tri.UVs = loadVec2FromIndices(modelObjectsBuffer, idx, index);

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
            int width = int(samplerLoadFloat(texturesBuffer, track.mat.texture.baseColorTexture));
            int height = int(samplerLoadFloat(texturesBuffer, track.mat.texture.baseColorTexture));

            track.uv.x = clamp(track.uv.x, 0.0f, 0.999999f);
            track.uv.y = clamp(track.uv.y, 0.0f, 0.999999f);
            track.mat.texture.baseColorTexture += (int(track.uv.x * width) + int(track.uv.y * height) * width) * 3;
            track.mat.albedo = samplerLoadVec3(texturesBuffer, track.mat.texture.baseColorTexture);
        }

        if (track.mat.texture.metallicRoughnessTexture != -1) {
            int width = int(samplerLoadFloat(texturesBuffer, track.mat.texture.metallicRoughnessTexture));
            int height = int(samplerLoadFloat(texturesBuffer, track.mat.texture.metallicRoughnessTexture));

            track.uv.x = clamp(track.uv.x, 0.0f, 0.999999f);
            track.uv.y = clamp(track.uv.y, 0.0f, 0.999999f);
            track.mat.texture.metallicRoughnessTexture += (int(track.uv.x * width) + int(track.uv.y * height) * width) * 3;
            vec3 metallicRoughness = samplerLoadVec3(texturesBuffer, track.mat.texture.metallicRoughnessTexture);
            track.mat.roughness *= metallicRoughness.g;
            track.mat.metallic *= metallicRoughness.b;
        }

        if (track.mat.texture.normalTexture != -1) {
            int width = int(samplerLoadFloat(texturesBuffer, track.mat.texture.normalTexture));
            int height = int(samplerLoadFloat(texturesBuffer, track.mat.texture.normalTexture));

            track.uv.x = clamp(track.uv.x, 0.0f, 0.999999f);
            track.uv.y = clamp(track.uv.y, 0.0f, 0.999999f);
            track.mat.texture.normalTexture += (int(track.uv.x * width) + int(track.uv.y * height) * width) * 3;
            vec3 tangentNormal = samplerLoadVec3(texturesBuffer, track.mat.texture.normalTexture);
            tangentNormal = normalize(tangentNormal * 2.0 - 1.0);
            
            mat3 TBN = mat3(track.tangent, track.bitangent, track.normal);
            track.normal = normalize(TBN * tangentNormal);
            track.front_face = dot(r.direction, track.normal) < 0;
        }

        if (track.mat.texture.emissiveTexture != -1) {
            int width = int(samplerLoadFloat(texturesBuffer, track.mat.texture.emissiveTexture));
            int height = int(samplerLoadFloat(texturesBuffer, track.mat.texture.emissiveTexture));

            track.uv.x = clamp(track.uv.x, 0.0f, 0.999999f);
            track.uv.y = clamp(track.uv.y, 0.0f, 0.999999f);
            track.mat.texture.emissiveTexture += (int(track.uv.x * width) + int(track.uv.y * height) * width) * 3;
            vec3 textureColor = samplerLoadVec3(texturesBuffer, track.mat.texture.emissiveTexture);
            float strength = samplerLoadFloat(texturesBuffer, track.mat.texture.emissiveTexture);
            track.mat.emissionColor *= textureColor;
        }
    }
}

void hit(in Ray r, inout HitInfo track) {
    HitInfo tmp = track;

    float closest = tmp.t;
    float startClosest = closest;

    int objectIndex = 0;

    for (int i = 0; i < objectCount; ++i) {
        int type = int(samplerLoadFloat(objectsBuffer, objectIndex));
        tmp.materialIndex = int(samplerLoadFloat(objectsBuffer, objectIndex));

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

vec3 refract(in vec3 uv, in vec3 n, float etai_over_etat) {
    float cos_theta = min(dot(-uv, n), 1.0);
    vec3 r_out_perp =  etai_over_etat * (uv + cos_theta * n);
    vec3 r_out_parallel = -sqrt(abs(1.0 - dot(r_out_perp, r_out_perp))) * n;
    return r_out_perp + r_out_parallel;
}

float reflectance(float cosine, float reflectance_index) {
    // Use Schlick's approximation for reflectance.
    float r0 = (1 - reflectance_index) / (1 + reflectance_index);
    r0 = r0 * r0;
    return r0 + (1 - r0) * pow(1 - cosine, 5);
}

vec3 sampleTransmission(in vec3 N, in vec3 V, bool front_face, in Material mat, inout SeedType seed) {
    float eta = front_face ? (1.0 / mat.ior) : mat.ior;

    float cos_theta = min(dot(V, N), 1);
    float sin_theta = sqrt(1.0 - cos_theta * cos_theta);

    vec3 H = sampleGGXVNDF_H(N, V, mat.roughness, seed);
    float R = reflectance(cos_theta, eta);
    bool cannot_refract = eta * sin_theta > 1.0;
    if (cannot_refract || randFloat(seed) < R) {
        return reflect(-V, H);
    }

    return refract(-V, H, eta);
}

vec3 traceColor(in Ray r, inout SeedType seed) {
    vec3 incomingLight = vec3(0.0);
    vec3 rayColor = vec3(1.0);

    int tests = 0;
    for (int i = 0; i <= camera.bounces; ++i) {
        HitInfo info;
        info.t = 1e20;
        hit(r, info);

        if (info.t >= 1e20) {
            float t = r.direction.y * 0.5 + 0.5;
            vec3 envColor = (1.0 - t) * vec3(1) + t * skyColor;
            if (dot(skyColor, skyColor) > 0)
                incomingLight += envColor * rayColor;
            return incomingLight;
        }

        tests += info.tests;

        // Material mat = info.mat;
        // mat.triangleLocation = info.triangleLocation;
        // mat.uvLocation = info.uvLocation;
        vec3 N = normalize(info.normal);
        vec3 V = normalize(-r.direction);

        if (!info.front_face) {
            N = -N;
        }

        float transmissionProb = info.mat.transmission;
        float subsurfaceProb = info.mat.subsurface * (1.0 - transmissionProb);
        float diffuseProb = (1.0 - info.mat.metallic) * (1.0 - transmissionProb);
        float specularProb = (0.5 + 0.5 * info.mat.metallic) * (1.0 - transmissionProb);

        float totalProb = subsurfaceProb + diffuseProb + specularProb + transmissionProb;
        subsurfaceProb /= totalProb;
        diffuseProb /= totalProb;
        specularProb /= totalProb;
        transmissionProb /= totalProb;

        vec3 L;
        float Xi = randFloat(seed);
        float diff = 0, spec = 0, subsurface = 0, trans = 0;
        if (Xi < diffuseProb) {
            L = sampleHemisphereCosine(N, seed);
            diff = 1;
        } else if (Xi < diffuseProb + specularProb) {
            L = sampleGGXVNDF(N, V, info.mat.roughness, seed);
            spec = 1;
        } else if (Xi < diffuseProb + specularProb + transmissionProb) {
            L = sampleTransmission(N, V, info.front_face, info.mat, seed);
            trans = 1;
        } else { // Subsurface — also treated diffuse-like
            L = sampleHemisphereCosine(N, seed);
            subsurface = 1;
        }

        L = normalize(L);

        vec3 H = normalize(V + L);
        float NoV = clamp(dot(N, V), 0.0, 1.0);
        float NoL = clamp(dot(N, L), 0.0, 1.0);
        float NoH = clamp(dot(N, H), 0.0, 1.0);
        float VoH = clamp(dot(V, H), 0.0, 1.0);
        float LoV = clamp(dot(L, V), 0.0, 1.0);

        // Continue path
        r.origin = info.point + L * 0.001;
        r.direction = L;

        if (trans == 1) {
            float eta = info.front_face ? (1.0 / info.mat.ior) : info.mat.ior;
            float cos_theta = min(dot(V, N), 1);
            float sin_theta = sqrt(1.0 - cos_theta * cos_theta);
            if (!info.front_face) {
                vec3 albedo = pow(getAlbedo(info), vec3(2.2));
                vec3 transmittance = exp(info.t * log(albedo)); // Beer–Lambert
                float R = reflectance(cos_theta, eta);
                rayColor *= (1.0 - R) * transmittance;
            }
            continue;
        }

        if (NoL < MIN_DENOMINATOR) {
            break;
        }

        // Always evaluate both BRDFs and PDFs for MIS
        vec3 brdf_sss = shadeSubsurface(info, NoL, NoV, LoV);
        vec3 brdf_spec = shadeSpecular(info, NoV, NoL, NoH, VoH);
        vec3 brdf_diff = shadeDiffuse(info, NoL, NoV, VoH);

        float p_surf = 1.0 - transmissionProb;

        // avoid values in (0, 1e-8) that arise from FP error
        p_surf = (p_surf < 1e-8) ? 0.0 : p_surf;
        float surfaceNormalization = (p_surf > 0.0) ? 1.0 / p_surf : 1.0;

        float pdf_sss = NoL * INV_PI * subsurfaceProb * subsurface * surfaceNormalization;
        float pdf_spec = specularPdf(NoH, VoH, info.mat.roughness) * specularProb * spec * surfaceNormalization;
        float pdf_diff = diffusePdf(NoL) * diffuseProb * diff * surfaceNormalization;

        float pdf_used = pdf_sss + pdf_spec + pdf_diff;

        float denom = pdf_diff * pdf_diff + pdf_spec * pdf_spec + pdf_sss * pdf_sss;
        float rdenom = 1.0 / max(denom, MIN_DENOMINATOR);

        // Combine weighted BRDFs (all lobes)
        vec3 brdf_total = ((pdf_spec * pdf_spec) * brdf_spec
                        + (pdf_diff * pdf_diff) * brdf_diff
                        + (pdf_sss * pdf_sss) * brdf_sss) * rdenom;

        // Final contribution
        vec3 contribution = (brdf_total * NoL) / max(pdf_used, MIN_DENOMINATOR);

        // Emission (add before rayColor is updated)
        if (info.mat.emissionStrength > 0.0)
            incomingLight += rayColor * info.mat.emissionColor * info.mat.emissionStrength;

        rayColor *= contribution;
        if (dot(rayColor, vec3(1)) < 1e-4) break;
    }

    return incomingLight;
}

void main() {
    ivec2 fragCoord = ivec2(gl_FragCoord.xy);
    vec2 imgSize = camera.resolution;
    vec2 rImgSize = 1.0 / vec2(imgSize);

    vec3 lookat = camera.forward + camera.position;
    vec3 cameraCenter = camera.position;

    float viewportRatio = imgSize.x * rImgSize.y;
    float focalLength = length(lookat - cameraCenter);
    float fov = camera.fov;

    float viewportHeight = 2.0 * tan(RAD * fov * 0.5) * focalLength;
    float viewportWidth = viewportHeight * viewportRatio;
    vec2 viewport = vec2(viewportWidth, viewportHeight);

    vec3 uv = vec3(fragCoord * rImgSize * 2.0 - 1.0, 0);
    uv = viewportWidth * 0.5 * uv.x * camera.right
       + viewportHeight * 0.5 * uv.y * camera.up
       + focalLength * camera.forward
       + cameraCenter;

    SeedType seed;

    vec3 color = vec3(0.0);

#if 1
    int ssq = int(sqrt(camera.rayPerPixel));
    float rssq = 1.0 / ssq;
    for (int i = 0; i < ssq; ++i) {
        for (int j = 0; j < ssq; ++j) {
            seed = SeedType(hashSeed(uint(fragCoord.x), uint(fragCoord.y), frameCount, uint(j + i * ssq)));
            Ray r;
            r.origin = cameraCenter;
            r.direction = uv + ((j + randFloat(seed)) * rssq) * rImgSize.x * camera.right + ((i + randFloat(seed)) * rssq) * rImgSize.y * camera.up;
            r.direction = normalize(r.direction - cameraCenter);
            color += traceColor(r, seed);
        }
    }

    color *= rssq * rssq;
#else
    for (int i = 0; i < camera.rayPerPixel; ++i) {
        seed = SeedType(hashSeed(uint(fragCoord.x), uint(fragCoord.y), frameCount, uint(i)));
        Ray r;
        r.origin = cameraCenter;
        r.direction = uv + randFloat(seed) * rImgSize.x * camera.right + randFloat(seed) * rImgSize.y * camera.up;
        r.direction = normalize(r.direction - cameraCenter);
        color += traceColor(r, seed);
    }
    color /= camera.rayPerPixel;
#endif

    color = (texture(previousFrame, vec2(gl_FragCoord.xy) * rImgSize).rgb * (frameCount - 1.0) + color) / float(frameCount);

    fragColor = vec4(color, 1.0);
}
)";

#include <iostream>

bool RayTracer::initialize(glm::ivec2 resolution) {
    m_shader = std::make_unique<gl::ShaderProgram>();
    m_shader->attachShaderCode(GL_VERTEX_SHADER, vertexShaderSource);
    m_shader->attachShaderCode(GL_FRAGMENT_SHADER, fragmentShaderSource);
    m_shader->link();

    auto errors = m_shader->getShaderError();
    if (!errors.empty()) {
        for (auto str : errors) {
            std::cout << str << std::endl;
        }
        return false;
    }

    gl::Texture2D::Construct con;
    con.width = resolution.x;
    con.height = resolution.y;
    con.style = GL_LINEAR;
    con.style = GL_NEAREST;
    con.format = GL_RGBA;
    con.internal = GL_RGBA32F;

    m_frames[0] = std::make_unique<gl::Texture2D>(con);
    m_frames[1] = std::make_unique<gl::Texture2D>(con);
    return true;
}

void RayTracer::renderToTexture(const RayCamera &camera, const RayScene &scene) {
    m_shader->bind();

    getPreviousFrame().bind(0);
    m_shader->setUniform1i("previousFrame", 0);

    scene.bindObjects(1);
    m_shader->setUniform1i("objectsBuffer", 1);

    scene.bindMaterials(2);
    m_shader->setUniform1i("materialsBuffer", 2);

    scene.bindModelObjects(3);
    m_shader->setUniform1i("modelObjectsBuffer", 3);

    scene.bindTextures(4);
    m_shader->setUniform1i("texturesBuffer", 4);

    m_shader->setUniform1i("objectCount", scene.getObjectsCount());
    m_shader->setUniform1i("modelsCount", scene.getModelsCount());
    m_shader->setUniform1u("frameCount", m_frameCount);
    m_shader->setUniform3f("skyColor", scene.getSkyColor());

    m_shader->setUniform1f("camera.fov", camera.fov);
    m_shader->setUniform2f("camera.resolution", camera.resolution);
    m_shader->setUniform3f("camera.position", camera.position);
    m_shader->setUniform3f("camera.forward", camera.forward);
    m_shader->setUniform3f("camera.right", camera.right);
    m_shader->setUniform3f("camera.up", camera.up);
    m_shader->setUniform1i("camera.bounces", camera.bounces);
    m_shader->setUniform1i("camera.rayPerPixel", camera.rayPerPixel);

    m_frameIndex = !m_frameIndex;
    ++m_frameCount;
}

void RayTracer::changeResolution(glm::ivec2 resolution) {
    gl::Texture2D::Construct con;
    con.width = resolution.x;
    con.height = resolution.y;
    con.style = GL_LINEAR;
    con.style = GL_NEAREST;
    con.format = GL_RGBA;
    con.internal = GL_RGBA32F;

    m_frames[0] = std::make_unique<gl::Texture2D>(con);
    m_frames[1] = std::make_unique<gl::Texture2D>(con);
}

void RayTracer::reset() {
    m_frameCount = 1;
    m_frameIndex = 0;
}

