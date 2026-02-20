#include "RayTracer.h"

#include "glUtilities/ShaderProgram.h"
#include "glUtilities/Texture2D.h"

#include "RayCamera.h"
#include "RayScene.h"

#include <glad/glad.h>

static const char *vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec2 aPos;

void main() {
    gl_Position = vec4(aPos, 0, 1.0);
}
)";

static const char *RGBRayTracerFragShaderSource = R"(
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

struct TextureInfo {
    int width, height, channels, wrapS, wrapT, index;
};

struct MaterialTexture {
    int normalTexture;
    int baseColorTexture;
    int metallicRoughnessTexture;
    int emissiveTexture;
    int transmissionTexture;
    int occlusionTexture;
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

    float alphaCut;

    float normalScale;
    float occlusionStrength;

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

struct Identifier {
    ivec3 index;
    int materialIndex;
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

int samplerLoadFloatInt(samplerBuffer buffer, inout int index) {
    float x = samplerLoadFloat(buffer, index);
    return floatBitsToInt(x);
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
    float d = max(1.0 + (a * a - 1.0) * r2, MIN_DENOMINATOR);
    float cosTheta = sqrt((1.0 - r2) / d);
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

vec3 computeF0(in HitInfo info) {
    float specular = clamp(info.mat.specular, 0.0, 1.0);        // user control
    float tintAmount = clamp(info.mat.specularTint, 0.0, 1.0);  // influence of albedo

    vec3 f0 = vec3(0.16 * specular * specular);
    return mix(f0, info.mat.albedo, info.mat.metallic);

    vec3 baseTint = vec3(1.0);
    if (dot(info.mat.albedo, info.mat.albedo) > 0.0) {
        baseTint = normalize(info.mat.albedo);
    }

    vec3 tint = mix(vec3(1.0), baseTint, tintAmount);  // weighted albedo tint
    vec3 dielectricF0 = 0.08 * specular * tint;        // 0.08 ~ empirical fit

    vec3 metalF0 = clamp(info.mat.albedo, vec3(0.0), vec3(1.0));
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
    return kd * info.mat.albedo * INV_PI;
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

    return info.mat.albedo * Fd * INV_PI * info.mat.subsurface;
}

// TextureInfo
TextureInfo loadTextureInfo(int textureInfoIndex) {
    TextureInfo info;
    info.width = int(samplerLoadFloat(texturesBuffer, textureInfoIndex));
    info.height = int(samplerLoadFloat(texturesBuffer, textureInfoIndex));
    info.channels = int(samplerLoadFloat(texturesBuffer, textureInfoIndex));
    info.wrapS = int(samplerLoadFloat(texturesBuffer, textureInfoIndex));
    info.wrapT  = int(samplerLoadFloat(texturesBuffer, textureInfoIndex));
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
    iden.index = ivec3(samplerLoadVec3(modelObjectsBuffer, index));
    iden.materialIndex = int(samplerLoadFloat(modelObjectsBuffer, index));
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

float srgb_to_linear_c(float c) {
    if (c <= 0.04045)
        return c / 12.92;
    else
        return pow((c + 0.055) / 1.055, 2.4);
}

vec3 srgb_to_linear(vec3 srgb) {
    return vec3(srgb_to_linear_c(srgb.x), srgb_to_linear_c(srgb.y), srgb_to_linear_c(srgb.z));
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

float fresnelSchlick(float cosine, float reflectance_index) {
    // Use Schlick's approximation for reflectance.
    float r0 = (1 - reflectance_index) / (1 + reflectance_index);
    r0 = r0 * r0;
    return r0 + (1 - r0) * pow(1 - cosine, 5);
}

vec3 sampleTransmission(in vec3 N, in vec3 V, bool front_face, in Material mat, inout SeedType seed) {
    float eta = front_face ? (1.0 / mat.ior) : mat.ior;

    vec3 H = sampleGGXVNDF_H(N, V, mat.roughness, seed);
    float VoH = dot(V, H);

    // Handle backfacing microfacets
    if (VoH < 0.0) {
        // Backfacing microfacet - use macro normal
        H = N;
        VoH = dot(V, N);
    }

    float cos_theta = min(VoH, 1);
    float sin_theta = sqrt(max(1.0 - cos_theta * cos_theta, 0));

    bool cannot_refract = eta * sin_theta > 1.0;

    if (cannot_refract) {
        return reflect(-V, H);
    }

    // Fresnel effect implicitly handled by choosing
    // no need to apply (1.0 - R) later
    float R = fresnelSchlick(cos_theta, eta);

    if (randFloat(seed) < R) {
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
            if (!info.front_face) {
                vec3 albedo = max(info.mat.albedo, vec3(MIN_DENOMINATOR));
                vec3 transmittance = exp(info.t * log(albedo)); // Beer–Lambert
                rayColor *= transmittance;
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
        if (dot(rayColor, vec3(1)) < 1e-6) break;
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

static const char *SpectralRayTracerFragShaderSource = R"(
#version 330 core
out vec4 fragColor;

#define SeedType uint
#define MIN_DENOMINATOR 1e-8

#define HERO_WAVELENGTH_ENABLE 0

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

struct TextureInfo {
    int width, height, channels, wrapS, wrapT, index;
};

struct MaterialTexture {
    int normalTexture;
    int baseColorTexture;
    int metallicRoughnessTexture;
    int emissiveTexture;
    int transmissionTexture;
    int occlusionTexture;
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

    float alphaCut;

    float normalScale;
    float occlusionStrength;

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

struct Identifier {
    ivec3 index;
    int materialIndex;
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

int samplerLoadFloatInt(samplerBuffer buffer, inout int index) {
    float x = samplerLoadFloat(buffer, index);
    return floatBitsToInt(x);
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
    float d = max(1.0 + (a * a - 1.0) * r2, MIN_DENOMINATOR);
    float cosTheta = sqrt((1.0 - r2) / d);
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

float fresnelSchlickScalar(float cosTheta, float F0) {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
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

// TextureInfo
TextureInfo loadTextureInfo(int textureInfoIndex) {
    TextureInfo info;
    info.width = int(samplerLoadFloat(texturesBuffer, textureInfoIndex));
    info.height = int(samplerLoadFloat(texturesBuffer, textureInfoIndex));
    info.channels = int(samplerLoadFloat(texturesBuffer, textureInfoIndex));
    info.wrapS = int(samplerLoadFloat(texturesBuffer, textureInfoIndex));
    info.wrapT  = int(samplerLoadFloat(texturesBuffer, textureInfoIndex));
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
    iden.index = ivec3(samplerLoadVec3(modelObjectsBuffer, index));
    iden.materialIndex = int(samplerLoadFloat(modelObjectsBuffer, index));
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

float srgb_to_linear_c(float c) {
    if (c <= 0.04045)
        return c / 12.92;
    else
        return pow((c + 0.055) / 1.055, 2.4);
}

vec3 srgb_to_linear(vec3 srgb) {
    return vec3(srgb_to_linear_c(srgb.x), srgb_to_linear_c(srgb.y), srgb_to_linear_c(srgb.z));
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

float fresnelSchlick(float cosine, float reflectance_index) {
    // Use Schlick's approximation for reflectance.
    float r0 = (1 - reflectance_index) / (1 + reflectance_index);
    r0 = r0 * r0;
    return r0 + (1 - r0) * pow(1 - cosine, 5);
}

vec3 sampleTransmission(in vec3 N, in vec3 V, bool front_face, in Material mat, inout SeedType seed) {
    float eta = front_face ? (1.0 / mat.ior) : mat.ior;

    vec3 H = sampleGGXVNDF_H(N, V, mat.roughness, seed);
    float VoH = dot(V, H);

    // Handle backfacing microfacets
    if (VoH < 0.0) {
        // Backfacing microfacet - use macro normal
        H = N;
        VoH = dot(V, N);
    }

    float cos_theta = min(VoH, 1);
    float sin_theta = sqrt(max(1.0 - cos_theta * cos_theta, 0));

    bool cannot_refract = eta * sin_theta > 1.0;

    if (cannot_refract) {
        return reflect(-V, H);
    }

    // Fresnel effect implicitly handled by choosing
    // no need to apply (1.0 - R) later
    float R = fresnelSchlick(cos_theta, eta);

    if (randFloat(seed) < R) {
        return reflect(-V, H);
    }

    return refract(-V, H, eta);
}

// --- CONSTANTS ---
const float WL_MIN = 380.0;
const float WL_MAX = 780.0;
const float WL_RANGE = WL_MAX - WL_MIN;
const float CIE_Y_INTEGRAL = 106.856895;

// CIE 1931 Standard Observer (2*) - sampled every 10nm from 380-780nm
const int CIE_SAMPLES = 41;
const float CIE_WAVELENGTHS[41] = float[41](
    380.0, 390.0, 400.0, 410.0, 420.0, 430.0, 440.0, 450.0, 460.0, 470.0,
    480.0, 490.0, 500.0, 510.0, 520.0, 530.0, 540.0, 550.0, 560.0, 570.0,
    580.0, 590.0, 600.0, 610.0, 620.0, 630.0, 640.0, 650.0, 660.0, 670.0,
    680.0, 690.0, 700.0, 710.0, 720.0, 730.0, 740.0, 750.0, 760.0, 770.0, 780.0
);

// X-bar values
const float CIE_X[41] = float[41](
    0.0014, 0.0042, 0.0143, 0.0435, 0.1344, 0.2839, 0.3483, 0.3362, 0.2908, 0.1954,
    0.0956, 0.0320, 0.0049, 0.0093, 0.0633, 0.1655, 0.2904, 0.4334, 0.5945, 0.7621,
    0.9163, 1.0263, 1.0622, 1.0026, 0.8544, 0.6424, 0.4479, 0.2835, 0.1649, 0.0874,
    0.0468, 0.0227, 0.0114, 0.0058, 0.0029, 0.0014, 0.0007, 0.0003, 0.0002, 0.0001, 0.0000
);

// Y-bar values (luminance function)
const float CIE_Y[41] = float[41](
    0.0000, 0.0001, 0.0004, 0.0012, 0.0040, 0.0116, 0.0230, 0.0380, 0.0600, 0.0910,
    0.1390, 0.2080, 0.3230, 0.5030, 0.7100, 0.8620, 0.9540, 0.9950, 0.9950, 0.9520,
    0.8700, 0.7570, 0.6310, 0.5030, 0.3810, 0.2650, 0.1750, 0.1070, 0.0610, 0.0320,
    0.0170, 0.0082, 0.0041, 0.0021, 0.0010, 0.0005, 0.0003, 0.0001, 0.0001, 0.0000, 0.0000
);

// Z-bar values
const float CIE_Z[41] = float[41](
    0.0065, 0.0201, 0.0679, 0.2074, 0.6456, 1.3856, 1.7471, 1.7721, 1.6692, 1.2876,
    0.8130, 0.4652, 0.2720, 0.1582, 0.0782, 0.0422, 0.0203, 0.0087, 0.0039, 0.0021,
    0.0017, 0.0011, 0.0008, 0.0003, 0.0002, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
    0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000
);

vec3 get_cie_xyz(float lambda) {
    if (lambda < 380.0 || lambda > 780.0) return vec3(0.0);

    // Manual binary search (unrolled for speed)
    int idx = 0;

    // Divide and conquer: 41 samples
    if (lambda >= 580.0) idx += 20;  // Upper half
    if (lambda >= CIE_WAVELENGTHS[idx + 10]) idx += 10;
    if (lambda >= CIE_WAVELENGTHS[idx + 5]) idx += 5;
    if (lambda >= CIE_WAVELENGTHS[idx + 2]) idx += 2;
    if (lambda >= CIE_WAVELENGTHS[idx + 1]) idx += 1;

    // Interpolate
    float t = (lambda - CIE_WAVELENGTHS[idx]) / 10.0;
    return vec3(
        mix(CIE_X[idx], CIE_X[idx + 1], t),
        mix(CIE_Y[idx], CIE_Y[idx + 1], t),
        mix(CIE_Z[idx], CIE_Z[idx + 1], t)
    );
}

// Convert Spectrum back to XYZ
vec3 wavelength_to_xyz(float lambda, float radiance, float pdf) {
    vec3 xyz = get_cie_xyz(lambda);
    xyz *= radiance / pdf;
    return xyz;
}

// --- FAST SMITS METHOD (Optimized) ---

// Pack all 7 spectra into a single array for better cache performance
const float SMITS_TABLE[70] = float[70](
    // White [0-9]
    1.0000, 1.0000, 0.9999, 0.9993, 0.9992, 0.9998, 1.0000, 1.0000, 1.0000, 1.0000,
    // Cyan [10-19]
    0.9710, 0.9426, 1.0007, 1.0007, 1.0007, 1.0007, 0.1564, 0.0000, 0.0000, 0.0000,
    // Magenta [20-29]
    1.0000, 1.0000, 0.9685, 0.2229, 0.0000, 0.0458, 0.8369, 1.0000, 1.0000, 0.9959,
    // Yellow [30-39]
    0.0001, 0.0000, 0.1088, 0.6651, 1.0000, 1.0000, 0.9996, 0.9586, 0.9685, 0.9840,
    // Red [40-49]
    0.1012, 0.0515, 0.0000, 0.0000, 0.0000, 0.0000, 0.8325, 1.0149, 1.0149, 1.0149,
    // Green [50-59]
    0.0000, 0.0000, 0.0273, 0.7937, 1.0000, 0.9418, 0.1719, 0.0000, 0.0000, 0.0025,
    // Blue [60-69]
    1.0000, 1.0000, 0.8916, 0.3323, 0.0000, 0.0000, 0.0003, 0.0369, 0.0483, 0.0496
);

float smits_eval(float lambda, int spectrum_type) {
    // Normalize lambda to [0, 9] range (380-740nm mapped to 10 samples)
    lambda = clamp(lambda, 380.0, 740.0);
    float t = (lambda - 380.0) / 40.0; // 360nm range / 9 intervals = 40nm per interval
    int idx = int(floor(t));
    idx = clamp(idx, 0, 8);
    float frac = t - float(idx);

    // Direct array access (much faster than if-else)
    int offset = spectrum_type * 10;
    float v0 = SMITS_TABLE[offset + idx];
    float v1 = SMITS_TABLE[offset + idx + 1];

    return mix(v0, v1, frac);
}

float get_reflectance(float lambda, vec3 rgb) {
    rgb = clamp(rgb, 0.0, 1.0);

    float result = 0.0;

    // Smits decomposition (optimized with fewer branches)
    float minVal = min(rgb.r, min(rgb.g, rgb.b));
    result += minVal * smits_eval(lambda, 0); // white
                                              //
    vec3 excess = rgb - minVal;

    // Determine decomposition path based on which channel is smallest
    if (rgb.r == minVal) {
        // Red is smallest
        float midVal = min(excess.g, excess.b);
        result += midVal * smits_eval(lambda, 1); // cyan
        if (excess.g < excess.b) {
            result += (excess.b - excess.g) * smits_eval(lambda, 6); // blue
        } else {
            result += (excess.g - excess.b) * smits_eval(lambda, 5); // green
        }
    } else if (rgb.g == minVal) {
        // Green is smallest
        float midVal = min(excess.r, excess.b);
        result += midVal * smits_eval(lambda, 2); // magenta
        if (excess.r < excess.b) {
            result += (excess.b - excess.r) * smits_eval(lambda, 6); // blue
        } else {
            result += (excess.r - excess.b) * smits_eval(lambda, 4); // red
        }
    } else {
        // Blue is smallest
        float midVal = min(excess.r, excess.g);
        result += midVal * smits_eval(lambda, 3); // yellow
        if (excess.r < excess.g) {
            result += (excess.g - excess.r) * smits_eval(lambda, 5); // green
        } else {
            result += (excess.r - excess.g) * smits_eval(lambda, 4); // red
        }
    }

    return clamp(result, 0.0, 1.0);
}

float computeF0Spectral(in HitInfo info, float spectral_albedo, float lambda) {
    float specular = clamp(info.mat.specular, 0.0, 1.0);
    float tintAmount = clamp(info.mat.specularTint, 0.0, 1.0);

    // Base F0 for dielectrics (achromatic)
    float f0_dielectric = 0.16 * specular * specular;

    // Spectral base color
    float f0_colored = spectral_albedo;

    // Apply specular tint for dielectrics
    float f0_dielectric_tinted = mix(f0_dielectric, f0_colored, tintAmount);

    // Blend between dielectric (possibly tinted) and metal
    return mix(f0_dielectric_tinted, f0_colored, info.mat.metallic);
}

// === Diffuse ===
float shadeDiffuseSpectral(in HitInfo info, float spectral_albedo, float lambda, float NoL, float NoV, float VoH) {
    // Energy conservation: reduce diffuse by Fresnel reflection and metallic
    float F0 = computeF0Spectral(info, spectral_albedo, lambda);
    float F = fresnelSchlickScalar(VoH, F0);
    float kd = (1.0 - F) * (1.0 - info.mat.metallic);

    // Disney diffuse with roughness-based retroreflection
    float roughness = info.mat.roughness;
    float FD90 = 0.5 + 2.0 * roughness * VoH * VoH;

    float FL = pow(1.0 - NoL, 5.0);
    float FV = pow(1.0 - NoV, 5.0);

    float fresnelDiffuse = (1.0 + (FD90 - 1.0) * FL) * 
                           (1.0 + (FD90 - 1.0) * FV);

    // Final diffuse BRDF
    return kd * spectral_albedo * fresnelDiffuse * INV_PI;
}

float diffusePdf(float NoL) {
    return NoL * INV_PI;
}

// === Specular ===
float specularPdf(float NoH, float VoH, float roughness) {
    float a = roughness * roughness;
    float D = NDF_GGX(NoH, roughness);
    return D * NoH / max(4.0 * VoH, MIN_DENOMINATOR);
}

float shadeSpecularSpectral(in HitInfo info, float spectral_albedo, float lambda, float NoV, float NoL, float NoH, float VoH) {
    float F0 = computeF0Spectral(info, spectral_albedo, lambda);
    float F = fresnelSchlickScalar(VoH, F0);
    float D = NDF_GGX(NoH, info.mat.roughness);
    float G = geometrySmith(NoV, NoL, info.mat.roughness);
    return (D * G * F) / max(4.0 * NoV * NoL, MIN_DENOMINATOR);
}

float shadeSubsurfaceSpectral(in HitInfo info, float spectral_albedo, float lambda, float NoL, float NoV, float LoV) {
    // Disney subsurface (Hanrahan-Krueger approximation)
    float FL = pow(1.0 - NoL, 5.0);
    float FV = pow(1.0 - NoV, 5.0);

    // Subsurface Fresnel
    float Fss90 = LoV * info.mat.roughness;
    float Fss = mix(1.0, Fss90, FL) * mix(1.0, Fss90, FV);

    // Characteristic subsurface term with normalization
    float ss = 1.25 * (Fss * (1.0 / (NoL + NoV) - 0.5) + 0.5);

    return spectral_albedo * ss * INV_PI;
}

float traceColorWavelength(in Ray r, in float lambda, in SeedType seed) {
    float radiance = 0.0;
    float spectral_throughput = 1.0;

    for (int i = 0; i <= camera.bounces; ++i) {
        HitInfo info;
        info.t = 1e20;
        hit(r, info);

        if (info.t >= 1e20) {
            float t = r.direction.y * 0.5 + 0.5;
            vec3 envColor = (1.0 - t) * vec3(1) + t * skyColor;
            if (dot(skyColor, skyColor) > 0) {
                float reflectance = get_reflectance(lambda, envColor);
                radiance += reflectance * spectral_throughput;
            }
            return radiance;
        }

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
            float dispersion = 0.03;
            float wavelengthRef = 550.0;

            float dispersed_ior = info.mat.ior + dispersion * (pow(wavelengthRef / lambda, 2.0) - 1.0);
            info.mat.ior = dispersed_ior;

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

        r.origin = info.point + L * 0.001;
        r.direction = L;

        float spectral_albedo = get_reflectance(lambda, info.mat.albedo);

        if (trans == 1) {
            if (!info.front_face) {
                float spectral_albedo = max(spectral_albedo, MIN_DENOMINATOR);
                float absorption = -log(spectral_albedo);
                float transmittance = exp(-absorption * info.t); // Beer–Lambert
                spectral_throughput *= transmittance;
            }
            continue;
        }

        if (NoL < MIN_DENOMINATOR) {
            break;
        }

        // Always evaluate both BRDFs and PDFs for MIS
        float brdf_spec_s = shadeSpecularSpectral(info, spectral_albedo, lambda, NoV, NoL, NoH, VoH);
        float brdf_diff_s = shadeDiffuseSpectral(info, spectral_albedo, lambda, NoL, NoV, VoH);
        float brdf_sss_s = shadeSubsurfaceSpectral(info, spectral_albedo, lambda, NoL, NoV, LoV);

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
        float brdf_total_s = ((pdf_spec * pdf_spec) * brdf_spec_s
                        + (pdf_diff * pdf_diff) * brdf_diff_s
                        + (pdf_sss * pdf_sss) * brdf_sss_s) * rdenom;

        // Final contribution
        float contribution = (brdf_total_s * NoL) / max(pdf_used, MIN_DENOMINATOR);

        // Emission (add before rayColor is updated)
        if (info.mat.emissionStrength > 0.0) {
            float energy = get_reflectance(lambda, info.mat.emissionColor);
            radiance += energy * spectral_throughput * info.mat.emissionStrength;
        }

        spectral_throughput *= contribution;

        // Break if spectral throughput is too small
        if (spectral_throughput < 1e-6) {
            break;
        }
    }

    return radiance;
}

const int NUM_HERO_WAVELENGTHS = 4;

vec4 get_hero_wavelengths(float base_offset) {
    float stride = WL_RANGE / float(NUM_HERO_WAVELENGTHS);

    vec4 lambdas;
    lambdas.x = WL_MIN + stride * (0.0 + base_offset);
    lambdas.y = WL_MIN + stride * (1.0 + base_offset);
    lambdas.z = WL_MIN + stride * (2.0 + base_offset);
    lambdas.w = WL_MIN + stride * (3.0 + base_offset);

    // Wrap wavelengths that exceed bounds
    lambdas = mod(lambdas - WL_MIN, WL_RANGE) + WL_MIN;

    return lambdas;
}

vec3 hero_wavelengths_to_rgb(vec4 lambdas, vec4 radiances, float pdf) {
    vec3 rgb = vec3(0.0);
    for (int k = 0; k < 4; k++) {
        rgb += wavelength_to_xyz(lambdas[k], radiances[k], pdf);
    }
    return rgb / float(NUM_HERO_WAVELENGTHS);
}

const vec3 E_WHITE = vec3(0.997065, 1.002169, 0.988182);
const vec3 D65_WHITE = vec3(0.95047, 1.00000, 1.08883);

const mat3 BRADFORD_MA = mat3(
     0.8951, -0.7502,  0.0389,  // Column 0 (was row 0)
     0.2664,  1.7135, -0.0685,  // Column 1 (was row 1)
    -0.1614,  0.0367,  1.0296   // Column 2 (was row 2)
);

const mat3 BRADFORD_MA_INV = mat3(
     0.9870,  0.4323, -0.0085,  // Column 0
    -0.1471,  0.5184,  0.0400,  // Column 1
     0.1600,  0.0493,  0.9685   // Column 2
);

vec3 chromatic_adapt_E_to_D65(vec3 xyz) {
    vec3 rgb_src = BRADFORD_MA * E_WHITE;
    vec3 rgb_dst = BRADFORD_MA * D65_WHITE;
    vec3 gain = rgb_dst / rgb_src;
    vec3 adapted = BRADFORD_MA * xyz;
    adapted *= gain;
    adapted = BRADFORD_MA_INV * adapted;
    return adapted;
}

const mat3 XYZ_TO_RGB = mat3(
    3.2406, -0.9689,  0.0557,
   -1.5372,  1.8758, -0.2040,
   -0.4986,  0.0415,  1.0570
);

vec3 xyz_to_rgb(vec3 xyz) {
    return XYZ_TO_RGB * xyz;
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
    int ssq = int(sqrt(camera.rayPerPixel));
    float wl_dt = WL_RANGE / camera.rayPerPixel;
    float wl_pdf = 1.0 / WL_RANGE;

    float rssq = 1.0 / ssq;
    for (int i = 0; i < ssq; ++i) {
        for (int j = 0; j < ssq; ++j) {
            seed = SeedType(hashSeed(uint(fragCoord.x), uint(fragCoord.y), frameCount, uint(j + i * ssq)));
            Ray r;
            r.origin = cameraCenter;
            r.direction = uv
                + ((j + randFloat(seed)) * rssq) * rImgSize.x * camera.right
                + ((i + randFloat(seed)) * rssq) * rImgSize.y * camera.up;
            r.direction = normalize(r.direction - cameraCenter);

#if HERO_WAVELENGTH_ENABLE
            float base_offset = (randFloat(seed) + i * ssq + j) * wl_dt;
            vec4 lambdas = get_hero_wavelengths(base_offset);
            vec4 radiances;
            for (int k = 0; k < 4; k++) {
                radiances[k] = traceColorWavelength(r, lambdas[k], seed);
            }
            color += hero_wavelengths_to_rgb(lambdas, radiances, wl_pdf);
#else
            float lambda = (randFloat(seed) + i * ssq + j) * wl_dt + WL_MIN;
            float radiance = traceColorWavelength(r, lambda, seed);
            color += wavelength_to_xyz(lambda, radiance, wl_pdf);
#endif
        }
    }

    color *= rssq * rssq;

    color /= CIE_Y_INTEGRAL;

    color = chromatic_adapt_E_to_D65(color);
    color = xyz_to_rgb(color);

    // Accumulate color
    color = (texture(previousFrame, vec2(gl_FragCoord.xy) * rImgSize).rgb * (frameCount - 1.0) + color) / float(frameCount);
    fragColor = vec4(color, 1.0);
}
)";

#include <iostream>

bool RayTracer::initialize(glm::ivec2 resolution, Type type) {
    m_type = type;

    m_shader = std::make_unique<gl::ShaderProgram>();
    m_shader->attachShaderCode(GL_VERTEX_SHADER, vertexShaderSource);

    if (type == Type::RGB) {
        m_shader->attachShaderCode(GL_FRAGMENT_SHADER, RGBRayTracerFragShaderSource);
    }
    else if (type == Type::Spectral) {
        m_shader->attachShaderCode(GL_FRAGMENT_SHADER, SpectralRayTracerFragShaderSource);
    }
    else {
        std::cout << "Unsupported raytracer type: " << static_cast<int>(type) << std::endl;
        ASSERT(false);
    }

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

    scene.bindShader(*m_shader);

    m_shader->setUniform1i("objectCount", scene.getObjectsCount());
    m_shader->setUniform1i("modelsCount", scene.getModelsCount());
    m_shader->setUniform1u("frameCount", m_frameCount);
    m_shader->setUniform3f("skyColor", scene.getLinearSkyColor());

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

