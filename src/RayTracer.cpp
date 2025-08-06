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

struct HitInfo {
    vec3 point, normal;
    float t;
    int materialIndex;
};

struct Ray {
    vec3 origin, direction;
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
};

struct Sphere {
    float radius;
    vec3 center;
    int materialIndex;
};

struct Quad {
    vec3 q, u, v;
    int materialIndex;
};

uniform sampler2D previousFrame;
uniform samplerBuffer objectsBuffer;
uniform samplerBuffer materialsBuffer;

uniform int objectCount;
uniform uint frameCount;
uniform vec3 skyColor;

uniform Camera camera;

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

vec3 sampleGGXVNDF(in vec3 N, in vec3 V, float roughness, inout SeedType seed) {
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

    vec3 L = reflect(-V, H);
    return dot(N, L) > 0.0 ? L : vec3(0.0); // Ensure valid bounce
}

vec3 computeF0(in Material mat) {
    float specular = clamp(mat.specular, 0.0, 1.0);        // user control
    float tintAmount = clamp(mat.specularTint, 0.0, 1.0);  // influence of albedo

    vec3 f0 = vec3(0.16 * specular * specular);
    return mix(f0, mat.albedo, mat.metallic);

    vec3 baseTint = vec3(1.0);
    if (dot(mat.albedo, mat.albedo) > 0.0) {
        baseTint = normalize(mat.albedo);
    }

    vec3 tint = mix(vec3(1.0), baseTint, tintAmount);  // weighted albedo tint
    vec3 dielectricF0 = 0.08 * specular * tint;        // 0.08 ~ empirical fit

    vec3 metalF0 = clamp(mat.albedo, vec3(0.0), vec3(1.0));
    return mix(dielectricF0, metalF0, mat.metallic);
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

vec3 shadeSpecular(in Material mat, float NoV, float NoL, float NoH, float VoH) {
    vec3 F0 = computeF0(mat);
    vec3 F = fresnelSchlick(VoH, F0);
    float D = NDF_GGX(NoH, mat.roughness);
    float G = geometrySmith(NoV, NoL, mat.roughness);
    return (D * G * F) / max(4.0 * NoV * NoL, MIN_DENOMINATOR);
}

// === Diffuse ===
vec3 shadeDiffuse(in Material mat, float NoL, float NoV, float VoH) {
    vec3 F0 = computeF0(mat);
    vec3 F = fresnelSchlick(VoH, F0);
    vec3 kd = (vec3(1.0) - F) * (1.0 - mat.metallic);

    float FD90 = 0.5 + 2.0 * dot(F0, vec3(1.0)); // can tweak this
    float FL = fresnelSchlick(NoL, vec3(1.0)).x;
    float FV = fresnelSchlick(NoV, vec3(1.0)).x;

    float fresnelDiffuse = (1.0 + (FD90 - 1.0) * pow(1.0 - NoL, 5.0)) *
                           (1.0 + (FD90 - 1.0) * pow(1.0 - NoV, 5.0));
    return kd * mat.albedo * INV_PI;
}

float diffusePdf(float NoL) {
    return NoL * INV_PI;
}

// === Subsurface (approximate Burley diffusion model) ===
vec3 shadeSubsurface(in Material mat, float NoL, float NoV, float LoV) {
    float FL = pow(1.0 - NoL, 5.0);
    float FV = pow(1.0 - NoV, 5.0);
    float Fd90 = 0.5 + 2.0 * LoV * mat.roughness;
    float Fd = mix(1.0, Fd90, FL) * mix(1.0, Fd90, FV);

    return mat.albedo * Fd * INV_PI * mat.subsurface;
}

// Material
Material loadMaterial(int materialIndex) {
    Material result;

    int offset = materialIndex * 3;

    vec4 buf = texelFetch(materialsBuffer, offset + 0);
    result.emissionColor = buf.rgb;
    result.emissionStrength = buf.a;

    buf = texelFetch(materialsBuffer, offset + 1);
    result.albedo = buf.rgb;
    result.subsurface = buf.a;

    buf = texelFetch(materialsBuffer, offset + 2);
    result.roughness = buf.r;
    result.metallic = buf.g;
    result.specular = buf.b;
    result.specularTint = buf.a;

    return result;
}

// Ray Functions
vec3 rayAt(in Ray r, float t) {
    return r.origin + t * r.direction;
}

// Sphere Functions
Sphere loadSphere(inout int objectIndex) {
    vec4 buf = texelFetch(objectsBuffer, objectIndex++);
    Sphere result;
    result.center = buf.xyz;
    result.radius = buf.w;
    return result;
}

bool hitSphere(in Sphere sphere, in Ray r, float max, inout HitInfo info) {
    vec3 dir = sphere.center - r.origin;
    float a = dot(r.direction, r.direction);
    float b = -2.0 * dot(r.direction, dir);
    float c = dot(dir, dir) - sphere.radius * sphere.radius;
    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
        return false;
    }

    float sqrtd = sqrt(discriminant);
    info.t = (-b - sqrtd) / (2.0 * a);

    if (!(info.t > 1e-3 && info.t < max)) {
        info.t = (-b + sqrtd) / (2.0 * a);
        if (!(info.t > 0 && info.t < max)) {
            return false;
        }
    }

    info.point = rayAt(r, info.t);
    info.normal = (info.point - sphere.center) / sphere.radius;
    if (dot(r.direction, info.normal) > 0) {
        info.normal = -info.normal;
    }
    return true;
}

// Quad Functions
Quad loadQuad(inout int objectIndex) {
    Quad result;
    result.q = texelFetch(objectsBuffer, objectIndex++).xyz;
    result.u = texelFetch(objectsBuffer, objectIndex++).xyz;
    result.v = texelFetch(objectsBuffer, objectIndex++).xyz;
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
    if (t <= 1e-8 || t >= max) return false;

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
    info.normal = denom < 0.0 ? normalize(normal) : -normalize(normal); // Ensure it's facing opposite the ray

    return true;
}

void hit(in Ray r, inout HitInfo track) {
    HitInfo tmp;

    float closest = 0xffffff;

    int objectIndex = 0;

    for (int i = 0; i < objectCount; ++i) {
        vec4 buf = texelFetch(objectsBuffer, objectIndex++);
        int type = int(buf.r);
        tmp.materialIndex = int(buf.g);

        bool hitted = false;

        switch (type) {
            case 0:
                Sphere sphere = loadSphere(objectIndex);
                sphere.materialIndex = tmp.materialIndex;
                hitted = hitSphere(sphere, r, closest, tmp);
                break;
            case 1:
                Quad quad = loadQuad(objectIndex);
                quad.materialIndex = tmp.materialIndex;
                hitted = hitQuad(quad, r, closest, tmp);
                break;
            default:
                break;
        }

        if (hitted) {
            closest = tmp.t;
            track = tmp;
        }
    }

    track.t = closest;
}

vec3 traceColor(in Ray r, inout SeedType seed) {
    vec3 incomingLight = vec3(0.0);
    vec3 rayColor = vec3(1.0);

    for (int i = 0; i < camera.bounces; ++i) {
        HitInfo info;
        hit(r, info);

        if (info.t >= 0xffffff) {
            float t = r.direction.y * 0.5 + 0.5;
            vec3 envColor = (1.0 - t) * vec3(1) + t * skyColor;
            incomingLight += envColor * rayColor;
            return incomingLight;
        }

        Material mat = loadMaterial(info.materialIndex);
        vec3 N = normalize(info.normal);
        vec3 V = normalize(-r.direction);

        float subsurfaceProb = mat.subsurface;
        float diffuseProb = 1.0 - mat.metallic;
        float specularProb = 0.5 + 0.5 * mat.metallic;

        float totalProb = subsurfaceProb + diffuseProb + specularProb;
        subsurfaceProb /= totalProb;
        diffuseProb /= totalProb;
        specularProb /= totalProb;

        vec3 L;
        float Xi = randFloat(seed);
        float diff = 0, spec = 0, subsurface = 0;
        if (Xi <= diffuseProb) {
            L = sampleHemisphereCosine(N, seed);
            diff = 1;
        } else if (Xi <= diffuseProb + specularProb) {
            L = sampleGGXVNDF(N, V, mat.roughness, seed);
            spec = 1;
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

        if (NoL < MIN_DENOMINATOR) {
            break;
        }

        // Always evaluate both BRDFs and PDFs for MIS
        vec3 brdf_sss = shadeSubsurface(mat, NoL, NoV, LoV);
        vec3 brdf_spec = shadeSpecular(mat, NoV, NoL, NoH, VoH);
        vec3 brdf_diff = shadeDiffuse(mat, NoL, NoV, VoH);

        float pdf_sss = NoL * INV_PI * subsurfaceProb * subsurface;
        float pdf_spec = specularPdf(NoH, VoH, mat.roughness) * specularProb * spec;
        float pdf_diff = diffusePdf(NoL) * diffuseProb * diff;

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
        if (mat.emissionStrength > 0.0)
            incomingLight += rayColor * mat.emissionColor * mat.emissionStrength;

        // Continue path
        rayColor *= contribution;
        r.origin = info.point + L * 0.001;
        r.direction = L;
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

    color = (texture(previousFrame, vec2(gl_FragCoord.xy) * rImgSize).rgb * (frameCount - 1.0) + color) / float(frameCount);

    fragColor = vec4(color, 1.0);
}
)";

void RayTracer::initialize(Camera &camera) {
    m_shader = std::make_unique<gl::ShaderProgram>();
    m_shader->attachShaderCode(GL_VERTEX_SHADER, vertexShaderSource);
    m_shader->attachShaderCode(GL_FRAGMENT_SHADER, fragmentShaderSource);
    m_shader->link();

    gl::Texture2D::Construct con;
    con.width = camera.resolution.x;
    con.height = camera.resolution.y;
    con.style = GL_LINEAR;
    con.style = GL_NEAREST;
    con.format = GL_RGBA;
    con.internal = GL_RGBA32F;

    m_frames[0] = std::make_unique<gl::Texture2D>(con);
    m_frames[1] = std::make_unique<gl::Texture2D>(con);
}

void RayTracer::renderToTexture(RayScene &scene) {
    auto &camera = scene.getCamera();

    m_shader->bind();

    getPreviousFrame().bind(0);
    m_shader->setUniform1i("previousFrame", 0);

    scene.bindObjects(1);
    m_shader->setUniform1i("objectsBuffer", 1);

    scene.bindMaterials(2);
    m_shader->setUniform1i("materialsBuffer", 2);

    m_shader->setUniform1i("objectCount", scene.getObjectsCount());
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
