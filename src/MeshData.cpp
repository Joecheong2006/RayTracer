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

    // ================= Load UVs coordinates =================
    meshData.UVs.reserve(meshData.UVs.size() + vertexCount);
    if (primitive.attributes.find("TEXCOORD_0") != primitive.attributes.end()) {
        const tinygltf::Accessor &accessor = model.accessors[primitive.attributes.at("TEXCOORD_0")];
        const tinygltf::BufferView &view = model.bufferViews[accessor.bufferView];
        const tinygltf::Buffer &buffer = model.buffers[view.buffer];

        const u8 *dataPtr = buffer.data.data() + 
                                    view.byteOffset + 
                                    accessor.byteOffset;

        size_t stride = accessor.ByteStride(view);
        
        // Load based on component type
        if (accessor.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT) {
            std::cout << "TINYGLTF_COMPONENT_TYPE_FLOAT\n";
            for (size_t i = 0; i < vertexCount; i++) {
                const float* uv = reinterpret_cast<const float*>(dataPtr + i * stride);
                meshData.UVs.push_back(glm::vec2(uv[0], uv[1]));
            }
        }
        else if (accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE) {
            for (size_t i = 0; i < vertexCount; i++) {
                const u8* uv = dataPtr + i * stride;
                meshData.UVs.push_back(glm::vec2(uv[0] / 255.0f, uv[1] / 255.0f));
            }
        }
        else if (accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) {
            for (size_t i = 0; i < vertexCount; i++) {
                const u16* uv = reinterpret_cast<const uint16_t*>(dataPtr + i * stride);
                meshData.UVs.push_back(glm::vec2(uv[0] / 65535.0f, uv[1] / 65535.0f));
            }
        }
        
        std::cout << "  ✓ Loaded UVs (TEXCOORD_0)" << std::endl;
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
            glm::ivec3 index;
            switch (accessor.componentType) {
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE: {
                    const u8* buf = reinterpret_cast<const uint8_t*>(dataPtr);
                    index = glm::ivec3(buf[i * 3 + 0], buf[i * 3 + 1], buf[i * 3 + 2]);
                    break;
                }
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT: {
                    const u16* buf = reinterpret_cast<const uint16_t*>(dataPtr);
                    index = glm::ivec3(buf[i * 3 + 0], buf[i * 3 + 1], buf[i * 3 + 2]);
                    break;
                }
                case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT: {
                    const u32* buf = reinterpret_cast<const uint32_t*>(dataPtr);
                    index = glm::ivec3(buf[i * 3 + 0], buf[i * 3 + 1], buf[i * 3 + 2]);
                    break;
                }
                default:
                    ASSERT(false); // Unsupported index component type
            }
            index += glm::ivec3(verticesOffset);
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

    auto &pbr = material.pbrMetallicRoughness;
    outMat.texture.baseColorTexture = pbr.baseColorTexture.index;
    outMat.texture.metallicRoughnessTexture = pbr.metallicRoughnessTexture.index;
    outMat.texture.normalTexture = material.normalTexture.index;
    outMat.texture.normalScale = material.normalTexture.scale;
    outMat.texture.emissiveTexture = material.emissiveTexture.index;
    outMat.texture.transmissionTexture = -1;

    std::cout << "ColorTexture: " << outMat.texture.baseColorTexture << std::endl;
    std::cout << "RoughnessTexture: " << outMat.texture.metallicRoughnessTexture << std::endl;
    std::cout << "NormalTexture: " << outMat.texture.normalTexture << std::endl;
    std::cout << "EmissiveTexture: " << outMat.texture.emissiveTexture << std::endl;

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

    outMat.emissionColor = glm::vec3(0);
    it = material.additionalValues.find("emissiveFactor");
    if (it != material.additionalValues.end() && it->second.number_array.size() == 3) {
        outMat.emissionColor = glm::vec3(
            it->second.number_array[0],
            it->second.number_array[1],
            it->second.number_array[2]
        );
    }

    // emissiveStrength  (GLTF extension)
    outMat.emissionStrength = 1;
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

        if (ext.Has("transmissionTexture")) {
            auto& texInfo = ext.Get("transmissionTexture");
            if (texInfo.Has("index")) {
                outMat.texture.transmissionTexture = texInfo.Get("index").GetNumberAsInt();
                std::cout << "TransmissionTexture: " << outMat.texture.transmissionTexture << std::endl;
            }
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

#include <fstream>
#include <vector>
#include <cstdint>
#include <algorithm>

void writeBMP(
    const std::string &filename,
    const std::vector<float>& data, // RGB32F or RGBA32F
    int width,
    int height,
    int channels  // Must be 3 or 4
) {
    if (channels != 3 && channels != 4) {
        std::cerr << "Error: BMP only supports 3 (RGB) or 4 (RGBA) channels!" << std::endl;
        return;
    }

    const int bytesPerPixel = (channels == 3) ? 3 : 4;  // 24-bit or 32-bit BMP
    const int rowStride = width * bytesPerPixel;

    // BMP rows must be aligned to 4 bytes
    const int paddedStride = (rowStride + 3) & ~3;

    const int imageSize = paddedStride * height;
    const int fileSize = 54 + imageSize;

    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    // --- BMP header (14 bytes) + DIB header (40 bytes) = 54 bytes ---
    u8 header[54] = {
        // BMP Header (14 bytes)
        'B', 'M',                                           // Signature
        (u8)fileSize, (u8)(fileSize >> 8),
        (u8)(fileSize >> 16), (u8)(fileSize >> 24),        // File size
        0, 0, 0, 0,                                         // Reserved
        54, 0, 0, 0,                                        // Pixel data offset
        // DIB Header (BITMAPINFOHEADER, 40 bytes)
        40, 0, 0, 0,                                        // DIB header size
        (u8)width, (u8)(width >> 8),
        (u8)(width >> 16), (u8)(width >> 24),              // Width
        (u8)height, (u8)(height >> 8),
        (u8)(height >> 16), (u8)(height >> 24),            // Height
        1, 0,                                               // Color planes (must be 1)
        (u8)(bytesPerPixel * 8), 0,                        // Bits per pixel
        0, 0, 0, 0,                                         // Compression (0 = none)
        (u8)imageSize, (u8)(imageSize >> 8),
        (u8)(imageSize >> 16), (u8)(imageSize >> 24),      // Image size
        0, 0, 0, 0,                                         // X pixels per meter
        0, 0, 0, 0,                                         // Y pixels per meter
        0, 0, 0, 0,                                         // Colors in palette
        0, 0, 0, 0                                          // Important colors
    };

    file.write((char*)header, 54);

    std::vector<uint8_t> row(paddedStride);

    // BMP is stored bottom-up (last row first)
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            int srcIdx = (y * width + x) * channels;  // Index into input data

            // Read RGB (always present)
            float r = data[srcIdx + 0];
            float g = data[srcIdx + 1];
            float b = data[srcIdx + 2];

            // BMP stores as BGR (or BGRA)
            row[x * bytesPerPixel + 0] = (uint8_t)(glm::clamp(b, 0.0f, 1.0f) * 255.0f);
            row[x * bytesPerPixel + 1] = (uint8_t)(glm::clamp(g, 0.0f, 1.0f) * 255.0f);
            row[x * bytesPerPixel + 2] = (uint8_t)(glm::clamp(r, 0.0f, 1.0f) * 255.0f);

            // Alpha (only if channels == 4)
            if (channels == 4) {
                float a = data[srcIdx + 3];
                row[x * bytesPerPixel + 3] = (uint8_t)(glm::clamp(a, 0.0f, 1.0f) * 255.0f);
            }
        }

        // Zero-fill padding bytes (if any)
        std::fill(row.begin() + rowStride, row.end(), 0);

        file.write((char*)row.data(), paddedStride);
    }

    file.close();
}

#include <tinygltf/stb_image_write.h>

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

    meshData.textures.resize(model.textures.size());
    for (size_t i = 0; i < meshData.textures.size(); ++i) {
        auto &image = model.images[model.textures[i].source];

        auto &texture = meshData.textures[i];
        texture.width = image.width;
        texture.height = image.height;
        texture.channelSize = image.bits / 8;
        texture.pixelType = image.pixel_type;

        std::cout << image.name << "(" << i << ")" << ": " << image.width << "x" << image.height << std::endl;
        if (!image.image.empty()) {
            std::cout << "\tUsing pre-loaded image data" << std::endl;

            texture.channels = image.component;
            int pixelCount = texture.width * texture.height;

            if (image.pixel_type == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE) {
                const u8* src = image.image.data();
                
                for (size_t i = 0; i < pixelCount; i++) {
                    size_t srcIdx = i * image.component;
                    
                    switch (image.component) {
                        case 1:
                            texture.data.push_back(src[srcIdx + 0] / 255.0f);
                            break;
                        case 2:
                            texture.data.push_back(src[srcIdx + 0] / 255.0f);
                            texture.data.push_back(src[srcIdx + 1] / 255.0f);
                            break;
                        case 3:
                            texture.data.push_back(src[srcIdx + 0] / 255.0f);
                            texture.data.push_back(src[srcIdx + 1] / 255.0f);
                            texture.data.push_back(src[srcIdx + 2] / 255.0f);
                            break;
                        case 4:
                            texture.data.push_back(src[srcIdx + 0] / 255.0f);
                            texture.data.push_back(src[srcIdx + 1] / 255.0f);
                            texture.data.push_back(src[srcIdx + 2] / 255.0f);
                            texture.data.push_back(src[srcIdx + 3] / 255.0f);
                            break;
                    }
                }

                std::string filename = std::string("output") + std::to_string(i) + std::string(".bmp");
                writeBMP(filename, texture.data, texture.width, texture.height, texture.channels);
                std::cout << "\tDebug output: " << filename << std::endl;
            }
            else if (image.pixel_type == TINYGLTF_COMPONENT_TYPE_FLOAT) {
                const f32* src = reinterpret_cast<const f32*>(image.image.data());
                
                for (size_t i = 0; i < pixelCount; i++) {
                    size_t srcIdx = i * image.component;
                    size_t dstIdx = i * 3;
                    
                    if (image.component == 1) {
                        texture.data[dstIdx] = texture.data[dstIdx + 1] = 
                            texture.data[dstIdx + 2] = src[srcIdx];
                    }
                    else if (image.component >= 3) {
                        texture.data[dstIdx + 0] = src[srcIdx + 0];
                        texture.data[dstIdx + 1] = src[srcIdx + 1];
                        texture.data[dstIdx + 2] = src[srcIdx + 2];
                    }
                }
            }           
        }
        else if (image.bufferView >= 0) {
            std::cout << "\tLoading image data using stb_image" << std::endl;
            auto &view = model.bufferViews[image.bufferView];
            auto &buffer = model.buffers[view.buffer];
            const unsigned char* imageData = buffer.data.data() + view.byteOffset;

            int width, height, channels;
            u8 *decodedData = stbi_load_from_memory(
                    imageData, 
                    view.byteLength, 
                    &width, 
                    &height, 
                    &channels,
                    0
                );

            if (decodedData) {
                texture.channels = channels;
                texture.data.resize(width * height * channels);
                std::copy(decodedData, decodedData + texture.data.size(), texture.data.begin());

                stbi_image_free(decodedData);
            }
        }
        else {
            continue;
        }

        std::cout << "\tChannels: " << texture.channels << std::endl;

        stbi_write_jpg(
            "output.jpg",
            texture.width,
            texture.height,
            texture.channels,
            texture.data.data(),
            90  // quality (90 is good)
        );
    }

    return meshData;
}
