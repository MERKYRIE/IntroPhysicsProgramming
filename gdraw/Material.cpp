//
// Created by Gaëtan Blaise-Cazalet on 15/01/2025.
//

#include "Material.hpp"
#include <SDL3/SDL.h>
#include "GpuUploader.hpp"
#include "VertexTypes.hpp"
#include "Renderer.hpp"

namespace gdraw {
    Material::Material(Renderer* renderer_) : renderer { renderer_ } { }

    SDL_GPUShader* Material::LoadShader(const str& shaderFilename, u32 samplerCount,
                                        u32 uniformBufferCount, u32 storageBufferCount, u32 storageTextureCount) {
        const char* basePath = SDL_GetBasePath();

        // Auto-detect the shader stage from the file name for convenience
        SDL_GPUShaderStage stage { SDL_GPU_SHADERSTAGE_VERTEX };
        if (SDL_strstr(shaderFilename.c_str(), ".vert")) { stage = SDL_GPU_SHADERSTAGE_VERTEX; }
        else if (
                SDL_strstr(shaderFilename.c_str(), ".frag")) { stage = SDL_GPU_SHADERSTAGE_FRAGMENT; }
        else {
            SDL_Log("Invalid shader stage!");
        }

        char fullPath[256];
        SDL_GPUShaderFormat backendFormats = SDL_GetGPUShaderFormats(renderer->device);
        SDL_GPUShaderFormat format = SDL_GPU_SHADERFORMAT_INVALID;
        const char* entrypoint { "main" };

        if (backendFormats & SDL_GPU_SHADERFORMAT_SPIRV) {
            SDL_snprintf(fullPath, sizeof(fullPath), "%sContent/Shaders/Compiled/SPIRV/%s.spv", basePath,
                         shaderFilename.c_str());
            format = SDL_GPU_SHADERFORMAT_SPIRV;
            entrypoint = "main";
        } else if (backendFormats & SDL_GPU_SHADERFORMAT_MSL) {
            SDL_snprintf(fullPath, sizeof(fullPath), "%sContent/Shaders/Compiled/MSL/%s.msl", basePath,
                         shaderFilename.c_str());
            format = SDL_GPU_SHADERFORMAT_MSL;
            entrypoint = "main0";
        } else if (backendFormats & SDL_GPU_SHADERFORMAT_DXIL) {
            SDL_snprintf(fullPath, sizeof(fullPath), "%sContent/Shaders/Compiled/DXIL/%s.dxil", basePath,
                         shaderFilename.c_str());
            format = SDL_GPU_SHADERFORMAT_DXIL;
            entrypoint = "main";
        } else { SDL_Log("%s", "Unrecognized backend shader format!"); }

        size_t codeSize;
        void* code = SDL_LoadFile(fullPath, &codeSize);
        if (code == nullptr) { SDL_Log("Failed to load shader from disk! %s", fullPath); }

        SDL_GPUShaderCreateInfo shaderInfo = {
                .code_size = codeSize,
                .code = static_cast<Uint8*>(code),
                .entrypoint = entrypoint,
                .format = format,
                .stage = stage,
                .num_samplers = samplerCount,
                .num_storage_textures = storageTextureCount,
                .num_storage_buffers = storageBufferCount,
                .num_uniform_buffers = uniformBufferCount
        };
        SDL_GPUShader* loadedShader = SDL_CreateGPUShader(renderer->device, &shaderInfo);
        if (loadedShader == nullptr) {
            SDL_Log("Failed to create shader!");
            SDL_free(code);
        }

        SDL_free(code);
        return loadedShader;
    }

    void Material::Clear() {
        renderer->ReleaseSampler(sampler);
        renderer->ReleaseTexture(texture);
        renderer->ReleaseGraphicsPipeline(pipeline);
    }

    void Material::LoadTexture(const str& imageFilename, int desiredChannels) {
        // Load surface
        const char* basePath = SDL_GetBasePath();
        char fullPath[256];
        SDL_PixelFormat format;

        SDL_snprintf(fullPath, sizeof(fullPath), "%sContent/Images/%s", basePath, imageFilename.c_str());
        SDL_Surface* surface = SDL_LoadBMP(fullPath);
        if (surface == nullptr) { SDL_Log("Failed to load BMP: %s", SDL_GetError()); }

        if (desiredChannels == 4) { format = SDL_PIXELFORMAT_ABGR8888; }
        else {
            SDL_assert(!"Unexpected desiredChannels");
            SDL_DestroySurface(surface);
        }
        if (surface->format != format) {
            SDL_Surface* next = SDL_ConvertSurface(surface, format);
            SDL_DestroySurface(surface);
            surface = next;
        }
        if (surface == nullptr) { SDL_Log("Could not load image data!"); }

        // Create texture
        SDL_GPUTextureCreateInfo textureInfo
                {
                        .type = SDL_GPU_TEXTURETYPE_2D,
                        .format = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM,
                        .usage = SDL_GPU_TEXTUREUSAGE_SAMPLER,
                        .width = static_cast<Uint32>(surface->w),
                        .height = static_cast<Uint32>(surface->h),
                        .layer_count_or_depth = 1,
                        .num_levels = 1,
                };
        texture = SDL_CreateGPUTexture(renderer->device, &textureInfo);

        // Upload to GPU
        GPUUploader uploader { renderer->device };
        uploader.PrepareTextureData<PositionTextureVertex>(surface);
        uploader.Begin();
        SDL_GPUTextureTransferInfo textureBufferLocation {
                .transfer_buffer = uploader.GetTransferBuffer(),
                .offset = 0
        };
        SDL_GPUTextureRegion textureBufferRegion {
                .texture = texture,
                .w = static_cast<Uint32>(surface->w),
                .h = static_cast<Uint32>(surface->h),
                .d = 1
        };
        uploader.UploadToTexture(textureBufferLocation, textureBufferRegion, false);
        uploader.End();

        // Destroy surface data
        SDL_DestroySurface(surface);
    }

    void Material::CreateSampler(TextureFilter filter, TextureWrap wrap) {
        SDL_GPUSamplerCreateInfo createInfo {
                .min_filter = filter == TextureFilter::Nearest ? SDL_GPU_FILTER_NEAREST : SDL_GPU_FILTER_LINEAR,
                .mag_filter = filter == TextureFilter::Nearest ? SDL_GPU_FILTER_NEAREST : SDL_GPU_FILTER_LINEAR,
                .mipmap_mode = filter == TextureFilter::Nearest
                               ? SDL_GPU_SAMPLERMIPMAPMODE_NEAREST
                               : SDL_GPU_SAMPLERMIPMAPMODE_LINEAR
        };
        if (filter == TextureFilter::Anisotropic) {
            createInfo.max_anisotropy = 4;
            createInfo.enable_anisotropy = true;
        }
        if (wrap == TextureWrap::ClampToEdge) {
            createInfo.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
            createInfo.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
            createInfo.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
        } else if (wrap == TextureWrap::MirroredRepeat) {
            createInfo.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_MIRRORED_REPEAT;
            createInfo.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_MIRRORED_REPEAT;
            createInfo.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_MIRRORED_REPEAT;
        } else {
            createInfo.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
            createInfo.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
            createInfo.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_REPEAT;
        }

        sampler = SDL_CreateGPUSampler(renderer->device, &createInfo);
    }

    void Material::LoadVertexShader(const str& vertexShaderFilename, u32 samplerCount, u32 uniformBufferCount,
                                    u32 storageBufferCount, u32 storageTextureCount) {
        vertexShader = LoadShader(vertexShaderFilename, samplerCount, uniformBufferCount, storageBufferCount,
                                  storageTextureCount);
    }

    void Material::LoadFragmentShader(const str& fragmentShaderFilename, u32 samplerCount, u32 uniformBufferCount,
                                      u32 storageBufferCount, u32 storageTextureCount) {
        fragmentShader = LoadShader(fragmentShaderFilename, samplerCount, uniformBufferCount, storageBufferCount,
                                    storageTextureCount);
    }

    void Material::Bind() {
        renderer->BindGraphicsPipeline(pipeline);
        SDL_GPUTextureSamplerBinding textureSamplerBinding { .texture = texture, .sampler = sampler };
        renderer->BindFragmentSamplers(0, textureSamplerBinding, 1);
        renderer->PushFragmentUniformData(0, &colorMultiply, sizeof(FragMultiplyUniform));
    }

    void Material::Unload() {
        Clear();
    }

    void Material::SetFragmentColorMultiply(float r, float g, float b, float a) {
        colorMultiply.r = r;
        colorMultiply.g = g;
        colorMultiply.b = b;
        colorMultiply.a = a;
    }
} // gdraw
