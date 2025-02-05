//
// Created by Gaëtan Blaise-Cazalet on 19/11/2024.
//

#include "Renderer.hpp"

#include <SDL3/SDL_assert.h>

#include "Window.hpp"
#include <SDL3/SDL_log.h>

namespace gdraw {

    void Renderer::Init(Window& window) {
        // Init device
        renderWindow = window.sdlWindow;
        device = SDL_CreateGPUDevice(
                SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_DXIL | SDL_GPU_SHADERFORMAT_MSL,
                true,
                nullptr);
        SDL_ClaimWindowForGPUDevice(device, renderWindow);

        // Init depth texture
        SDL_GPUTextureCreateInfo depthTextureInfo {
                .type = SDL_GPU_TEXTURETYPE_2D,
                .format = SDL_GPU_TEXTUREFORMAT_D16_UNORM,
                .usage = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET,
                .width = static_cast<u32>(window.width),
                .height = static_cast<u32>(window.height),
                .layer_count_or_depth = 1,
                .num_levels = 1,
        };
        depthTexture = SDL_CreateGPUTexture(device, &depthTextureInfo);

        // Init projection matrix
        projection = Mat4::CreatePerspectiveFieldOfView(45.0f, static_cast<f32>(window.width) / static_cast<f32>(window.height), 0.1f, 1000000.0f);
    }

    void Renderer::Begin() {
        cmdBuffer = SDL_AcquireGPUCommandBuffer(device);
        if (cmdBuffer == nullptr) {
            SDL_Log("AcquireGPUCommandBuffer failed: %s", SDL_GetError());
        }

        if (!SDL_WaitAndAcquireGPUSwapchainTexture(cmdBuffer, renderWindow, &swapchainTexture, nullptr, nullptr)) {
            SDL_Log("AcquireGPUSwapchainTexture failed: %s", SDL_GetError());
        }

        if (swapchainTexture != nullptr) {
            // Color
            SDL_GPUColorTargetInfo colorTargetInfo = {};
            colorTargetInfo.texture = swapchainTexture;
            colorTargetInfo.clear_color = SDL_FColor { 0.392f, 0.584f, 0.929f, 1.0f };
            colorTargetInfo.load_op = SDL_GPU_LOADOP_CLEAR;
            colorTargetInfo.store_op = SDL_GPU_STOREOP_STORE;

            // Depth
            SDL_GPUDepthStencilTargetInfo depthStencilTargetInfo;
            SDL_zero(depthStencilTargetInfo);
            depthStencilTargetInfo.clear_depth = 1.0f;
            depthStencilTargetInfo.load_op = SDL_GPU_LOADOP_CLEAR;
            depthStencilTargetInfo.store_op = SDL_GPU_STOREOP_DONT_CARE;
            depthStencilTargetInfo.stencil_load_op = SDL_GPU_LOADOP_DONT_CARE;
            depthStencilTargetInfo.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;
            depthStencilTargetInfo.texture = depthTexture;
            depthStencilTargetInfo.cycle = true;

            renderPass = SDL_BeginGPURenderPass(cmdBuffer, &colorTargetInfo, 1, &depthStencilTargetInfo);
        }
    }

    void Renderer::End() const {
        SDL_EndGPURenderPass(renderPass);
        SDL_SubmitGPUCommandBuffer(cmdBuffer);
    }

    void Renderer::Close() const {
        SDL_ReleaseGPUTexture(device, depthTexture);
        SDL_ReleaseWindowFromGPUDevice(device, renderWindow);
        SDL_DestroyGPUDevice(device);
    }

    void Renderer::BindGraphicsPipeline(SDL_GPUGraphicsPipeline* pipeline) const {
        SDL_BindGPUGraphicsPipeline(renderPass, pipeline);
    }

    void Renderer::SetViewport(const SDL_GPUViewport& viewport) const {
        SDL_SetGPUViewport(renderPass, &viewport);
    }

    void Renderer::SetScissorRect(const SDL_Rect& rect) const {
        SDL_SetGPUScissor(renderPass, &rect);
    }

    void Renderer::SetStencilReference(Uint8 stencilReference) const {
        SDL_SetGPUStencilReference(renderPass, stencilReference);
    }

    bool Renderer::DoesTextureSupportFormat(SDL_GPUTextureFormat format, SDL_GPUTextureType type,
                                            SDL_GPUTextureUsageFlags usageFlags) const {
        return SDL_GPUTextureSupportsFormat(device, format, type, usageFlags);
    }

    void Renderer::DrawPrimitives(int numVertices, int numInstances, int firstVertex, int firstInstance) const {
        SDL_DrawGPUPrimitives(renderPass, numVertices, numInstances, firstVertex, firstInstance);
    }

    void Renderer::DrawIndexedPrimitives(int numIndices, int numInstances, int firstIndex,
                                         int vertexOffset, int firstInstance) const {
        SDL_DrawGPUIndexedPrimitives(renderPass, numIndices, numInstances, firstIndex, vertexOffset, firstInstance);
    }

    SDL_GPUGraphicsPipeline*
    Renderer::CreateGPUGraphicsPipeline(const SDL_GPUGraphicsPipelineCreateInfo& createInfo) const {
        return SDL_CreateGPUGraphicsPipeline(device, &createInfo);
    }

    void Renderer::ReleaseShader(SDL_GPUShader* shader) const {
        SDL_ReleaseGPUShader(device, shader);
    }

    SDL_Surface* Renderer::LoadBMPImage(const char* basePath, const char* imageFilename, int desiredChannels) {
        char fullPath[256];
        SDL_PixelFormat format;
        SDL_snprintf(fullPath, sizeof(fullPath), "%sContent/Images/%s", basePath, imageFilename);

        SDL_Surface* result = SDL_LoadBMP(fullPath);
        if (result == nullptr) {
            SDL_Log("Failed to load BMP: %s", SDL_GetError());
            return nullptr;
        }

        if (desiredChannels == 4) {
            format = SDL_PIXELFORMAT_ABGR8888;
        } else {
            SDL_assert(!"Unexpected desiredChannels");
            SDL_DestroySurface(result);
            return nullptr;
        }

        if (result->format != format) {
            SDL_Surface* next = SDL_ConvertSurface(result, format);
            SDL_DestroySurface(result);
            result = next;
        }

        return result;
    }

    SDL_GPUSampler* Renderer::CreateSampler(const SDL_GPUSamplerCreateInfo& createInfo) const {
        return SDL_CreateGPUSampler(device, &createInfo);
    }

    void Renderer::ReleaseSurface(SDL_Surface* surface) const {
        SDL_DestroySurface(surface);
    }

    SDL_GPUBuffer* Renderer::CreateBuffer(const SDL_GPUBufferCreateInfo& createInfo) const {
        return SDL_CreateGPUBuffer(device, &createInfo);
    }

    void Renderer::SetBufferName(SDL_GPUBuffer* buffer, const str& name) const {
        SDL_SetGPUBufferName(device, buffer, name.c_str());
    }

    SDL_GPUTexture* Renderer::CreateTexture(const SDL_GPUTextureCreateInfo& createInfo) const {
        return SDL_CreateGPUTexture(device, &createInfo);
    }

    void Renderer::SetTextureName(SDL_GPUTexture* texture, const str& name) const {
        SDL_SetGPUTextureName(device, texture, name.c_str());
    }

    void Renderer::ReleaseTexture(SDL_GPUTexture* texture) const {
        SDL_ReleaseGPUTexture(device, texture);
    }

    void Renderer::ReleaseSampler(SDL_GPUSampler* sampler) const {
        SDL_ReleaseGPUSampler(device, sampler);
    }

    void Renderer::BindVertexBuffers(Uint32 firstSlot, const SDL_GPUBufferBinding& bindings, Uint32 numBindings) const {
        SDL_BindGPUVertexBuffers(renderPass, firstSlot, &bindings, numBindings);
    }

    void
    Renderer::BindIndexBuffer(const SDL_GPUBufferBinding& bindings, SDL_GPUIndexElementSize indexElementSize) const {
        SDL_BindGPUIndexBuffer(renderPass, &bindings, indexElementSize);
    }

    void Renderer::BindFragmentSamplers(Uint32 firstSlot, const SDL_GPUTextureSamplerBinding& bindings,
                                        Uint32 numBindings) const {
        SDL_BindGPUFragmentSamplers(renderPass, firstSlot, &bindings, numBindings);
    }

    void Renderer::ReleaseBuffer(SDL_GPUBuffer* buffer) const {
        SDL_ReleaseGPUBuffer(device, buffer);
    }

    void Renderer::ReleaseGraphicsPipeline(SDL_GPUGraphicsPipeline* pipeline) const {
        SDL_ReleaseGPUGraphicsPipeline(device, pipeline);
    }

    void Renderer::PushVertexUniformData(uint32_t slot, const void* data, Uint32 size) const {
        SDL_PushGPUVertexUniformData(cmdBuffer, 0, data, size);
    }

    void Renderer::PushFragmentUniformData(uint32_t slot, const void* data, Uint32 size) const {
        SDL_PushGPUFragmentUniformData(cmdBuffer, 0, data, size);
    }
}