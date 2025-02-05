//
// Created by Gaëtan Blaise-Cazalet on 19/11/2024.
//

#ifndef RENDERER_HPP
#define RENDERER_HPP

#include <SDL3/SDL_gpu.h>
#include <Defines.hpp>
#include <Mat4.hpp>
#include <Vec.hpp>

using gmath::Mat4;
using gmath::Vec;

namespace gdraw {

    class Window;

    class Renderer {
    public:
        void Init(Window& window);

        void Begin();

        void End() const;

        void Close() const;

        void ReleaseShader(SDL_GPUShader* shader) const;

        SDL_Surface* LoadBMPImage(const char* basePath, const char* imageFilename, int desiredChannels);

        SDL_GPUSampler* CreateSampler(const SDL_GPUSamplerCreateInfo& createInfo) const;

        void ReleaseSurface(SDL_Surface* surface) const;

        SDL_GPUTexture* CreateTexture(const SDL_GPUTextureCreateInfo& createInfo) const;

        void SetTextureName(SDL_GPUTexture* texture, const str& name) const;

        void ReleaseTexture(SDL_GPUTexture* texture) const;

        void ReleaseSampler(SDL_GPUSampler* sampler) const;


        SDL_GPUGraphicsPipeline* CreateGPUGraphicsPipeline(const SDL_GPUGraphicsPipelineCreateInfo& createInfo) const;

        void BindGraphicsPipeline(SDL_GPUGraphicsPipeline* pipeline) const;

        void BindVertexBuffers(Uint32 firstSlot, const SDL_GPUBufferBinding& bindings, Uint32 numBindings) const;

        void BindIndexBuffer(const SDL_GPUBufferBinding& bindings, SDL_GPUIndexElementSize indexElementSize) const;

        void
        BindFragmentSamplers(Uint32 firstSlot, const SDL_GPUTextureSamplerBinding& bindings, Uint32 numBindings) const;

        void DrawPrimitives(int numVertices, int numInstances, int firstVertex, int firstInstance) const;

        void DrawIndexedPrimitives(int numIndices, int numInstances, int firstIndex, int vertexOffset,
                                   int firstInstance) const;

        void SetViewport(const SDL_GPUViewport& viewport) const;

        void SetScissorRect(const SDL_Rect& rect) const;

        void SetStencilReference(Uint8 stencilReference) const;

        bool DoesTextureSupportFormat(SDL_GPUTextureFormat format, SDL_GPUTextureType type,
                                      SDL_GPUTextureUsageFlags usageFlags) const;


        SDL_GPUBuffer* CreateBuffer(const SDL_GPUBufferCreateInfo& createInfo) const;

        void SetBufferName(SDL_GPUBuffer* buffer, const str& name) const;

        void ReleaseBuffer(SDL_GPUBuffer* buffer) const;

        void ReleaseGraphicsPipeline(SDL_GPUGraphicsPipeline* pipeline) const;

        void PushVertexUniformData(uint32_t slot, const void* data, Uint32 size) const;

        void PushFragmentUniformData(uint32_t slot, const void* data, Uint32 size) const;

        SDL_GPUDevice* device { nullptr };
        SDL_Window* renderWindow { nullptr };
        SDL_GPUCommandBuffer* cmdBuffer { nullptr };
        SDL_GPUTexture* swapchainTexture { nullptr };
        SDL_GPUTexture* depthTexture { nullptr };
        SDL_GPURenderPass* renderPass { nullptr };

        [[nodiscard]] const Mat4& GetView() const {
            return view;
        }

        void SetView(const Mat4& view_) {
            view = view_;
        }

        [[nodiscard]] const Mat4& GetProjection() const {
            return projection;
        }

        void SetProjection(const Mat4& projection_) {
            projection = projection_;
        }

    private:
        Mat4 projection { Mat4::identity };
        Mat4 view { Mat4::CreateLookAt(Vec {0, 0, 10.0f}, Vec::zero, Vec::up) };
    };
}


#endif //RENDERER_HPP