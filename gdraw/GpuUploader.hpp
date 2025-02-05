//
// Created by Gaëtan Blaise-Cazalet on 15/01/2025.
//

#ifndef GDRAW_GPUUPLOADER_HPP
#define GDRAW_GPUUPLOADER_HPP

#include <SDL3/SDL_gpu.h>
#include <cstring>
#include <Defines.hpp>

class GPUUploader {
public:
    explicit GPUUploader(SDL_GPUDevice* device_);

    template<typename T>
    void PrepareTextureData(SDL_Surface* surface) {
        u32 bufferSize = surface->w * surface->h * 4;
        PrepareTransferBuffer(bufferSize);
        // Mapping data
        auto textureTransferData = static_cast<T*>(SDL_MapGPUTransferBuffer(
            device, transferBuffer, false));
        std::memcpy(textureTransferData, surface->pixels, bufferSize);
        SDL_UnmapGPUTransferBuffer(device, transferBuffer);
    }

    void PrepareTransferBuffer(u32 size);
    void* MapTransferBuffer(bool cycle) const;
    void UnmapTransferBuffer() const ;

    void Begin();
    void UploadToBuffer(const SDL_GPUTransferBufferLocation& source, const SDL_GPUBufferRegion& destination, bool cycle) const;
    void UploadToTexture(const SDL_GPUTextureTransferInfo& source, const SDL_GPUTextureRegion& destination, bool cycle) const;
    void End();

    SDL_GPUTransferBuffer* GetTransferBuffer() const { return transferBuffer; }

private:
    SDL_GPUDevice* device {nullptr};
    SDL_GPUTransferBuffer* transferBuffer {nullptr};
    SDL_GPUCommandBuffer* uploadCmdBuf {nullptr};
    SDL_GPUCopyPass* copyPass {nullptr};
};



#endif //GDRAW_GPUUPLOADER_HPP
