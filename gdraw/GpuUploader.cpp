//
// Created by Gaëtan Blaise-Cazalet on 15/01/2025.
//

#include "GpuUploader.hpp"

GPUUploader::GPUUploader(SDL_GPUDevice* device_) :
    device{device_} {}

void GPUUploader::PrepareTransferBuffer(u32 size) {
    SDL_GPUTransferBufferCreateInfo transferBufferCreateInfo = {
        .usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD,
        .size = size
    };
    transferBuffer = SDL_CreateGPUTransferBuffer(device, &transferBufferCreateInfo);
}

void* GPUUploader::MapTransferBuffer(bool cycle) const {
    return SDL_MapGPUTransferBuffer(device, transferBuffer, cycle);
}

void GPUUploader::UnmapTransferBuffer() const { SDL_UnmapGPUTransferBuffer(device, transferBuffer); }

void GPUUploader::Begin() {
    uploadCmdBuf = SDL_AcquireGPUCommandBuffer(device);
    copyPass = SDL_BeginGPUCopyPass(uploadCmdBuf);
}

void GPUUploader::UploadToBuffer(const SDL_GPUTransferBufferLocation& source,
                                 const SDL_GPUBufferRegion& destination,
                                 bool cycle) const {
    SDL_UploadToGPUBuffer(copyPass, &source, &destination, cycle);
}

void GPUUploader::UploadToTexture(const SDL_GPUTextureTransferInfo& source,
                                  const SDL_GPUTextureRegion& destination, bool cycle) const {
    SDL_UploadToGPUTexture(copyPass, &source, &destination, cycle);
}

void GPUUploader::End() {
    SDL_EndGPUCopyPass(copyPass);
    SDL_SubmitGPUCommandBuffer(uploadCmdBuf);
    SDL_ReleaseGPUTransferBuffer(device, transferBuffer);
    transferBuffer = nullptr;
}
