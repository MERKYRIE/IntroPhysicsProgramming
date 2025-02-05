//
// Created by Gaëtan Blaise-Cazalet on 15/01/2025.
//

#include "QuadMesh.hpp"

#include <GpuUploader.hpp>
#include <Renderer.hpp>
#include <VertexTypes.hpp>
#include <Mat4.hpp>

namespace gdraw
{
    QuadMesh::QuadMesh(Renderer* renderer_) : renderer{renderer_} {
        // Create the vertex buffer
        SDL_GPUBufferCreateInfo vertexBufferCreateInfo =
        {
            .usage = SDL_GPU_BUFFERUSAGE_VERTEX,
            .size = sizeof(PositionTextureVertex) * 4
        };
        vertexBuffer = renderer->CreateBuffer(vertexBufferCreateInfo);

        // Create the index buffer
        SDL_GPUBufferCreateInfo indexBufferCreateInfo =
        {
            .usage = SDL_GPU_BUFFERUSAGE_INDEX,
            .size = sizeof(Uint16) * 6
        };
        indexBuffer = renderer->CreateBuffer(indexBufferCreateInfo);
    }

    void QuadMesh::Load() const {
        GPUUploader uploader{renderer->device};
        uploader.PrepareTransferBuffer((sizeof(PositionTextureVertex) * 4) + (sizeof(u16) * 6));

        // Map the transfer buffer and fill it with data (data is bound to the transfer buffer)
        auto transferData = static_cast<PositionTextureVertex*>(uploader.MapTransferBuffer(false));

        transferData[0] = PositionTextureVertex{-0.5f, 0.5f, 0, 0, 0};
        transferData[1] = PositionTextureVertex{0.5f, 0.5f, 0, 1, 0};
        transferData[2] = PositionTextureVertex{0.5f, -0.5f, 0, 1, 1};
        transferData[3] = PositionTextureVertex{-0.5f, -0.5f, 0, 0, 1};
        auto indexData = reinterpret_cast<u16*>(&transferData[4]);
        indexData[0] = 0;
        indexData[1] = 1;
        indexData[2] = 2;
        indexData[3] = 0;
        indexData[4] = 2;
        indexData[5] = 3;

        uploader.UnmapTransferBuffer();

        // Upload the transfer data to the vertex and index buffer
        SDL_GPUTransferBufferLocation transferVertexBufferLocation
        {
            .transfer_buffer = uploader.GetTransferBuffer(),
            .offset = 0
        };
        SDL_GPUBufferRegion vertexBufferRegion
        {
            .buffer = vertexBuffer,
            .offset = 0,
            .size = sizeof(PositionTextureVertex) * 4
        };
        SDL_GPUTransferBufferLocation transferIndexBufferLocation
        {
            .transfer_buffer = uploader.GetTransferBuffer(),
            .offset = sizeof(PositionTextureVertex) * 4
        };
        SDL_GPUBufferRegion indexBufferRegion
        {
            .buffer = indexBuffer,
            .offset = 0,
            .size = sizeof(u16) * 6
        };
        uploader.Begin();
        uploader.UploadToBuffer(transferVertexBufferLocation, vertexBufferRegion, false);
        uploader.UploadToBuffer(transferIndexBufferLocation, indexBufferRegion, false);
        uploader.End();
    }

    void QuadMesh::Bind() {
        SDL_GPUBufferBinding vertexBindings{.buffer = vertexBuffer, .offset = 0};
        renderer->BindVertexBuffers(0, vertexBindings, 1);

        SDL_GPUBufferBinding indexBindings{.buffer = indexBuffer, .offset = 0};
        renderer->BindIndexBuffer(indexBindings, SDL_GPU_INDEXELEMENTSIZE_16BIT);
    }

    void QuadMesh::Draw() {
        renderer->DrawIndexedPrimitives(6, 1, 0, 0, 0);
    }

    void QuadMesh::Unload() {
        Clear();
    }

    void QuadMesh::SetTransform(const Mat4& transform) {
        renderer->PushVertexUniformData(0, &transform, sizeof(transform));
    }

    void QuadMesh::Clear() {
        renderer->ReleaseBuffer(vertexBuffer);
        renderer->ReleaseBuffer(indexBuffer);
    }
}
