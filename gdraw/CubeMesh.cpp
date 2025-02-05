//
// Created by Gaëtan Blaise-Cazalet on 28/01/2025.
//

#include "CubeMesh.hpp"

#include <GpuUploader.hpp>
#include <Renderer.hpp>
#include <VertexTypes.hpp>
#include <Mat4.hpp>

namespace gdraw
{
    CubeMesh::CubeMesh(Renderer* renderer_) : renderer{renderer_} {
        // Create the vertex buffer
        SDL_GPUBufferCreateInfo vertexBufferCreateInfo =
        {
            .usage = SDL_GPU_BUFFERUSAGE_VERTEX,
            .size = sizeof(PositionTextureVertex) * 8
        };
        vertexBuffer = renderer->CreateBuffer(vertexBufferCreateInfo);

        // Create the index buffer
        SDL_GPUBufferCreateInfo indexBufferCreateInfo =
        {
            .usage = SDL_GPU_BUFFERUSAGE_INDEX,
            .size = sizeof(Uint16) * 36
        };
        indexBuffer = renderer->CreateBuffer(indexBufferCreateInfo);
    }

    void CubeMesh::Load() const {
        GPUUploader uploader{renderer->device};
        uploader.PrepareTransferBuffer((sizeof(PositionTextureVertex) * 8) + (sizeof(u16) * 36));

        // Map the transfer buffer and fill it with data (data is bound to the transfer buffer)
        auto transferData = static_cast<PositionTextureVertex*>(uploader.MapTransferBuffer(false));

        transferData[0] = PositionTextureVertex{ -0.5f, -0.5f,  0.5f, 0, 0 };
        transferData[1] = PositionTextureVertex{ 0.5f, -0.5f,  0.5f, 1, 0 };
        transferData[2] = PositionTextureVertex{ 0.5f,  0.5f,  0.5f, 1, 1 };
        transferData[3] = PositionTextureVertex{ -0.5f,  0.5f,  0.5f, 0, 1 };

        transferData[4] = PositionTextureVertex{ -0.5f, -0.5f, -0.5f, 0, 0 };
        transferData[5] = PositionTextureVertex{ 0.5f, -0.5f, -0.5f, 1, 0 };
        transferData[6] = PositionTextureVertex{ 0.5f,  0.5f, -0.5f, 1, 1 };
        transferData[7] = PositionTextureVertex{ -0.5f,  0.5f, -0.5f, 0, 1 };

        auto indexData = reinterpret_cast<u16*>(&transferData[8]);
        u16 indices[] = {
                // Front
                0, 1, 2,
                0, 2, 3,

                // Back
                4, 6, 5,
                4, 7, 6,

                // Left
                4, 0, 3,
                4, 3, 7,

                // Right
                1, 5, 6,
                1, 6, 2,

                // Bottom
                4, 5, 1,
                4, 1, 0,

                // Top
                3, 2, 6,
                3, 6, 7
        };
        std::memcpy(indexData, indices, sizeof(indices));

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
            .size = sizeof(PositionTextureVertex) * 8
        };
        SDL_GPUTransferBufferLocation transferIndexBufferLocation
        {
            .transfer_buffer = uploader.GetTransferBuffer(),
            .offset = sizeof(PositionTextureVertex) * 8
        };
        SDL_GPUBufferRegion indexBufferRegion
        {
            .buffer = indexBuffer,
            .offset = 0,
            .size = sizeof(u16) * 36
        };
        uploader.Begin();
        uploader.UploadToBuffer(transferVertexBufferLocation, vertexBufferRegion, false);
        uploader.UploadToBuffer(transferIndexBufferLocation, indexBufferRegion, false);
        uploader.End();
    }

    void CubeMesh::Bind() {
        SDL_GPUBufferBinding vertexBindings{.buffer = vertexBuffer, .offset = 0};
        renderer->BindVertexBuffers(0, vertexBindings, 1);

        SDL_GPUBufferBinding indexBindings{.buffer = indexBuffer, .offset = 0};
        renderer->BindIndexBuffer(indexBindings, SDL_GPU_INDEXELEMENTSIZE_16BIT);
    }

    void CubeMesh::Draw() {
        renderer->DrawIndexedPrimitives(36, 1, 0, 0, 0);
    }

    void CubeMesh::Unload() {
        Clear();
    }

    void CubeMesh::SetTransform(const Mat4& transform) {
        renderer->PushVertexUniformData(0, &transform, sizeof(transform));
    }

    void CubeMesh::Clear() {
        renderer->ReleaseBuffer(vertexBuffer);
        renderer->ReleaseBuffer(indexBuffer);
    }
}
