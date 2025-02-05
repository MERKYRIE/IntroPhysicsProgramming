//
// Created by Gaëtan Blaise-Cazalet on 28/01/2025.
//

#include "SphereCubeMesh.hpp"

#include <GpuUploader.hpp>
#include <Renderer.hpp>
#include <VertexTypes.hpp>
#include <Mat4.hpp>
#include <Vec.hpp>

namespace gdraw
{
    SphereCubeMesh::SphereCubeMesh(Renderer* renderer_) : renderer{renderer_} {
        Build();
        // Create the vertex buffer
        SDL_GPUBufferCreateInfo vertexBufferCreateInfo =
        {
            .usage = SDL_GPU_BUFFERUSAGE_VERTEX,
            .size = static_cast<u32>(sizeof(PositionNormalVertex)) * vertexCount
        };
        vertexBuffer = renderer->CreateBuffer(vertexBufferCreateInfo);

        // Create the index buffer
        SDL_GPUBufferCreateInfo indexBufferCreateInfo =
        {
            .usage = SDL_GPU_BUFFERUSAGE_INDEX,
            .size = static_cast<u32>(sizeof(Uint16)) * quadCount * 6
        };
        indexBuffer = renderer->CreateBuffer(indexBufferCreateInfo);
    }

    void SphereCubeMesh::Load() {
        GPUUploader uploader{renderer->device};
        i32 indexCount = quadCount * 6;
        uploader.PrepareTransferBuffer((sizeof(PositionNormalVertex) * vertexCount) + (sizeof(u16) * indexCount));

        // Map the transfer buffer and fill it with data (data is bound to the transfer buffer)
        auto transferData = static_cast<PositionNormalVertex*>(uploader.MapTransferBuffer(false));

        for (i32 i = 0; i < vertexCount; ++i) {
            transferData[i] = PositionNormalVertex{ vertices[i].x, vertices[i].y, vertices[i].z, normals[i].x, normals[i].y, normals[i].z };
        }
        auto indexData = reinterpret_cast<u16*>(&transferData[vertexCount]);
        for (i32 i = 0; i < indexCount; ++i) {
            indexData[i] = indices[i];
        }
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
            .size = static_cast<u32>(sizeof(PositionNormalVertex)) * vertexCount
        };
        SDL_GPUTransferBufferLocation transferIndexBufferLocation
        {
            .transfer_buffer = uploader.GetTransferBuffer(),
            .offset = static_cast<u32>(sizeof(PositionNormalVertex)) * vertexCount
        };
        SDL_GPUBufferRegion indexBufferRegion
        {
            .buffer = indexBuffer,
            .offset = 0,
            .size = static_cast<u32>(sizeof(u16)) * indexCount
        };
        uploader.Begin();
        uploader.UploadToBuffer(transferVertexBufferLocation, vertexBufferRegion, false);
        uploader.UploadToBuffer(transferIndexBufferLocation, indexBufferRegion, false);
        uploader.End();
    }

    void SphereCubeMesh::Bind() {
        SDL_GPUBufferBinding vertexBindings{.buffer = vertexBuffer, .offset = 0};
        renderer->BindVertexBuffers(0, vertexBindings, 1);

        SDL_GPUBufferBinding indexBindings{.buffer = indexBuffer, .offset = 0};
        renderer->BindIndexBuffer(indexBindings, SDL_GPU_INDEXELEMENTSIZE_16BIT);
    }

    void SphereCubeMesh::Draw() {
        renderer->DrawIndexedPrimitives(quadCount * 6, 1, 0, 0, 0);
    }

    void SphereCubeMesh::Unload() {
        Clear();
    }

    void SphereCubeMesh::SetTransform(const Mat4& transform, const Mat4& view, const Mat4& projection) {
        UboData uboData{};
        memcpy(uboData.model, transform.ToArrayConst(), sizeof(uboData.model));
        memcpy(uboData.view, view.ToArrayConst(), sizeof(uboData.view));
        memcpy(uboData.projection, projection.ToArrayConst(), sizeof(uboData.projection));
        renderer->PushVertexUniformData(0, &uboData, sizeof(UboData));
    }

    void SphereCubeMesh::Clear() {
        renderer->ReleaseBuffer(vertexBuffer);
        renderer->ReleaseBuffer(indexBuffer);
    }

    i32 SphereCubeMesh::SetQuadIndices(i32 i, i32 v00, i32 v10, i32 v01, i32 v11) {
        indices[i] = v00;
        indices[i + 1] = indices[i + 4] = v01;
        indices[i + 2] = indices[i + 3] = v10;
        indices[i + 5] = v11;
        return i + 6;
    }

    i32 SphereCubeMesh::CreateTopFace(i32 t, i32 ring) {
        i32 v = ring * gridSize;
        for (i32 x = 0; x < gridSize - 1; x++, v++) {
            t = SetQuadIndices(t, v, v + 1, v + ring - 1, v + ring);
        }
        t = SetQuadIndices(t, v, v + 1, v + ring - 1, v + 2);

        i32 vMin = ring * (gridSize + 1) - 1;
        i32 vMid = vMin + 1;
        i32 vMax = v + 2;

        for (i32 z = 1; z < gridSize - 1; z++, vMin--, vMid++, vMax++) {
            t = SetQuadIndices(t, vMin, vMid, vMin - 1, vMid + gridSize - 1);
            for (i32 x = 1; x < gridSize - 1; x++, vMid++) {
                t = SetQuadIndices(t, vMid, vMid + 1, vMid + gridSize - 1, vMid + gridSize);
            }
            t = SetQuadIndices(t, vMid, vMax, vMid + gridSize - 1, vMax + 1);
        }

        i32 vTop = vMin - 2;
        t = SetQuadIndices(t, vMin, vMid, vTop + 1, vTop);
        for (i32 x = 1; x < gridSize - 1; x++, vTop--, vMid++) {
            t = SetQuadIndices(t, vMid, vMid + 1, vTop, vTop - 1);
        }
        t = SetQuadIndices(t, vMid, vTop - 2, vTop, vTop - 1);

        return t;
    }

    i32 SphereCubeMesh::CreateBottomFace(i32 t, i32 ring) {
        i32 v = 1;
        i32 vMid = vertexCount - (gridSize - 1) * (gridSize - 1);
        t = SetQuadIndices(t, ring - 1, vMid, 0, 1);
        for (i32 x = 1; x < gridSize - 1; x++, v++, vMid++) {
            t = SetQuadIndices(t, vMid, vMid + 1, v, v + 1);
        }
        t = SetQuadIndices(t, vMid, v + 2, v, v + 1);

        i32 vMin = ring - 2;
        vMid -= gridSize - 2;
        i32 vMax = v + 2;

        for (i32 z = 1; z < gridSize - 1; z++, vMin--, vMid++, vMax++) {
            t = SetQuadIndices(t, vMin, vMid + gridSize - 1, vMin + 1, vMid);
            for (i32 x = 1; x < gridSize - 1; x++, vMid++) {
                t = SetQuadIndices(t, vMid + gridSize - 1, vMid + gridSize, vMid, vMid + 1);
            }
            t = SetQuadIndices(t, vMid + gridSize - 1, vMax + 1, vMid, vMax);
        }

        i32 vTop = vMin - 1;
        t = SetQuadIndices(t, vTop + 1, vTop, vTop + 2, vMid);
        for (i32 x = 1; x < gridSize - 1; x++, vTop--, vMid++) {
            t = SetQuadIndices(t, vTop, vTop - 1, vMid, vMid + 1);
        }
        t = SetQuadIndices(t, vTop, vTop - 1, vMid, vTop - 2);

        return t;
    }

    void SphereCubeMesh::SetVertex(i32 i, i32 x, i32 y, i32 z) {
        Vec v = Vec {static_cast<f32>(x), static_cast<f32>(y), static_cast<f32>(z) } * 2.0f / gridSize - Vec::one;
        normals[i] = v.Normalize();
        vertices[i] = normals[i] * radius;
    }

    void SphereCubeMesh::Build() {
        // Setup geometry data
        i32 cornerVertices { 8 };
        i32 edgeVertices { (gridSize + gridSize + gridSize - 3) * 4 };
        i32 faceVertices { ((gridSize - 1) * (gridSize - 1) + (gridSize - 1) * (gridSize - 1) + (gridSize - 1) * (gridSize - 1)) * 2 };
        vertexCount = cornerVertices + edgeVertices + faceVertices;
        quadCount = (gridSize * gridSize + gridSize * gridSize + gridSize * gridSize) * 2;

        vertices.reserve(vertexCount);
        vertices.resize(vertexCount);
        normals.reserve(vertexCount);
        normals.resize(vertexCount);
        indices.reserve(quadCount * 6);
        indices.resize(quadCount * 6);

        // Generate vertices
        i32 v { 0 };
        // -- Lateral faces
        for (i32 y = 0; y <= gridSize; y++) {
            for (i32 x = 0; x <= gridSize; x++) {
                SetVertex(v++, x, y, 0);
            }
            for (i32 z = 1; z <= gridSize; z++) {
                SetVertex(v++, gridSize, y, z);
            }
            for (i32 x = gridSize - 1; x >= 0; x--) {
                SetVertex(v++, x, y, gridSize);
            }
            for (i32 z = gridSize - 1; z > 0; z--) {
                SetVertex(v++, 0, y, z);
            }
        }
        // -- Top and bottom
        for (i32 z = 1; z < gridSize; z++) {
            for (i32 x = 1; x < gridSize; x++) {
                SetVertex(v++, x, gridSize, z);
            }
        }
        for (i32 z = 1; z < gridSize; z++) {
            for (i32 x = 1; x < gridSize; x++) {
                SetVertex(v++, x, 0, z);
            }
        }

        // Generate indices
        i32 ring { (gridSize + gridSize) * 2 };
        i32 t { 0 };
        v = 0;
        // -- Lateral faces
        for (i32 y = 0; y < gridSize; y++, v++) {
            for (i32 q = 0; q < ring - 1; q++, v++) {
                t = SetQuadIndices(t, v, v + 1, v + ring, v + ring + 1);
            }
            t = SetQuadIndices(t, v, v - ring + 1, v + ring, v + 1);
        }
        // -- Top and bottom
        t = CreateTopFace(t, ring);
        t = CreateBottomFace(t, ring);
    }
}
