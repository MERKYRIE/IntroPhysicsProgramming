//
// Created by Gaëtan Blaise-Cazalet on 31/01/2025.
//

#include "SphereCube.hpp"

#include <VertexTypes.hpp>
#include <Renderer.hpp>

namespace gdraw
{
    void SphereCube::Load() {
        material.LoadVertexShader("PositionNormalTransform.vert", 0, 1, 0, 0);
        material.LoadFragmentShader("ColoredShape.frag", 1, 1, 0, 0);
        material.LoadTexture("cube.bmp", 4);
        material.CreateSampler(TextureFilter::Nearest, TextureWrap::ClampToEdge);
        material.CreatePipeline();
        mesh.Load();
    }

    void SphereCube::Update(f32 dt) {

    }

    void SphereCube::Bind() {
        material.Bind();
        mesh.Bind();
    }

    void SphereCube::Draw(const Renderer& renderer) {
        mesh.SetTransform(transform, renderer.GetView(), renderer.GetProjection());
        Bind();
        mesh.Draw();
    }

    void SphereCube::Unload() {
        mesh.Unload();
        material.Unload();
    }
}
