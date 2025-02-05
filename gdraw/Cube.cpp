//
// Created by Gaëtan Blaise-Cazalet on 28/01/2025.
//

#include "Cube.hpp"

#include <VertexTypes.hpp>

namespace gdraw
{
    void Cube::Load() {
        material.LoadVertexShader("TexturedQuadWithMatrix.vert", 0, 1, 0, 0);
        material.LoadFragmentShader("TexturedQuadWithMultiplyColor.frag", 1, 1, 0, 0);
        material.LoadTexture("cube.bmp", 4);
        material.CreateSampler(TextureFilter::Nearest, TextureWrap::ClampToEdge);
        material.CreatePipeline();
        mesh.Load();
    }

    void Cube::Update(f32 dt) {
        time += dt;
        transform = Mat4::CreateRotationZ(time / 2.0f) * Mat4::CreateRotationX(time / 2.0f) * Mat4::CreateTranslation(0.0f, 0.0f, 0);
    }

    void Cube::Bind() {
        material.Bind();
        mesh.Bind();
        mesh.SetTransform(transform);
    }

    void Cube::Draw(const Renderer& renderer) {
        Bind();
        mesh.Draw();
    }

    void Cube::Unload() {
        mesh.Unload();
        material.Unload();
    }
}
