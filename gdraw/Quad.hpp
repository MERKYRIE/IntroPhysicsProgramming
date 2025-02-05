//
// Created by Gaëtan Blaise-Cazalet on 15/01/2025.
//

#ifndef GDRAW_QUAD_HPP
#define GDRAW_QUAD_HPP

#include "QuadMaterial.hpp"
#include "QuadMesh.hpp"
#include "Mat4.hpp"
#include "Drawable.hpp"

using gmath::Mat4;

namespace gdraw
{
    class Quad : public Drawable
    {
    public:
        explicit Quad(Renderer& renderer_) : material(&renderer_), mesh(&renderer_) {}

        void Load();
        void Update(f32 dt);
        void Draw(const Renderer& renderer) override;
        void Unload();

    private:
        QuadMaterial material;
        QuadMesh mesh;
        Mat4 transform { Mat4::identity };
        f32 time { 0.0f };

        void Bind();
    };
}


#endif //GDRAW_QUAD_HPP
