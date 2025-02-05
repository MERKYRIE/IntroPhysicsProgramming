//
// Created by Gaëtan Blaise-Cazalet on 28/01/2025.
//

#ifndef GDRAW_CUBE_HPP
#define GDRAW_CUBE_HPP

#include "CubeMaterial.hpp"
#include "CubeMesh.hpp"
#include "Mat4.hpp"
#include "Drawable.hpp"

using gmath::Mat4;

namespace gdraw
{
    class Cube : public Drawable
    {
    public:
        explicit Cube(Renderer& renderer_) : material(&renderer_), mesh(&renderer_) {}

        void Load();
        void Update(f32 dt);
        void Draw(const Renderer& renderer);
        void Unload();

    private:
        CubeMaterial material;
        CubeMesh mesh;
        Mat4 transform { Mat4::identity };
        f32 time { 0.0f };

        void Bind();
    };
}


#endif //GDRAW_CUBE_HPP
