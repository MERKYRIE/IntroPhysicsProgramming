//
// Created by Gaëtan Blaise-Cazalet on 15/01/2025.
//

#ifndef GDRAW_QUADMESH_HPP
#define GDRAW_QUADMESH_HPP
#include <SDL3/SDL_gpu.h>

namespace gmath {
    class Mat4;
}

using gmath::Mat4;

namespace gdraw
{
    class Renderer;

    class QuadMesh
    {
    public:
        explicit QuadMesh(Renderer* renderer_);
        void Load() const;
        void Bind();
        void Draw();
        void Unload();
        void SetTransform(const Mat4& transform_);

    private:
        Renderer* renderer{nullptr};
        SDL_GPUBuffer* vertexBuffer{ nullptr };
        SDL_GPUBuffer* indexBuffer{ nullptr };
        float time;

        void Clear();
    };
}


#endif //GDRAW_QUAD_HPP
