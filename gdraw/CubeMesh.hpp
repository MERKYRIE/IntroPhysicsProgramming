//
// Created by Gaëtan Blaise-Cazalet on 28/01/2025.
//

#ifndef GDRAW_CUBEMESH_HPP
#define GDRAW_CUBEMESH_HPP
#include <SDL3/SDL_gpu.h>

namespace gmath {
    class Mat4;
}

using gmath::Mat4;

namespace gdraw
{
    class Renderer;

    class CubeMesh
    {
    public:
        explicit CubeMesh(Renderer* renderer_);
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


#endif //GDRAW_CUBEMESH_HPP
