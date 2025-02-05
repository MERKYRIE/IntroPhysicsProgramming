//
// Created by gaetz on 28/01/2025.
//

#ifndef GDRAW_CUBEMATERIAL_HPP
#define GDRAW_CUBEMATERIAL_HPP

#include "Material.hpp"

namespace gdraw {

    class CubeMaterial : public Material {
    public:
        explicit CubeMaterial(Renderer* renderer_) : Material(renderer_) {}
        void CreatePipeline() override;
    };

} // gdraw

#endif //GDRAW_CUBEMATERIAL_HPP
