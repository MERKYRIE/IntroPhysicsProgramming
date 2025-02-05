//
// Created by gaetz on 31/01/2025.
//

#ifndef GDRAW_SPHERECUBEMATERIAL_HPP
#define GDRAW_SPHERECUBEMATERIAL_HPP

#include "Material.hpp"

namespace gdraw {

    class SphereCubeMaterial : public Material {
    public:
        explicit SphereCubeMaterial(Renderer* renderer_) : Material(renderer_) {}
        void CreatePipeline() override;
    };

} // gdraw

#endif //GDRAW_SPHERECUBEMATERIAL_HPP
