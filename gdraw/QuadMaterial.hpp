//
// Created by gaetz on 28/01/2025.
//

#ifndef GDRAW_QUADMATERIAL_HPP
#define GDRAW_QUADMATERIAL_HPP

#include "Material.hpp"

namespace gdraw {

    class QuadMaterial : public Material {
    public:
        explicit QuadMaterial(Renderer* renderer_) : Material(renderer_) {}
        void CreatePipeline() override;
    };

} // gdraw

#endif //GDRAW_QUADMATERIAL_HPP
