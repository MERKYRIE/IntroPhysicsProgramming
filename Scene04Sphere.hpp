//
// Created by Gaëtan Blaise-Cazalet on 31/01/2024.
//

#ifndef SCENE04SPHERE_HPP
#define SCENE04SPHERE_HPP

#include <SphereCube.hpp>

#include "Scene.hpp"

using gdraw::SphereCube;

class Scene04Sphere : public Scene {
public:
    void Load(Renderer& renderer) override;
    bool Update(float dt) override;
    void Draw(Renderer& renderer) override;
    void Unload(Renderer& renderer) override;

private:
    uptr<SphereCube> sphere;
    InputState inputState;
};



#endif //SCENE04SPHERE_HPP