//
// Created by Gaëtan Blaise-Cazalet on 19/11/2024.
//

#ifndef WINDOW_HPP
#define WINDOW_HPP

#include "SDL3/SDL_video.h"

namespace gdraw {
    class Window {
    public:
        SDL_Window* sdlWindow { nullptr };

        void Init();

        void Close() const;

        int width { 1280 };
        int height { 720 };
    };
}


#endif //WINDOW_HPP