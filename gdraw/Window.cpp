#include "Window.hpp"
#include "Defines.hpp"

#include <string>
#include <SDL3/SDL.h>


namespace gdraw {
    void Window::Init() {
        SDL_Init(SDL_INIT_VIDEO);
        const str title { "Physics Engine" };
        sdlWindow = SDL_CreateWindow(title.c_str(), width, height, SDL_WINDOW_RESIZABLE);
    }

    void Window::Close() const {
        SDL_DestroyWindow(sdlWindow);
        SDL_Quit();
    }
}