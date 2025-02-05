cmake_minimum_required(VERSION 3.20)
project(physics-with-SDL3)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_policy(SET CMP0135 NEW)

# SDL3
set(SDL3_VERSION 3.2.0)
find_package(SDL3 ${SDL3_VERSION} QUIET) # QUIET or REQUIRED
if (NOT SDL3_FOUND) # If there's none, fetch and build SDL3
    include(FetchContent)
    if (UNIX)
        FetchContent_Declare(
                SDL3
                URL https://github.com/libsdl-org/SDL/releases/download/release-${SDL3_VERSION}/SDL3-${SDL3_VERSION}.tar.gz
        )
        FetchContent_MakeAvailable(SDL3)
    elseif (WIN32)
        FetchContent_Declare(
                SDL3
                URL https://github.com/libsdl-org/SDL/releases/download/release-${SDL3_VERSION}/SDL3-${SDL3_VERSION}.zip
        )
        FetchContent_MakeAvailable(SDL3)
    endif()
endif()

# SDL Shadercross

## Installing SDL_shadercross, for windows:
#
# A zipfile with the SDL_shadercross executable and the SDL3.dll file can be found the course shared folder.
# It has been compiled for windows x64. If you wish to compile it yourself, follow the instructions below.
#
## ----------------------------------------
## Thanks to TrojanFoe, who provided a partial tutorial: https://discourse.libsdl.org/t/instructions-on-building-sdl-shadercross-on-macos-and-windows/56906
## ----------------------------------------
# 1. Install Chocolatey:
#  - Open PowerShell as Administrator:
#  > Get-ExecutionPolicy
#  - If it returns Restricted, then run Set-ExecutionPolicy AllSigned or Set-ExecutionPolicy Bypass -Scope Process.
#  > Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
#  - Test by typing choco or choco -?
#
# 2. Install ninja:
#  - Run from an Administrator:
#  > choco install ninja
#
# 3. Clone SDL then compile it in Release mode:
#   > git clone https://github.com/libsdl-org/SDL
#   > cd SDL
#   > cmake -S. -Bbuild -GNinja -DCMAKE_BUILD_TYPE=Release
#   > cmake --build build
#   > cd ..
#
# 4. Clone SDL_shadercross then compile it in Release mode:
#   > git clone --recursive https://github.com/libsdl-org/SDL_shadercross
#   > cd SDL_shadercross
#   > git submodule update --remote
#   > cmake -S. -Bbuild -GNinja -DCMAKE_BUILD_TYPE=Release -DSDLSHADERCROSS_VENDORED=ON -DSDL3_DIR=../SDL/build
#   > cmake --build build
#
# 5. Copy the SDL_shadercross executable to a folder in your PATH, with the SDL3.dll file in the same folder.
#
# 6. You can now run the compile bat/sh file from Content/Shaders/ to compile the shaders.

#find_package(SDLSHADERCROSS QUIET)
#if (NOT SDLSHADERCROSS_FOUND)
#    include(FetchContent)
#    FetchContent_Declare(
#            SDLSHADERCROSS
#            GIT_REPOSITORY https://github.com/libsdl-org/SDL_shadercross.git
#            GIT_TAG main
#    )
#    FetchContent_MakeAvailable(SDLSHADERCROSS)
#endif()

add_subdirectory(gplatform)
add_subdirectory(gmath)
add_subdirectory(gphysics)
add_subdirectory(gdraw)

file( GLOB graphics-with-SDL3_SOURCES *.cpp )
add_executable(${PROJECT_NAME} ${graphics-with-SDL3_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC ${SDL3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} SDL3::SDL3 gplatform gphysics gdraw)