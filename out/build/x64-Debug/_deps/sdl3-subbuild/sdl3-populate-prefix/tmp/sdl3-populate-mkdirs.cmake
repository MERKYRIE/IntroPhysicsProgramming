# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "D:/Programs/_Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-src")
  file(MAKE_DIRECTORY "D:/Programs/_Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-src")
endif()
file(MAKE_DIRECTORY
  "D:/Programs/_Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-build"
  "D:/Programs/_Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix"
  "D:/Programs/_Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix/tmp"
  "D:/Programs/_Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix/src/sdl3-populate-stamp"
  "D:/Programs/_Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix/src"
  "D:/Programs/_Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix/src/sdl3-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Programs/_Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix/src/sdl3-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Programs/_Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix/src/sdl3-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
