# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/MRK/Visual Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-src"
  "D:/MRK/Visual Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-build"
  "D:/MRK/Visual Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix"
  "D:/MRK/Visual Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix/tmp"
  "D:/MRK/Visual Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix/src/sdl3-populate-stamp"
  "D:/MRK/Visual Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix/src"
  "D:/MRK/Visual Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix/src/sdl3-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/MRK/Visual Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix/src/sdl3-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/MRK/Visual Studio/Projects/IntroPhysicsProgramming/out/build/x64-Debug/_deps/sdl3-subbuild/sdl3-populate-prefix/src/sdl3-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
