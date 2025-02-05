cmake_minimum_required(VERSION 3.20)
project(gphysics)
set(CMAKE_CXX_STANDARD 20)

file( GLOB gphysics_SOURCES *.cpp )
add_library( ${PROJECT_NAME} ${gphysics_SOURCES} )
target_include_directories( ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} gmath gplatform gdraw)
target_link_libraries( ${PROJECT_NAME} gplatform gmath gdraw)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(${PROJECT_NAME} PUBLIC DEBUG=1)
else()
    target_compile_definitions(${PROJECT_NAME} PUBLIC DEBUG=0)
endif()