//
// Created by gaetz on 31/01/2025.
//

#include "SphereCubeMaterial.hpp"
#include <SDL3/SDL.h>
#include "Renderer.hpp"

namespace gdraw {

    void SphereCubeMaterial::CreatePipeline() {
        // Create the pipeline
        SDL_GPUGraphicsPipelineCreateInfo pipelineCreateInfo = {
                .vertex_shader = vertexShader,
                .fragment_shader = fragmentShader,
                // This is set up to match the vertex shader layout!
                .vertex_input_state = SDL_GPUVertexInputState {
                        .vertex_buffer_descriptions = new SDL_GPUVertexBufferDescription[1] {
                                {
                                        .slot = 0,
                                        .pitch = sizeof(PositionNormalVertex),
                                        .input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX,
                                        .instance_step_rate = 0,
                                }
                        },
                        .num_vertex_buffers = 1,
                        .vertex_attributes = new SDL_GPUVertexAttribute[2] {
                                {
                                        .location = 0,
                                        .buffer_slot = 0,
                                        .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
                                        .offset = 0
                                },
                                {
                                        .location = 1,
                                        .buffer_slot = 0,
                                        .format = SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3,
                                        .offset = sizeof(float) * 3
                                }
                        },
                        .num_vertex_attributes = 2,
                },
                .primitive_type = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST,
                .rasterizer_state = {
                        .fill_mode = SDL_GPU_FILLMODE_FILL,
                        .cull_mode = SDL_GPU_CULLMODE_BACK,
                        .front_face = SDL_GPU_FRONTFACE_CLOCKWISE,
                },
                .depth_stencil_state = {.compare_op = SDL_GPU_COMPAREOP_LESS, .enable_depth_test = true, .enable_depth_write = true},
                .target_info = {
                        .color_target_descriptions = new SDL_GPUColorTargetDescription[1] {{
                                                                                                   .format = SDL_GetGPUSwapchainTextureFormat(
                                                                                                           renderer->device,
                                                                                                           renderer->renderWindow),
                                                                                                   .blend_state = {
                                                                                                           .src_color_blendfactor = SDL_GPU_BLENDFACTOR_SRC_ALPHA,
                                                                                                           .dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA,
                                                                                                           .color_blend_op = SDL_GPU_BLENDOP_ADD,
                                                                                                           .src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_SRC_ALPHA,
                                                                                                           .dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA,
                                                                                                           .alpha_blend_op = SDL_GPU_BLENDOP_ADD,
                                                                                                           .enable_blend = true,
                                                                                                   }
                                                                                           }},
                        .num_color_targets = 1,
                        .depth_stencil_format = SDL_GPU_TEXTUREFORMAT_D16_UNORM,
                        .has_depth_stencil_target = true
                },
        };
        pipeline = SDL_CreateGPUGraphicsPipeline(renderer->device, &pipelineCreateInfo);
        renderer->ReleaseShader(vertexShader);
        renderer->ReleaseShader(fragmentShader);
    }
} // gdraw