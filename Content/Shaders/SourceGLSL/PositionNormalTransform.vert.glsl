#version 450

layout(set = 1, binding = 0) uniform UBO {
    mat4 transform;
    mat4 view;
    mat4 projection;
};

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inNormal;

layout(location = 0) out vec4 fragNormal;

void main() {
    vec4 worldPosition = transform * vec4(inPosition, 1.0);
    vec4 viewPosition = view * worldPosition;
    gl_Position = projection * viewPosition;
    fragNormal = inNormal;
}