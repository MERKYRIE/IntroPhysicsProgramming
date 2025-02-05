#version 450

layout(location = 0) in vec4 fragNormal;
layout(location = 0) out vec4 outColor;

const float pi = 3.141519;

vec3 GetColorFromPositionAndNormal(vec3 worldPosition, vec3 normal) {
    vec3 scaledPos = worldPosition * pi * 2.0;
    vec3 scaledPos2 = worldPosition * pi * 2.0 / 10.0 + vec3(pi / 4.0);
    float s = cos(scaledPos2.x) * cos(scaledPos2.y) * cos(scaledPos2.z);
    float t = cos(scaledPos.x) * cos(scaledPos.y) * cos(scaledPos.z);

    vec3 colorMultiplier = vec3(0.5, 0.5, 1.0);
    if (abs(normal.x) > abs(normal.y) && abs(normal.x) > abs(normal.z)) {
        colorMultiplier = vec3(1.0, 0.5, 0.5);
    } else if (abs(normal.y) > abs(normal.x) && abs(normal.y) > abs(normal.z)) {
        colorMultiplier = vec3(0.5, 1.0, 0.5);
    }

    t = ceil(t * 0.9);
    s = (ceil(s * 0.9) + 3.0) * 0.25;
    vec3 colorB = vec3(0.85, 0.85, 0.85);
    vec3 colorA = vec3(1.0, 1.0, 1.0);
    vec3 finalColor = mix(colorA, colorB, t) * s;

    return colorMultiplier * finalColor;
}

void main() {
    float dx = 0.25;
    float dy = 0.25;
    vec3 colorMultiplier = vec3(0.0, 0.0, 0.0);
    for (float y = 0.0; y < 1.0; y += dy) {
        for (float x = 0.0; x < 1.0; x += dx) {
            vec4 samplePos = vec4(gl_FragCoord.xyz + dFdx(gl_FragCoord.xyz) * x + dFdy(gl_FragCoord.xyz) * y, 1.0);
            colorMultiplier += GetColorFromPositionAndNormal(samplePos.xyz, fragNormal.xyz) * dx * dy;
        }
    }

    outColor = vec4(colorMultiplier, 1.0);
}