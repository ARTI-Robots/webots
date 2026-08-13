#version 330 core

precision highp float;

in vec2 texUv;

out vec4 fragColor;

uniform sampler2D inputTextures[2];

void main() {
  vec3 rgb =
    texture(inputTextures[0], texUv).rgb;

  float range =
    texture(inputTextures[1], texUv).r;

  fragColor =
    vec4(rgb, range);
}
