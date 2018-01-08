#version 330 core

// inputs
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;

uniform mat4 world_to_gl;
uniform float edge_length;
uniform float size;
uniform float gamma;
uniform vec3 min;

// varying outputs
out vec4 v_color;

void main() {
    vec3 corrected_color = pow(color / 255., vec3(1.0 / gamma));
    v_color = vec4(corrected_color, 1.);
    gl_PointSize = size;
    gl_Position = world_to_gl * vec4(position * edge_length + min, 1.0);
}
