#version 410 core

layout(location = 0) in dvec3 position;

uniform dmat4 transform;

void main() { gl_Position = vec4(transform * dvec4(position, 1.0lf)); }
