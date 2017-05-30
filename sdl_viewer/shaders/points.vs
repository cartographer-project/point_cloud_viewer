#version 100

precision mediump float;

uniform mat4 world_to_gl;
uniform float edge_length;
uniform vec3 min;

attribute vec3 position;
attribute vec3 color;

varying vec4 v_color;

void main() {
  v_color = vec4(color / 255., 1.);
  // TODO(hrapp): Support point size as a uniform.
  gl_PointSize = 2.;
  // TODO(hrapp): In the WebGL viewer, we are CPU bound, so doing the decoding
  // on the GPU made sense. In the native viewer, the CPU is mostly idle, so
  // pre-decoding this on the CPU might be more performant.
  gl_Position = world_to_gl * vec4(position * edge_length + min, 1.0);
}
