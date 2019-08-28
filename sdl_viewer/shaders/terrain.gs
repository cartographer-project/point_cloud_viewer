#version 410 core

layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in VS_OUT {
    vec4 color;
    uint quads;
} gs_in[];

out vec4 color;

void main() {
  uint render_quad = gs_in[0].quads & gs_in[1].quads & gs_in[2].quads;
  if (render_quad > 0) {
	  gl_Position = gl_in[0].gl_Position;
	  color = gs_in[0].color;
	  EmitVertex();
	  gl_Position = gl_in[1].gl_Position;
	  color = gs_in[1].color;
	  EmitVertex();
	  gl_Position = gl_in[2].gl_Position;
	  color = gs_in[2].color;
	  EmitVertex();
	  EndPrimitive();
	}
}
