#version 410 core

layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

// The input is a length 3 array of vertices – a triangle.
in VS_OUT {
    vec4 color;
    uint quads;
} gs_in[];

out vec4 color;

// Why this stage?
// Since the terrain is sparse, we'd like to leave out some quads from rendering.
// Updating the actual mesh on the graphics card is inefficient, however. Instead,
// the shader should eliminate quads that are unset in the terrain.
// The easiest way to do that would be to set an indicator value in the vertex shader:
// vertices that have an elevation value are set to 1, and others to 0. The fragment
// shader can then set the opacity of all pixels with indicator value < 1 to 0 to hide
// them.¹ It's possible and what I'd recommend doing in WebGL. However, it is a little
// limited/inexact.

// Let's say we have the following elevation map (X indicates an existing value), and
// we want to render only quads whose four corners are set (this is actually not the
// only option, but what we do here) and not display others at all.

// +---X---X
// |   |   |
// |   |   |
// |   |   |
// +---+---X
// |   |   |
// |   |   |
// |   |   |
// X---X---X

// The mesh used to render the elevation map consists of triangles though:

// +---+---+
// |  /|  /|
// | / | / |
// |/  |/  |
// +---+---+
// |  /|  /|
// | / | / |
// |/  |/  |
// +---+---+

// And that structure will be visible, even though we'd like the user to think that we
// deal only with quads. For instance, the very lowest rightmost triangle will be rendered,
// but the triangles in the quad above it can be fully hidden by filtering indicator
// values < 1.

// The problem is that each triangle doesn't know anything about the vertex opposite of
// itself in the quad. That brings us to the solution that's implemented here: Each quad
// essentially has an id, and each vertex stores a list of its adjacent quads' ids, but
// only of those quads that should be rendered. What the geometry shader do is look at a
// triangle, and compute the intersection of the three vertices' quad lists. The result
// is the id of the quad that this triangle is part of, or nothing if the quad should not
// be rendered. If the result is nothing, the triangle is simply dropped.

// The list of quad ids can be implement efficiently as a uint, where each bit corresponds
// to a quad. That means there's only 32 quad ids, but they don't need to be globally
// unique, only within a triangle. List intersection then becomes the bitwise AND
// operation.

// ¹ Footnote: Alternatively, since the fragment shader will interpolate the indicator value
// linearly within a triangle, the opacity could be set to the indicator value itself,
// which will produce a fade-out effect at the borders of the terrain. You need to take
// care that the non-existing points still have sensible values though, because the triangle
// will still be semi-visible and it will look bad if one vertex has a nonsense position.

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
