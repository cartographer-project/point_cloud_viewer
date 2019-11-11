#version 410 core
// The vertex positions are in grid coordinates, i.e. between 0 and grid_size (inclusive).
layout(location = 0) in ivec3 aPos;

// These are constant throughout the program.
uniform dvec3 terrain_origin_m;
uniform double terrain_res_m;

// The grid coordinates of the min corner of the currently visible terrain (the window).
// Essentially, the terrain regular grid with resolution terrain_res_m and origin
// terrain_origin_m, and the mesh is rendering a rectangle within that grid.
uniform dvec2 terrain_pos;
// These two are equal and set by the GlMovingWindowTexture.
// They hold the coordinates (in pixels) of the min corner
// of the texture within the sampler.
uniform ivec2 height_texture_offset;
uniform ivec2 color_texture_offset;
// The full projective transform.
uniform dmat4 world_to_gl;
// The frame of the terrain.
// TODO(nnmm): This can also include the terrain_origin.
uniform dmat4 terrain_to_world;
// The textures.
uniform sampler2D height;
uniform sampler2D color;

// The data that's passed to the geometry shader.
out VS_OUT {
  vec4 color;
  uint quads;
} vs_out;

void main() {
  // Each vertex position (aPos) maps to a pixel in the texture, but
  // shifted (by height_texture_offset) with wraparound.
  ivec2 texCoord = height_texture_offset + aPos.xy;
  ivec2 texSize = textureSize(height, 0);
  ivec2 texCoordModSize = texCoord - (texCoord / texSize) * texSize;

  // Look up the height texture at mipmap level 0 (the only one).
  // It's a two-channel texture, so x and y are set, and x is the height.
  vec4 tex = texelFetch(height, texCoordModSize, 0);
  dvec4 local_pos = dvec4(terrain_origin_m, 1.0lf);
  // Calculate the vertex position in the terrain coordinate system.
  local_pos.xy += terrain_res_m * (dvec2(aPos.xy) + terrain_pos);
  local_pos.z += double(tex.x);
  // Transform from terrain coordinates to Gl camerae coordinates.
  gl_Position = vec4(world_to_gl * terrain_to_world * local_pos);
  // The second channel contains the quad adjacency list.
  vs_out.quads = uint(tex.y);
  // Look up the color.
  vs_out.color = texelFetch(color, texCoordModSize, 0);
  vs_out.color.w = 1.0f;
}
