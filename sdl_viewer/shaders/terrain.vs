#version 410 core
// The vertex positions are in grid coordinates, i.e. between 0 and grid_size (inclusive).
// The grid scale is one less than a power of two.
layout(location = 0) in ivec3 aPos;

// These are constant throughout the program.
uniform double grid_size;
uniform dvec3 terrain_origin_m;
uniform double terrain_res_m;

// The min corner of the currently visible terrain
uniform dvec2 terrain_pos;
uniform ivec2 height_texture_offset;
uniform ivec2 color_texture_offset;
uniform dmat4 world_to_gl; // the full projective transform
uniform dmat4 terrain_to_world;
uniform sampler2D height;
uniform sampler2D color;

out VS_OUT {
    vec4 color;
    uint quads;
} vs_out;

void main() {
  // vec2 texCoord = height_texture_offset + vec2(aPos.xy) / float(grid_size+1.0);
  // vec4 tex = texture(height, texCoord);
  ivec2 texSize = textureSize(height, 0);
  ivec2 texCoord = height_texture_offset + aPos.xy;
  ivec2 texCoordModSize = texCoord - (texCoord / texSize) * texSize;
  vec4 tex = texelFetch(height, texCoordModSize, 0);
  vs_out.quads = uint(tex.y);
  dvec4 local_pos = dvec4(terrain_origin_m, 1.0lf);
  local_pos.xy += terrain_res_m * (dvec2(aPos.xy) + terrain_pos);
  local_pos.z += double(tex.x);
  vs_out.color = texelFetch(color, texCoordModSize, 0);
  vs_out.color.w = 1.0f;
  gl_Position = vec4(world_to_gl * terrain_to_world * local_pos);
}
