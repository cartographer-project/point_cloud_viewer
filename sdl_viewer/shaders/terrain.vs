#version 410 core
// The vertex positions are in grid coordinates, i.e. between 0 and grid_size (inclusive).
// The grid scale is one less than a power of two.
layout(location = 0) in ivec3 aPos;

// These are constant throughout the program.
uniform double grid_size;
uniform dvec3 terrain_origin_m;
uniform double terrain_res_m;

// The min corner of the currently visible terrain
uniform ivec2 terrain_pos;
uniform ivec2 texture_offset;
uniform dmat4 world_to_gl; // the full projective transform
uniform sampler2D heightmap_sampler;

out VS_OUT {
    vec4 color;
    uint quads;
} vs_out;

void main() {
  // vec2 texCoord = texture_offset + vec2(aPos.xy) / float(grid_size+1.0);
  // vec4 tex = texture(heightmap_sampler, texCoord);
  ivec2 texSize = textureSize(heightmap_sampler, 0);
  ivec2 texCoord = texture_offset + aPos.xy;
  ivec2 texCoordModSize = texCoord - (texCoord / texSize) * texSize;
  vec4 tex = texelFetch(heightmap_sampler, texCoordModSize, 0);
  vs_out.quads = uint(tex.y);
  dvec4 world_pos = dvec4(terrain_origin_m, 1.0lf);
  world_pos.xy += terrain_res_m * (dvec2(aPos.xy) + dvec2(terrain_pos));
  world_pos.z += double(tex.x);
  vs_out.color = vec4(0.2, 0.0, 1.0, 1.0);
  gl_Position = vec4(world_to_gl * world_pos);
}
