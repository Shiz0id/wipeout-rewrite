/*
	antigrav_mesh - optional static triangle mesh collision for antigrav

	A BVH over triangle soups with a raycast that plugs straight into
	ag_world_desc_t.raycast. Use it when the host engine has no convenient
	raycast of its own, or to feed antigrav a dedicated, simplified collision
	mesh of the race course.

		ag_mesh_part_t part = {
			.positions = vertices, .vertex_count = n,
			.indices = indices, .triangle_count = tri_count,
		};
		ag_mesh_t *mesh = ag_mesh_create(&part, 1);
		desc.raycast = ag_mesh_raycast;
		desc.user = mesh;

	Memory: ~80 bytes per triangle. Build time is O(n log n); a million
	triangles take on the order of a second.
*/

#ifndef ANTIGRAV_MESH_H
#define ANTIGRAV_MESH_H

#include "antigrav.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	const float *positions;           // x, y, z per vertex
	int position_stride;              // Bytes between vertices; 0 = tightly packed (12)
	int vertex_count;

	const uint32_t *indices;          // 3 per triangle; NULL = non-indexed (vertex_count / 3 triangles)
	int triangle_count;               // Ignored when indices is NULL

	// Optional 4x4 transform applied to positions. Column-major with the
	// translation in elements 12, 13, 14 (OpenGL style) unless row_major is
	// set, in which case the translation is in elements 3, 7, 11.
	const float *transform;
	bool row_major;

	uint32_t surface_flags;           // AG_SURFACE_* for all triangles of this part
	uint32_t material;                // Reported in ag_hit_t for all triangles of this part
	const uint32_t *triangle_flags;   // Optional per triangle flags, OR'ed with surface_flags
	const uint32_t *triangle_materials; // Optional per triangle material, overrides material
} ag_mesh_part_t;

typedef struct ag_mesh_t ag_mesh_t;

// Build a mesh from one or more parts. Degenerate triangles are dropped.
// Returns NULL on allocation failure or if there are no triangles.
ag_mesh_t *ag_mesh_create(const ag_mesh_part_t *parts, int parts_len);
void ag_mesh_destroy(ag_mesh_t *mesh);

int ag_mesh_triangle_count(const ag_mesh_t *mesh);

// Matches ag_raycast_func_t; pass the mesh as user. Triangles are double
// sided; the hit normal faces the ray origin.
bool ag_mesh_raycast(void *mesh, ag_vec3_t origin, ag_vec3_t dir, float max_distance, ag_hit_t *hit);

#ifdef __cplusplus
}
#endif

#endif
