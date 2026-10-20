// Test geometry for antigrav: procedural triangle soups, collided through
// antigrav_mesh, standing in for the host engine's map and collision queries.

#ifndef TEST_WORLD_H
#define TEST_WORLD_H

#include "../antigrav.h"
#include "../antigrav_mesh.h"

typedef struct {
	ag_vec3_t a, b, c;
	ag_vec3_t normal;
	uint32_t flags;
} test_tri_t;

typedef struct {
	test_tri_t *tris;
	int tris_len;
	int tris_cap;

	ag_mesh_t *mesh;

	ag_route_point_t *route;
	int route_len;
} test_world_t;

void test_world_init(test_world_t *tw);
void test_world_free(test_world_t *tw);
void test_world_add_tri(test_world_t *tw, ag_vec3_t a, ag_vec3_t b, ag_vec3_t c, ag_vec3_t up_hint, uint32_t flags);
void test_world_add_quad(test_world_t *tw, ag_vec3_t a, ag_vec3_t b, ag_vec3_t c, ag_vec3_t d, ag_vec3_t up_hint, uint32_t flags);
void test_world_build(test_world_t *tw);

// ag_raycast_func_t; user is a test_world_t
bool test_world_raycast(void *user, ag_vec3_t origin, ag_vec3_t dir, float max_distance, ag_hit_t *hit);

// Reference raycast testing every triangle
bool test_world_raycast_brute(test_world_t *tw, ag_vec3_t origin, ag_vec3_t dir, float max_distance, ag_hit_t *hit);

// A ~3km closed circuit with hills, banked turns, a chicane, a jump over a
// kill pit and boost pads. Without walls it's an open strip, like a course
// laid out across open terrain. Fills tw->route.
void test_world_make_circuit(test_world_t *tw, bool with_walls);
void test_world_make_circuit_ex(test_world_t *tw, bool with_walls, float wall_height);

// A straight run over a 90 degree convex edge, down a 60m vertical drop and
// through a concave curve onto the floor below. The curved and vertical
// sections can be mag-strips. Runs along +z; fills tw->route.
void test_world_make_drop(test_world_t *tw, bool maglock);

// A large flat floor with an optional straight wall along z at x = wall_x
// (facing +x)
void test_world_make_flat(test_world_t *tw, bool with_wall, float wall_x);

#endif
