#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "test_world.h"

#define PI 3.14159265358979323846f

typedef ag_vec3_t vec3_t;

static vec3_t v3(float x, float y, float z) { return (vec3_t){x, y, z}; }
static vec3_t add(vec3_t a, vec3_t b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static vec3_t sub(vec3_t a, vec3_t b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static vec3_t mulf(vec3_t a, float f) { return v3(a.x * f, a.y * f, a.z * f); }
static float dot(vec3_t a, vec3_t b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static vec3_t cross(vec3_t a, vec3_t b) { return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
static float len(vec3_t a) { return sqrtf(dot(a, a)); }
static vec3_t norm(vec3_t a) { float l = len(a); return l > 1e-9f ? mulf(a, 1.0f / l) : a; }

void test_world_init(test_world_t *tw) {
	memset(tw, 0, sizeof(*tw));
}

void test_world_free(test_world_t *tw) {
	free(tw->tris);
	free(tw->cell_start);
	free(tw->cell_items);
	free(tw->route);
	memset(tw, 0, sizeof(*tw));
}

void test_world_add_tri(test_world_t *tw, vec3_t a, vec3_t b, vec3_t c, vec3_t up_hint, uint32_t flags) {
	if (tw->tris_len == tw->tris_cap) {
		tw->tris_cap = tw->tris_cap ? tw->tris_cap * 2 : 1024;
		tw->tris = realloc(tw->tris, sizeof(test_tri_t) * tw->tris_cap);
	}
	vec3_t n = norm(cross(sub(b, a), sub(c, a)));
	if (dot(n, up_hint) < 0) {
		n = mulf(n, -1);
	}
	tw->tris[tw->tris_len++] = (test_tri_t){a, b, c, n, flags};
}

void test_world_add_quad(test_world_t *tw, vec3_t a, vec3_t b, vec3_t c, vec3_t d, vec3_t up_hint, uint32_t flags) {
	test_world_add_tri(tw, a, b, c, up_hint, flags);
	test_world_add_tri(tw, a, c, d, up_hint, flags);
}

static void tri_cell_range(test_world_t *tw, test_tri_t *t, int *x0, int *x1, int *z0, int *z1) {
	float min_x = fminf(t->a.x, fminf(t->b.x, t->c.x));
	float max_x = fmaxf(t->a.x, fmaxf(t->b.x, t->c.x));
	float min_z = fminf(t->a.z, fminf(t->b.z, t->c.z));
	float max_z = fmaxf(t->a.z, fmaxf(t->b.z, t->c.z));
	*x0 = (int)((min_x - tw->min_x) / tw->cell_size);
	*x1 = (int)((max_x - tw->min_x) / tw->cell_size);
	*z0 = (int)((min_z - tw->min_z) / tw->cell_size);
	*z1 = (int)((max_z - tw->min_z) / tw->cell_size);
}

void test_world_build_grid(test_world_t *tw, float cell_size) {
	float min_x = INFINITY, min_z = INFINITY, max_x = -INFINITY, max_z = -INFINITY;
	for (int i = 0; i < tw->tris_len; i++) {
		vec3_t p[3] = {tw->tris[i].a, tw->tris[i].b, tw->tris[i].c};
		for (int k = 0; k < 3; k++) {
			min_x = fminf(min_x, p[k].x); max_x = fmaxf(max_x, p[k].x);
			min_z = fminf(min_z, p[k].z); max_z = fmaxf(max_z, p[k].z);
		}
	}
	tw->cell_size = cell_size;
	tw->min_x = min_x;
	tw->min_z = min_z;
	tw->cells_x = (int)((max_x - min_x) / cell_size) + 1;
	tw->cells_z = (int)((max_z - min_z) / cell_size) + 1;

	int cells = tw->cells_x * tw->cells_z;
	int *counts = calloc(cells + 1, sizeof(int));
	for (int i = 0; i < tw->tris_len; i++) {
		int x0, x1, z0, z1;
		tri_cell_range(tw, &tw->tris[i], &x0, &x1, &z0, &z1);
		for (int z = z0; z <= z1; z++) {
			for (int x = x0; x <= x1; x++) {
				counts[z * tw->cells_x + x]++;
			}
		}
	}

	tw->cell_start = calloc(cells + 1, sizeof(int));
	for (int c = 0; c < cells; c++) {
		tw->cell_start[c + 1] = tw->cell_start[c] + counts[c];
	}
	tw->cell_items = malloc(sizeof(int) * (tw->cell_start[cells] + 1));
	memset(counts, 0, sizeof(int) * cells);
	for (int i = 0; i < tw->tris_len; i++) {
		int x0, x1, z0, z1;
		tri_cell_range(tw, &tw->tris[i], &x0, &x1, &z0, &z1);
		for (int z = z0; z <= z1; z++) {
			for (int x = x0; x <= x1; x++) {
				int c = z * tw->cells_x + x;
				tw->cell_items[tw->cell_start[c] + counts[c]++] = i;
			}
		}
	}
	free(counts);
}

static bool ray_tri(vec3_t o, vec3_t d, test_tri_t *t, float *dist) {
	vec3_t e1 = sub(t->b, t->a);
	vec3_t e2 = sub(t->c, t->a);
	vec3_t p = cross(d, e2);
	float det = dot(e1, p);
	if (fabsf(det) < 1e-9f) {
		return false;
	}
	float inv = 1.0f / det;
	vec3_t s = sub(o, t->a);
	float u = dot(s, p) * inv;
	if (u < 0 || u > 1) {
		return false;
	}
	vec3_t q = cross(s, e1);
	float v = dot(d, q) * inv;
	if (v < 0 || u + v > 1) {
		return false;
	}
	*dist = dot(e2, q) * inv;
	return *dist >= 0;
}

bool test_world_raycast(void *user, vec3_t origin, vec3_t dir, float max_distance, ag_hit_t *hit) {
	test_world_t *tw = user;
	vec3_t end = add(origin, mulf(dir, max_distance));

	int x0 = (int)floorf((fminf(origin.x, end.x) - tw->min_x) / tw->cell_size);
	int x1 = (int)floorf((fmaxf(origin.x, end.x) - tw->min_x) / tw->cell_size);
	int z0 = (int)floorf((fminf(origin.z, end.z) - tw->min_z) / tw->cell_size);
	int z1 = (int)floorf((fmaxf(origin.z, end.z) - tw->min_z) / tw->cell_size);
	if (x0 < 0) x0 = 0;
	if (z0 < 0) z0 = 0;
	if (x1 >= tw->cells_x) x1 = tw->cells_x - 1;
	if (z1 >= tw->cells_z) z1 = tw->cells_z - 1;

	float best = max_distance;
	int best_tri = -1;
	for (int z = z0; z <= z1; z++) {
		for (int x = x0; x <= x1; x++) {
			int c = z * tw->cells_x + x;
			for (int k = tw->cell_start[c]; k < tw->cell_start[c + 1]; k++) {
				float dist;
				int i = tw->cell_items[k];
				if (ray_tri(origin, dir, &tw->tris[i], &dist) && dist < best) {
					best = dist;
					best_tri = i;
				}
			}
		}
	}
	if (best_tri < 0) {
		return false;
	}

	test_tri_t *t = &tw->tris[best_tri];
	hit->distance = best;
	hit->position = add(origin, mulf(dir, best));
	hit->normal = dot(t->normal, dir) > 0 ? mulf(t->normal, -1) : t->normal;
	hit->surface_flags = t->flags;
	hit->material = (uint32_t)best_tri;
	return true;
}


// Circuit ----------------------------------------------------------------------

#define CIRCUIT_SAMPLES 900
#define CIRCUIT_HALF_WIDTH 12.0f
#define CIRCUIT_WALL_HEIGHT 4.0f

static vec3_t circuit_center(float t) {
	float a = t * 2 * PI;

	// A chicane on the back straight
	float d = (a - 4.4f) / 0.22f;
	float chicane = 26 * expf(-d * d) * sinf(14 * a);
	return v3(
		450 * cosf(a) + 60 * cosf(3 * a) + chicane * cosf(a),
		22 * sinf(2 * a + 0.5f) + 5 * sinf(5 * a),
		450 * sinf(a) + 80 * sinf(2 * a) + chicane * sinf(a)
	);
}

static bool in_range(int i, int start, int count) {
	return i >= start && i < start + count;
}

void test_world_make_circuit(test_world_t *tw) {
	test_world_init(tw);
	const int n = CIRCUIT_SAMPLES;
	vec3_t *c = malloc(sizeof(vec3_t) * n);
	vec3_t *up = malloc(sizeof(vec3_t) * n);
	vec3_t *side = malloc(sizeof(vec3_t) * n);

	for (int i = 0; i < n; i++) {
		c[i] = circuit_center((float)i / n);
	}

	// Bank into the turns: tilt up towards the center of curvature
	for (int i = 0; i < n; i++) {
		vec3_t prev = c[(i + n - 1) % n], next = c[(i + 1) % n];
		vec3_t prev2 = c[(i + n - 2) % n], next2 = c[(i + 2) % n];
		vec3_t t = norm(sub(next, prev));
		vec3_t t_prev = norm(sub(c[i], prev2));
		vec3_t t_next = norm(sub(next2, c[i]));
		vec3_t curvature = mulf(sub(t_next, t_prev), 1.0f / len(sub(next, prev)));
		curvature.y = 0;
		vec3_t tilt = mulf(curvature, 25.0f);
		float tilt_len = len(tilt);
		if (tilt_len > 0.3f) {
			tilt = mulf(tilt, 0.3f / tilt_len);
		}
		vec3_t u = norm(add(v3(0, 1, 0), tilt));
		u = norm(sub(u, mulf(t, dot(u, t))));
		up[i] = u;
		side[i] = norm(cross(u, t));
	}

	const int gap_start = (int)(0.30f * n);
	const int gap_len = 6;
	const int ramp_len = 10;
	const int boost_a = (int)(0.06f * n);
	const int boost_b = (int)(0.56f * n);
	const int boost_len = 4;

	for (int i = 0; i < n; i++) {
		int j = (i + 1) % n;
		vec3_t li = add(c[i], mulf(side[i], CIRCUIT_HALF_WIDTH));
		vec3_t ri = sub(c[i], mulf(side[i], CIRCUIT_HALF_WIDTH));
		vec3_t lj = add(c[j], mulf(side[j], CIRCUIT_HALF_WIDTH));
		vec3_t rj = sub(c[j], mulf(side[j], CIRCUIT_HALF_WIDTH));

		bool gap = in_range(i, gap_start, gap_len);
		if (!gap) {
			uint32_t floor_flags = in_range(i, gap_start - ramp_len, ramp_len) ? AG_SURFACE_NO_MAGNET : 0;
			uint32_t left_flags = floor_flags;
			if (in_range(i, boost_a, boost_len) || in_range(i, boost_b, boost_len)) {
				left_flags |= AG_SURFACE_BOOST;
			}
			test_world_add_quad(tw, li, c[i], c[j], lj, up[i], left_flags);
			test_world_add_quad(tw, c[i], ri, rj, c[j], up[i], floor_flags);
		}

		vec3_t wall_i = mulf(up[i], CIRCUIT_WALL_HEIGHT);
		vec3_t wall_j = mulf(up[j], CIRCUIT_WALL_HEIGHT);
		test_world_add_quad(tw, li, lj, add(lj, wall_j), add(li, wall_i), mulf(side[i], -1), 0);
		test_world_add_quad(tw, ri, rj, add(rj, wall_j), add(ri, wall_i), side[i], 0);
	}

	// Kill pit under the jump
	vec3_t pit = c[gap_start + gap_len / 2];
	pit.y -= 30;
	test_world_add_quad(tw,
		add(pit, v3(-80, 0, -80)), add(pit, v3(80, 0, -80)),
		add(pit, v3(80, 0, 80)), add(pit, v3(-80, 0, 80)),
		v3(0, 1, 0), AG_SURFACE_KILL
	);

	// Route along the center line
	tw->route_len = n / 3;
	tw->route = malloc(sizeof(ag_route_point_t) * tw->route_len);
	for (int k = 0; k < tw->route_len; k++) {
		int i = k * 3;
		tw->route[k] = (ag_route_point_t){c[i], up[i], CIRCUIT_HALF_WIDTH * 2};
	}

	free(c);
	free(up);
	free(side);
	test_world_build_grid(tw, 16);
}

void test_world_make_flat(test_world_t *tw, bool with_wall, float wall_x) {
	test_world_init(tw);
	float s = 5000;
	test_world_add_quad(tw, v3(-s, 0, -s), v3(s, 0, -s), v3(s, 0, s), v3(-s, 0, s), v3(0, 1, 0), 0);
	if (with_wall) {
		test_world_add_quad(tw, v3(wall_x, 0, -s), v3(wall_x, 0, s), v3(wall_x, 6, s), v3(wall_x, 6, -s), v3(1, 0, 0), 0);
	}
	test_world_build_grid(tw, 250);
}
