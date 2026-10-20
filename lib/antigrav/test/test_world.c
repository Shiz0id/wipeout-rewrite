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
	ag_mesh_destroy(tw->mesh);
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

void test_world_build(test_world_t *tw) {
	float *positions = malloc(sizeof(float) * 9 * tw->tris_len);
	uint32_t *flags = malloc(sizeof(uint32_t) * tw->tris_len);
	uint32_t *materials = malloc(sizeof(uint32_t) * tw->tris_len);
	for (int i = 0; i < tw->tris_len; i++) {
		test_tri_t *t = &tw->tris[i];
		float *p = &positions[i * 9];
		p[0] = t->a.x; p[1] = t->a.y; p[2] = t->a.z;
		p[3] = t->b.x; p[4] = t->b.y; p[5] = t->b.z;
		p[6] = t->c.x; p[7] = t->c.y; p[8] = t->c.z;
		flags[i] = t->flags;
		materials[i] = (uint32_t)i;
	}
	ag_mesh_part_t part = {
		.positions = positions,
		.vertex_count = tw->tris_len * 3,
		.triangle_flags = flags,
		.triangle_materials = materials,
	};
	tw->mesh = ag_mesh_create(&part, 1);
	free(positions);
	free(flags);
	free(materials);
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
	return ag_mesh_raycast(tw->mesh, origin, dir, max_distance, hit);
}

bool test_world_raycast_brute(test_world_t *tw, vec3_t origin, vec3_t dir, float max_distance, ag_hit_t *hit) {
	float best = max_distance;
	int best_tri = -1;
	for (int i = 0; i < tw->tris_len; i++) {
		float dist;
		if (ray_tri(origin, dir, &tw->tris[i], &dist) && dist < best) {
			best = dist;
			best_tri = i;
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

void test_world_make_circuit(test_world_t *tw, bool with_walls) {
	test_world_init(tw);
	const int n = CIRCUIT_SAMPLES;
	vec3_t *c = malloc(sizeof(vec3_t) * n);
	vec3_t *up = malloc(sizeof(vec3_t) * n);
	vec3_t *side = malloc(sizeof(vec3_t) * n);

	const int gap_start = (int)(0.30f * n);
	const int gap_len = 6;
	const int ramp_len = 10;
	const int boost_a = (int)(0.06f * n);
	const int boost_b = (int)(0.56f * n);
	const int boost_len = 4;
	const int landing_len = 150;

	for (int i = 0; i < n; i++) {
		c[i] = circuit_center((float)i / n);

		// A kicker ramp before the jump and a landing 10m below the lip
		if (in_range(i, gap_start - ramp_len, ramp_len + gap_len)) {
			c[i].y += 4.0f * fminf((float)(i - (gap_start - ramp_len)) / ramp_len, 1);
		}
		else if (in_range(i, gap_start + gap_len, landing_len)) {
			c[i].y -= 10.0f * (1 - (float)(i - gap_start - gap_len) / landing_len);
		}
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
			// Boost pads cover only the outer half of the left lane
			vec3_t mi = add(mulf(li, 0.5f), mulf(c[i], 0.5f)), mj = add(mulf(lj, 0.5f), mulf(c[j], 0.5f));
			test_world_add_quad(tw, li, mi, mj, lj, up[i], left_flags);
			test_world_add_quad(tw, mi, c[i], c[j], mj, up[i], floor_flags);
			test_world_add_quad(tw, c[i], ri, rj, c[j], up[i], floor_flags);
		}

		if (with_walls) {
			vec3_t wall_i = mulf(up[i], CIRCUIT_WALL_HEIGHT);
			vec3_t wall_j = mulf(up[j], CIRCUIT_WALL_HEIGHT);
			test_world_add_quad(tw, li, lj, add(lj, wall_j), add(li, wall_i), mulf(side[i], -1), 0);
			test_world_add_quad(tw, ri, rj, add(rj, wall_j), add(ri, wall_i), side[i], 0);
		}
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
	test_world_build(tw);
}

void test_world_make_flat(test_world_t *tw, bool with_wall, float wall_x) {
	test_world_init(tw);
	float s = 5000;
	test_world_add_quad(tw, v3(-s, 0, -s), v3(s, 0, -s), v3(s, 0, s), v3(-s, 0, s), v3(0, 1, 0), 0);
	if (with_wall) {
		test_world_add_quad(tw, v3(wall_x, 0, -s), v3(wall_x, 0, s), v3(wall_x, 6, s), v3(wall_x, 6, -s), v3(1, 0, 0), 0);
	}
	test_world_build(tw);
}


// Drop -------------------------------------------------------------------------

typedef struct {
	vec3_t pos, up;
	uint32_t flags;
} strip_point_t;

void test_world_make_drop(test_world_t *tw, bool maglock) {
	test_world_init(tw);
	const float r = 25, half_width = 10;
	const uint32_t mag = maglock ? AG_SURFACE_MAGLOCK : 0;
	strip_point_t pts[1024];
	int n = 0;

	// Approach
	for (float z = 0; z < 200; z += 5) {
		pts[n++] = (strip_point_t){v3(0, 0, z), v3(0, 1, 0), 0};
	}
	// Convex edge into the drop
	for (int i = 0; i <= 40; i++) {
		float a = i / 40.0f * PI / 2;
		pts[n++] = (strip_point_t){v3(0, -r + r * cosf(a), 200 + r * sinf(a)), v3(0, cosf(a), sinf(a)), mag};
	}
	// Vertical drop
	for (float y = -r - 5; y > -r - 60; y -= 5) {
		pts[n++] = (strip_point_t){v3(0, y, 200 + r), v3(0, 0, 1), mag};
	}
	// Concave curve onto the floor below
	for (int i = 0; i <= 40; i++) {
		float b = i / 40.0f * PI / 2;
		pts[n++] = (strip_point_t){v3(0, -r - 60 - r * sinf(b), 200 + 2 * r - r * cosf(b)), v3(0, sinf(b), cosf(b)), mag};
	}
	// Run out
	for (float z = 200 + 2 * r + 5; z < 700; z += 5) {
		pts[n++] = (strip_point_t){v3(0, -2 * r - 60, z), v3(0, 1, 0), 0};
	}

	vec3_t x = v3(half_width, 0, 0);
	for (int i = 0; i < n - 1; i++) {
		strip_point_t *a = &pts[i], *b = &pts[i + 1];
		test_world_add_quad(tw, sub(a->pos, x), add(a->pos, x), add(b->pos, x), sub(b->pos, x), a->up, a->flags & b->flags);
	}

	tw->route_len = n;
	tw->route = malloc(sizeof(ag_route_point_t) * n);
	for (int i = 0; i < n; i++) {
		tw->route[i] = (ag_route_point_t){pts[i].pos, pts[i].up, half_width * 2};
	}
	test_world_build(tw);
}
