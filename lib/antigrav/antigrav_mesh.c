#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "antigrav_mesh.h"

#define LEAF_MAX 4
#define SAH_BINS 12
#define STACK_MAX 64

typedef ag_vec3_t vec3_t;

typedef struct {
	float min[3], max[3];
} aabb_t;

typedef struct {
	aabb_t box;
	uint32_t first;   // Leaf: first triangle; inner: index of the left child (right = +1)
	uint32_t count;   // Triangles in a leaf; 0 for inner nodes
} node_t;

typedef struct {
	vec3_t v0, e1, e2;
	uint32_t flags;
	uint32_t material;
} tri_t;

struct ag_mesh_t {
	tri_t *tris;
	int tris_len;
	node_t *nodes;
	int nodes_len;
};

static inline vec3_t sub(vec3_t a, vec3_t b) { return (vec3_t){a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline float dot(vec3_t a, vec3_t b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline vec3_t cross(vec3_t a, vec3_t b) {
	return (vec3_t){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
static inline float comp(vec3_t v, int axis) { return axis == 0 ? v.x : (axis == 1 ? v.y : v.z); }


// Build ------------------------------------------------------------------------

static void aabb_reset(aabb_t *b) {
	b->min[0] = b->min[1] = b->min[2] = FLT_MAX;
	b->max[0] = b->max[1] = b->max[2] = -FLT_MAX;
}

static void aabb_add(aabb_t *b, vec3_t p) {
	float c[3] = {p.x, p.y, p.z};
	for (int i = 0; i < 3; i++) {
		if (c[i] < b->min[i]) b->min[i] = c[i];
		if (c[i] > b->max[i]) b->max[i] = c[i];
	}
}

static void aabb_merge(aabb_t *b, const aabb_t *o) {
	for (int i = 0; i < 3; i++) {
		if (o->min[i] < b->min[i]) b->min[i] = o->min[i];
		if (o->max[i] > b->max[i]) b->max[i] = o->max[i];
	}
}

static float aabb_area(const aabb_t *b) {
	float dx = b->max[0] - b->min[0], dy = b->max[1] - b->min[1], dz = b->max[2] - b->min[2];
	if (dx < 0 || dy < 0 || dz < 0) {
		return 0;
	}
	return dx * dy + dy * dz + dz * dx;
}

typedef struct {
	ag_mesh_t *mesh;
	aabb_t *tri_boxes;
	vec3_t *centroids;
	uint32_t *order;
} build_t;

static void build_node(build_t *b, uint32_t node_index, uint32_t first, uint32_t count, int depth) {
	node_t *node = &b->mesh->nodes[node_index];
	aabb_reset(&node->box);
	aabb_t centroid_box;
	aabb_reset(&centroid_box);
	for (uint32_t i = first; i < first + count; i++) {
		aabb_merge(&node->box, &b->tri_boxes[b->order[i]]);
		aabb_add(&centroid_box, b->centroids[b->order[i]]);
	}

	node->first = first;
	node->count = count;
	if (count <= LEAF_MAX || depth >= STACK_MAX - 2) {
		return;
	}

	// Split along the axis with the largest centroid spread
	int axis = 0;
	float extent = centroid_box.max[0] - centroid_box.min[0];
	for (int a = 1; a < 3; a++) {
		float e = centroid_box.max[a] - centroid_box.min[a];
		if (e > extent) {
			extent = e;
			axis = a;
		}
	}
	if (extent <= 0) {
		return; // All centroids coincide
	}

	// Binned SAH
	struct { aabb_t box; uint32_t count; } bins[SAH_BINS];
	for (int i = 0; i < SAH_BINS; i++) {
		aabb_reset(&bins[i].box);
		bins[i].count = 0;
	}
	float bin_scale = SAH_BINS / extent * 0.9999f;
	float base = centroid_box.min[axis];
	for (uint32_t i = first; i < first + count; i++) {
		uint32_t t = b->order[i];
		int bin = (int)((comp(b->centroids[t], axis) - base) * bin_scale);
		bins[bin].count++;
		aabb_merge(&bins[bin].box, &b->tri_boxes[t]);
	}

	float left_area[SAH_BINS - 1], right_area[SAH_BINS - 1];
	uint32_t left_count[SAH_BINS - 1], right_count[SAH_BINS - 1];
	aabb_t acc;
	uint32_t acc_count = 0;
	aabb_reset(&acc);
	for (int i = 0; i < SAH_BINS - 1; i++) {
		aabb_merge(&acc, &bins[i].box);
		acc_count += bins[i].count;
		left_area[i] = aabb_area(&acc);
		left_count[i] = acc_count;
	}
	aabb_reset(&acc);
	acc_count = 0;
	for (int i = SAH_BINS - 1; i > 0; i--) {
		aabb_merge(&acc, &bins[i].box);
		acc_count += bins[i].count;
		right_area[i - 1] = aabb_area(&acc);
		right_count[i - 1] = acc_count;
	}

	int best_split = -1;
	float best_cost = FLT_MAX;
	for (int i = 0; i < SAH_BINS - 1; i++) {
		if (left_count[i] == 0 || right_count[i] == 0) {
			continue;
		}
		float cost = left_area[i] * left_count[i] + right_area[i] * right_count[i];
		if (cost < best_cost) {
			best_cost = cost;
			best_split = i;
		}
	}

	uint32_t mid;
	if (best_split < 0) {
		mid = first + count / 2;
	}
	else {
		// Partition in place
		uint32_t lo = first, hi = first + count - 1;
		while (lo <= hi) {
			int bin = (int)((comp(b->centroids[b->order[lo]], axis) - base) * bin_scale);
			if (bin <= best_split) {
				lo++;
			}
			else {
				uint32_t tmp = b->order[lo];
				b->order[lo] = b->order[hi];
				b->order[hi] = tmp;
				if (hi == 0) break;
				hi--;
			}
		}
		mid = lo;
		if (mid == first || mid == first + count) {
			mid = first + count / 2;
		}
	}

	uint32_t left = (uint32_t)b->mesh->nodes_len;
	b->mesh->nodes_len += 2;
	node = &b->mesh->nodes[node_index];
	node->first = left;
	node->count = 0;
	build_node(b, left, first, mid - first, depth + 1);
	build_node(b, left + 1, mid, first + count - mid, depth + 1);
}

static vec3_t transform_point(const float *m, bool row_major, vec3_t p) {
	if (!m) {
		return p;
	}
	if (row_major) {
		return (vec3_t){
			m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3],
			m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
			m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11],
		};
	}
	return (vec3_t){
		m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12],
		m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13],
		m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14],
	};
}

static vec3_t part_vertex(const ag_mesh_part_t *part, uint32_t i) {
	int stride = part->position_stride ? part->position_stride : (int)(sizeof(float) * 3);
	const float *v = (const float *)((const uint8_t *)part->positions + (size_t)stride * i);
	return transform_point(part->transform, part->row_major, (vec3_t){v[0], v[1], v[2]});
}

ag_mesh_t *ag_mesh_create(const ag_mesh_part_t *parts, int parts_len) {
	size_t total = 0;
	for (int p = 0; p < parts_len; p++) {
		total += parts[p].indices ? (size_t)parts[p].triangle_count : (size_t)parts[p].vertex_count / 3;
	}
	if (total == 0 || total > 0x3fffffff) {
		return NULL;
	}

	ag_mesh_t *mesh = calloc(1, sizeof(ag_mesh_t));
	tri_t *tris = malloc(sizeof(tri_t) * total);
	build_t b = {
		.mesh = mesh,
		.tri_boxes = malloc(sizeof(aabb_t) * total),
		.centroids = malloc(sizeof(vec3_t) * total),
		.order = malloc(sizeof(uint32_t) * total),
	};
	if (!mesh || !tris || !b.tri_boxes || !b.centroids || !b.order) {
		goto fail;
	}

	// Gather triangles, dropping degenerate ones
	int len = 0;
	for (int p = 0; p < parts_len; p++) {
		const ag_mesh_part_t *part = &parts[p];
		int count = part->indices ? part->triangle_count : part->vertex_count / 3;
		for (int t = 0; t < count; t++) {
			uint32_t i0 = part->indices ? part->indices[t * 3 + 0] : (uint32_t)t * 3 + 0;
			uint32_t i1 = part->indices ? part->indices[t * 3 + 1] : (uint32_t)t * 3 + 1;
			uint32_t i2 = part->indices ? part->indices[t * 3 + 2] : (uint32_t)t * 3 + 2;
			if (i0 >= (uint32_t)part->vertex_count || i1 >= (uint32_t)part->vertex_count || i2 >= (uint32_t)part->vertex_count) {
				continue;
			}
			vec3_t a = part_vertex(part, i0), c1 = part_vertex(part, i1), c2 = part_vertex(part, i2);
			vec3_t e1 = sub(c1, a), e2 = sub(c2, a);
			vec3_t n = cross(e1, e2);
			if (dot(n, n) < 1e-12f) {
				continue;
			}
			tris[len] = (tri_t){
				.v0 = a, .e1 = e1, .e2 = e2,
				.flags = part->surface_flags | (part->triangle_flags ? part->triangle_flags[t] : 0),
				.material = part->triangle_materials ? part->triangle_materials[t] : part->material,
			};
			aabb_reset(&b.tri_boxes[len]);
			aabb_add(&b.tri_boxes[len], a);
			aabb_add(&b.tri_boxes[len], c1);
			aabb_add(&b.tri_boxes[len], c2);
			b.centroids[len] = (vec3_t){(a.x + c1.x + c2.x) / 3, (a.y + c1.y + c2.y) / 3, (a.z + c1.z + c2.z) / 3};
			b.order[len] = (uint32_t)len;
			len++;
		}
	}
	if (len == 0) {
		goto fail;
	}

	mesh->nodes = malloc(sizeof(node_t) * (size_t)len * 2);
	if (!mesh->nodes) {
		goto fail;
	}
	mesh->nodes_len = 1;
	build_node(&b, 0, 0, (uint32_t)len, 0);

	// Store triangles in leaf order
	mesh->tris = malloc(sizeof(tri_t) * len);
	if (!mesh->tris) {
		goto fail;
	}
	for (int i = 0; i < len; i++) {
		mesh->tris[i] = tris[b.order[i]];
	}
	mesh->tris_len = len;
	mesh->nodes = realloc(mesh->nodes, sizeof(node_t) * mesh->nodes_len);

	free(tris);
	free(b.tri_boxes);
	free(b.centroids);
	free(b.order);
	return mesh;

fail:
	free(tris);
	free(b.tri_boxes);
	free(b.centroids);
	free(b.order);
	ag_mesh_destroy(mesh);
	return NULL;
}

void ag_mesh_destroy(ag_mesh_t *mesh) {
	if (!mesh) {
		return;
	}
	free(mesh->tris);
	free(mesh->nodes);
	free(mesh);
}

int ag_mesh_triangle_count(const ag_mesh_t *mesh) {
	return mesh ? mesh->tris_len : 0;
}


// Raycast ----------------------------------------------------------------------

// Entry distance into the box, or INFINITY if missed within max
static inline float ray_box(const aabb_t *b, const float o[3], const float inv[3], float max) {
	float t0 = 0, t1 = max;
	for (int i = 0; i < 3; i++) {
		float near = (b->min[i] - o[i]) * inv[i];
		float far = (b->max[i] - o[i]) * inv[i];
		if (near > far) {
			float tmp = near; near = far; far = tmp;
		}
		if (near > t0) t0 = near;
		if (far < t1) t1 = far;
		if (t0 > t1) {
			return INFINITY;
		}
	}
	return t0;
}

bool ag_mesh_raycast(void *user, vec3_t origin, vec3_t dir, float max_distance, ag_hit_t *hit) {
	ag_mesh_t *mesh = user;
	if (!mesh || mesh->tris_len == 0) {
		return false;
	}

	float o[3] = {origin.x, origin.y, origin.z};
	float d[3] = {dir.x, dir.y, dir.z};
	float inv[3];
	for (int i = 0; i < 3; i++) {
		inv[i] = fabsf(d[i]) > 1e-12f ? 1.0f / d[i] : (d[i] >= 0 ? 1e30f : -1e30f);
	}

	float best = max_distance;
	int best_tri = -1;
	uint32_t stack[STACK_MAX];
	int sp = 0;
	stack[sp++] = 0;

	while (sp > 0) {
		node_t *node = &mesh->nodes[stack[--sp]];
		if (ray_box(&node->box, o, inv, best) == INFINITY) {
			continue;
		}

		if (node->count) {
			for (uint32_t i = node->first; i < node->first + node->count; i++) {
				tri_t *t = &mesh->tris[i];
				vec3_t p = cross(dir, t->e2);
				float det = dot(t->e1, p);
				if (fabsf(det) < 1e-12f) {
					continue;
				}
				float inv_det = 1.0f / det;
				vec3_t s = sub(origin, t->v0);
				float u = dot(s, p) * inv_det;
				if (u < 0 || u > 1) {
					continue;
				}
				vec3_t q = cross(s, t->e1);
				float v = dot(dir, q) * inv_det;
				if (v < 0 || u + v > 1) {
					continue;
				}
				float dist = dot(t->e2, q) * inv_det;
				if (dist >= 0 && dist < best) {
					best = dist;
					best_tri = (int)i;
				}
			}
			continue;
		}

		// Visit the nearer child first
		uint32_t l = node->first, r = node->first + 1;
		float tl = ray_box(&mesh->nodes[l].box, o, inv, best);
		float tr = ray_box(&mesh->nodes[r].box, o, inv, best);
		if (tl > tr) {
			uint32_t ti = l; l = r; r = ti;
			float tf = tl; tl = tr; tr = tf;
		}
		if (tr != INFINITY && sp < STACK_MAX) stack[sp++] = r;
		if (tl != INFINITY && sp < STACK_MAX) stack[sp++] = l;
	}

	if (best_tri < 0) {
		return false;
	}

	tri_t *t = &mesh->tris[best_tri];
	vec3_t n = cross(t->e1, t->e2);
	float n_len = sqrtf(dot(n, n));
	n = (vec3_t){n.x / n_len, n.y / n_len, n.z / n_len};
	if (dot(n, dir) > 0) {
		n = (vec3_t){-n.x, -n.y, -n.z};
	}
	hit->position = (vec3_t){origin.x + dir.x * best, origin.y + dir.y * best, origin.z + dir.z * best};
	hit->normal = n;
	hit->distance = best;
	hit->surface_flags = t->flags;
	hit->material = t->material;
	return true;
}
