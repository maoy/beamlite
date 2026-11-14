// BeamLite — a tiny soft-body driving sandbox in the spirit of BeamNG.drive.
// Rendering/input via raylib; physics in physics.c.

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"
#include "physics.h"
#include "render.h"
#include "traffic.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

static Vector3 RV(V3 a) { return (Vector3){a.x, a.y, a.z}; }
static V3 SUN, AMB, SUNC;   // baked lighting: sun direction, ambient and sun multipliers

// Lighting presets (L cycles). Sky/fog colours are linear; bake* are vertex-light multipliers.
static const Env ENVS[] = {
    {"DUSK", {-0.62f, 0.05f, 0.78f}, {1.5f, 0.66f, 0.28f}, {0}, {0.016f, 0.032f, 0.095f}, {0.62f, 0.30f, 0.14f}, {0.075f, 0.11f, 0.22f},
     {0.045f, 0.06f, 0.11f}, 0.0040f, 0.045f, 1.05f, 1.0f, 1.0f, {0.27f, 0.31f, 0.43f}, {0.50f, 0.31f, 0.18f}},
    {"DAY", {0.45f, 0.85f, 0.30f}, {1.5f, 1.4f, 1.25f}, {0}, {0.10f, 0.26f, 0.62f}, {0.66f, 0.74f, 0.84f}, {0.52f, 0.64f, 0.80f},
     {0.58f, 0.68f, 0.82f}, 0.0024f, 0.030f, 1.0f, 0.5f, 0.0f, {0.44f, 0.45f, 0.48f}, {0.64f, 0.62f, 0.58f}},
};
static int envIdx = 0;
static const Env *ENV = &ENVS[0];

static void set_env(int i) {
    envIdx = i; ENV = &ENVS[i];
    SUN = vnorm(v3(ENV->sunDir.x, ENV->sunDir.y, ENV->sunDir.z));
    AMB = v3(ENV->bakeAmbient.x, ENV->bakeAmbient.y, ENV->bakeAmbient.z);
    SUNC = v3(ENV->bakeSun.x, ENV->bakeSun.y, ENV->bakeSun.z);
}

// Snow lying on the world (0..1). Baked into terrain, buildings and trees, so changing it rebuilds them.
static float snowCover;
static Color snow_over(Color c, float k) {          // blend toward fresh snow by k
    if (k <= 0) return c;
    const float sr = 228, sg = 233, sb = 242;
    return (Color){(unsigned char)(c.r + (sr - c.r) * k), (unsigned char)(c.g + (sg - c.g) * k), (unsigned char)(c.b + (sb - c.b) * k), c.a};
}
static float smoothk(float e0, float e1, float x) { float t = fminf(fmaxf((x - e0) / (e1 - e0), 0.0f), 1.0f); return t * t * (3 - 2 * t); }

// Bake ambient + sun light into a colour for a face with normal n.
static Color lit(Color c, V3 n, float k) {
    float d = fmaxf(vdot(n, SUN), 0.0f);
    float sky = 0.10f * fmaxf(n.y, 0.0f);
    float r = (AMB.x + sky + SUNC.x * d) * k, g = (AMB.y + sky + SUNC.y * d) * k, b = (AMB.z + sky * 1.2f + SUNC.z * d) * k;
    return (Color){(unsigned char)fminf(255, c.r * r), (unsigned char)fminf(255, c.g * g), (unsigned char)fminf(255, c.b * b), c.a};
}

// ------------------------------------------------------------- terrain ----

#define CH_N 6          // chunks per side
#define CH_CELLS 50     // cells per chunk side
#define CELL 2.0f       // metres per cell  -> 600 m square world

static Model chunks[CH_N * CH_N];

static float lerpf(float a, float b, float t) { return a + (b - a) * t; }

static Color terrain_color(float x, float z, float y, V3 n) {
    float r, g, b, cover = 1.0f;
    float noise = 0.5f + 0.25f * sinf(x * 0.37f + 1.7f) * sinf(z * 0.41f) + 0.25f * sinf(x * 0.13f - z * 0.17f);
    if (x > 40 && x < 58 && fabsf(z) < 6.5f) { r = 0.72f; g = 0.66f; b = 0.55f; cover = 0.7f; }  // ramp
    else if (dirt_at(x, z)) { r = 0.55f; g = 0.42f; b = 0.28f; cover = 0.8f; }                  // dirt
    else if (road_at(x, z) == 3) { r = 0.50f + 0.03f * noise; g = 0.47f; b = 0.44f; cover = 0.7f; }   // plaza paving
    else if (road_at(x, z) == 4) { r = g = b = 0.24f + 0.03f * noise; cover = 0.25f; }                 // runway / taxiway asphalt
    else if (road_at(x, z) == 5) { r = 0.60f + 0.03f * noise; g = 0.60f; b = 0.58f; cover = 0.5f; }    // apron concrete
    else if (road_at(x, z) == 1) { r = g = b = 0.30f + 0.04f * noise; cover = 0.18f + 0.22f * noise; }   // highway (ploughed)
    else if (road_at(x, z) == 2) { r = g = b = 0.27f + 0.03f * noise; cover = 0.25f + 0.25f * noise; }   // town street
    else if (road_at(x + 2.2f, z) == 2 || road_at(x - 2.2f, z) == 2 || road_at(x, z + 2.2f) == 2 || road_at(x, z - 2.2f) == 2) {
        r = 0.50f; g = 0.49f; b = 0.47f; cover = 0.85f;                                          // pavement
    }
    else if (map_has_broadway() && fabsf(x) < CITY_X + 4 && z > CITY_Z0 + 1 && z < CITY_Z1 + 4) {
        r = 0.46f + 0.03f * noise; g = 0.45f + 0.03f * noise; b = 0.43f; cover = 0.85f;           // Midtown: paved kerb to kerb
    }
    else if (sea_dist(x, z) > -7.0f) {                                                           // beach, then seabed
        float wet = fminf(fmaxf((AP_SEA_Y + 0.5f - y) / 2.5f, 0), 1);
        r = 0.74f - 0.44f * wet + 0.04f * noise; g = 0.69f - 0.36f * wet + 0.04f * noise; b = 0.53f - 0.22f * wet; cover = 0.5f;
    }
    else {                                                                                       // grass / hills
        r = 0.26f + 0.10f * noise; g = 0.47f + 0.12f * noise; b = 0.20f;
        float rock = fminf(fmaxf((1.0f - n.y) * 4.0f, 0), 1);
        r += (0.50f - r) * rock; g += (0.47f - g) * rock; b += (0.42f - b) * rock;
        cover = 1.0f - 0.8f * rock;                                                              // steep rock sheds snow
        (void)y;
    }
    int gx = (int)lroundf(x / CELL), gz = (int)lroundf(z / CELL);
    if ((gx % 5 == 0 || gz % 5 == 0) && fabsf(x) < 1500 && fabsf(z) < 320) { r *= 0.93f; g *= 0.93f; b *= 0.93f; }  // 10 m grid for speed sense
    Color base = {(unsigned char)fminf(255, r * 255), (unsigned char)fminf(255, g * 255), (unsigned char)fminf(255, b * 255), 255};
    base = snow_over(base, snowCover * cover * (0.92f + 0.08f * noise));
    return lit(base, n, 1.0f);
}

// The airport map reaches far beyond the 600 m detailed square: fine chunks carry on along the
// runway causeway, and one coarse sheet covers everything else out to AP_FAR.
#define EXT_X0 300.0f
#define EXT_X1 1400.0f
#define EXT_Z0 100.0f
#define EXT_Z1 300.0f
#define FAR_CELL 50.0f
static int fine_terrain(float x, float z, float m) {      // inside the finely meshed area (shrunk by m)?
    const float half = CH_N * CH_CELLS * CELL * 0.5f;
    if (fabsf(x) <= half - m && fabsf(z) <= half - m) return 1;
    return x >= EXT_X0 - 1 && x <= EXT_X1 - m && z >= EXT_Z0 + m && z <= EXT_Z1 - m;
}

static Model build_terrain(float x0, float z0, int cells, float cell, int coarse) {
    const int V = cells + 1;
    Mesh m = {0};
    m.vertexCount = V * V;
    m.triangleCount = cells * cells * 2;
    m.vertices = MemAlloc(m.vertexCount * 3 * sizeof(float));
    m.normals = MemAlloc(m.vertexCount * 3 * sizeof(float));
    m.colors = MemAlloc(m.vertexCount * 4);
    m.indices = MemAlloc(m.triangleCount * 3 * sizeof(unsigned short));
    for (int j = 0; j < V; j++)
    for (int i = 0; i < V; i++) {
        int v = j * V + i;
        float x = x0 + i * cell, z = z0 + j * cell, y = terrain_height(x, z);
        V3 n = terrain_normal(x, z);
        Color c = terrain_color(x, z, y, n);
        // The coarse sheet ducks under the fine mesh, and sits a touch low along the seam.
        if (coarse) y -= fine_terrain(x, z, 1.0f) ? 6.0f : fine_terrain(x, z, -1.0f) ? 0.4f : 0.0f;
        m.vertices[v * 3] = x; m.vertices[v * 3 + 1] = y; m.vertices[v * 3 + 2] = z;
        m.normals[v * 3] = n.x; m.normals[v * 3 + 1] = n.y; m.normals[v * 3 + 2] = n.z;
        m.colors[v * 4] = c.r; m.colors[v * 4 + 1] = c.g; m.colors[v * 4 + 2] = c.b; m.colors[v * 4 + 3] = 255;
    }
    int t = 0;
    for (int j = 0; j < cells; j++)
    for (int i = 0; i < cells; i++) {
        unsigned short a = j * V + i, b = a + 1, c = a + V, d = c + 1;
        m.indices[t++] = a; m.indices[t++] = c; m.indices[t++] = b;   // CCW seen from above
        m.indices[t++] = b; m.indices[t++] = c; m.indices[t++] = d;
    }
    UploadMesh(&m, false);
    return LoadModelFromMesh(m);
}
static Model build_chunk(int cx, int cz) {
    const float half = CH_N * CH_CELLS * CELL * 0.5f;
    return build_terrain(-half + cx * CH_CELLS * CELL, -half + cz * CH_CELLS * CELL, CH_CELLS, CELL, 0);
}
#define EXT_MAX 24
static Model extChunk[EXT_MAX], farTerrain;
static int nExt, hasFar;

// ------------------------------------------------- static mesh builder ----
// Flat-shaded, vertex-coloured triangles with lighting baked in (like the terrain).

typedef struct { float *v; unsigned char *c; int n, cap; } MeshB;
static int mbFlat;                 // when set, mb_tri skips lighting (glowing surfaces)

static void mb_push(MeshB *m, V3 p, Color c) {
    if (m->n == m->cap) {
        m->cap = m->cap ? m->cap * 2 : 4096;
        m->v = MemRealloc(m->v, m->cap * 3 * sizeof(float));
        m->c = MemRealloc(m->c, m->cap * 4);
    }
    m->v[m->n * 3] = p.x; m->v[m->n * 3 + 1] = p.y; m->v[m->n * 3 + 2] = p.z;
    m->c[m->n * 4] = c.r; m->c[m->n * 4 + 1] = c.g; m->c[m->n * 4 + 2] = c.b; m->c[m->n * 4 + 3] = 255;
    m->n++;
}

// Triangle wound and lit so its normal faces away from `ref`.
static void mb_tri(MeshB *m, V3 a, V3 b, V3 c, Color col, V3 ref) {
    V3 n = vnorm(vcross(vsub(b, a), vsub(c, a)));
    V3 ctr = vscale(vadd(vadd(a, b), c), 1.0f / 3.0f);
    if (vdot(n, vsub(ctr, ref)) < 0) { V3 t = b; b = c; c = t; n = vscale(n, -1); }
    if (!mbFlat && snowCover > 0) col = snow_over(col, snowCover * 0.9f * smoothk(0.40f, 0.80f, n.y));
    Color s = mbFlat ? col : lit(col, n, 1.08f);
    s.a = 255;
    mb_push(m, a, s); mb_push(m, b, s); mb_push(m, c, s);
}
// Unlit triangle with per-vertex colours (glows and light pools).
static void mb_tri3(MeshB *m, V3 a, V3 b, V3 c, Color ca, Color cb, Color cc) { mb_push(m, a, ca); mb_push(m, b, cb); mb_push(m, c, cc); }
static void mb_quad(MeshB *m, V3 a, V3 b, V3 c, V3 d, Color col, V3 ref) {
    mb_tri(m, a, b, c, col, ref); mb_tri(m, a, c, d, col, ref);
}

static Model mb_model(MeshB *m) {
    Mesh mesh = {0};
    mesh.vertexCount = m->n;
    mesh.triangleCount = m->n / 3;
    mesh.vertices = MemAlloc(m->n * 3 * sizeof(float));
    mesh.colors = MemAlloc(m->n * 4);
    memcpy(mesh.vertices, m->v, m->n * 3 * sizeof(float));
    memcpy(mesh.colors, m->c, m->n * 4);
    UploadMesh(&mesh, false);
    MemFree(m->v); MemFree(m->c);
    memset(m, 0, sizeof *m);
    return LoadModelFromMesh(mesh);
}

// ------------------------------------------------------------ buildings ----

static unsigned bseed;
static float brnd(void) { bseed = bseed * 1664525u + 1013904223u; return (bseed >> 8) / 16777216.0f; }

static const Color WALLS_HOUSE[] = {{226, 214, 190}, {200, 170, 140}, {236, 230, 220}, {180, 196, 206}, {214, 190, 160}, {172, 120, 96}};
static const Color WALLS_CITY[] = {{190, 188, 182}, {168, 160, 150}, {150, 156, 166}, {206, 198, 184}, {140, 120, 104}, {120, 128, 136}};
static const Color ROOFS[] = {{120, 56, 44}, {82, 82, 88}, {100, 74, 60}, {64, 70, 80}};
static const Color SIGNS[] = {{200, 50, 40}, {40, 110, 190}, {230, 170, 30}, {40, 150, 90}, {150, 60, 160}};

// Facade frame for side s (0 +x, 1 -x, 2 +z, 3 -z): origin corner, unit along-vector, outward normal, length.
static void facade(const Building *B, int s, V3 *o, V3 *a, V3 *n, float *len) {
    switch (s) {
        case 0: *o = v3(B->mx.x, 0, B->mn.z); *a = v3(0, 0, 1); *n = v3(1, 0, 0); *len = B->mx.z - B->mn.z; break;
        case 1: *o = v3(B->mn.x, 0, B->mx.z); *a = v3(0, 0, -1); *n = v3(-1, 0, 0); *len = B->mx.z - B->mn.z; break;
        case 2: *o = v3(B->mx.x, 0, B->mx.z); *a = v3(-1, 0, 0); *n = v3(0, 0, 1); *len = B->mx.x - B->mn.x; break;
        default: *o = v3(B->mn.x, 0, B->mn.z); *a = v3(1, 0, 0); *n = v3(0, 0, -1); *len = B->mx.x - B->mn.x; break;
    }
}

// A rectangle on a facade, `u` along it and `y` up from street level, standing proud by `out`.
static void facade_rect(MeshB *m, const Building *B, int s, float u, float y, float w, float h, float out, Color col) {
    V3 o, a, n; float len;
    facade(B, s, &o, &a, &n, &len);
    V3 base = vadd(vadd(o, vscale(n, out)), v3(0, B->ground, 0));
    V3 p0 = vadd(base, vadd(vscale(a, u), v3(0, y, 0)));
    V3 p1 = vadd(p0, vscale(a, w)), p2 = vadd(p1, v3(0, h, 0)), p3 = vadd(p0, v3(0, h, 0));
    mb_quad(m, p0, p1, p2, p3, col, vsub(p0, n));
}

static void box_shell(MeshB *m, V3 mn, V3 mx, Color wall, Color top) {
    V3 ref = vscale(vadd(mn, mx), 0.5f);
    V3 c[8];
    for (int i = 0; i < 8; i++) c[i] = v3(i & 1 ? mx.x : mn.x, i & 2 ? mx.y : mn.y, i & 4 ? mx.z : mn.z);
    mb_quad(m, c[1], c[3], c[7], c[5], wall, ref);   // +x
    mb_quad(m, c[0], c[4], c[6], c[2], wall, ref);   // -x
    mb_quad(m, c[4], c[5], c[7], c[6], wall, ref);   // +z
    mb_quad(m, c[0], c[2], c[3], c[1], wall, ref);   // -z
    mb_quad(m, c[2], c[6], c[7], c[3], top, ref);    // top
}

// ---------------------------------------------------------------- lights ----
// Glowing surfaces (lit windows, lamp heads) go in GLOW; additive light pools on walls and
// the ground go in POOL. Both are drawn only when the environment has its lights on.

static MeshB GLOW, POOL;
static const Color WARM = {255, 196, 128, 255}, LAMP = {255, 214, 160, 255};

static Color scale_col(Color c, float k) {
    return (Color){(unsigned char)fminf(255, c.r * k), (unsigned char)fminf(255, c.g * k), (unsigned char)fminf(255, c.b * k), 255};
}

// Radial pool on the ground: bright centre fading to nothing at radius R.
static void ground_pool(V3 c, float R, Color col) {
    Color off = {0, 0, 0, 255};
    V3 cc = v3(c.x, terrain_height(c.x, c.z) + 0.05f, c.z);
    for (int k = 0; k < 14; k++) {
        float a0 = 6.2831853f * k / 14, a1 = 6.2831853f * (k + 1) / 14;
        V3 p0 = v3(c.x + cosf(a0) * R, 0, c.z + sinf(a0) * R), p1 = v3(c.x + cosf(a1) * R, 0, c.z + sinf(a1) * R);
        p0.y = terrain_height(p0.x, p0.z) + 0.05f; p1.y = terrain_height(p1.x, p1.z) + 0.05f;
        // Mid ring keeps the falloff soft instead of a linear cone.
        V3 m0 = vlerp(cc, p0, 0.5f), m1 = vlerp(cc, p1, 0.5f);
        m0.y = terrain_height(m0.x, m0.z) + 0.05f; m1.y = terrain_height(m1.x, m1.z) + 0.05f;
        Color mid = scale_col(col, 0.35f);
        mb_tri3(&POOL, cc, m0, m1, col, mid, mid);
        mb_tri3(&POOL, m0, p0, p1, mid, off, off);
        mb_tri3(&POOL, m0, p1, m1, mid, off, mid);
    }
}

// A lamp on a facade at (u along it, y up): fixture, glowing lens, a wash down the wall, a pool below.
static void wall_lamp(MeshB *m, const Building *B, int s, float u, float y) {
    V3 o, a, n; float len;
    facade(B, s, &o, &a, &n, &len);
    V3 p = vadd(vadd(o, vscale(a, u)), v3(0, B->ground + y, 0));
    V3 q = vadd(p, vscale(n, 0.28f));
    V3 mn = v3(fminf(p.x, q.x) - 0.18f * fabsf(a.x), p.y - 0.10f, fminf(p.z, q.z) - 0.18f * fabsf(a.z));
    V3 mx = v3(fmaxf(p.x, q.x) + 0.18f * fabsf(a.x), p.y + 0.12f, fmaxf(p.z, q.z) + 0.18f * fabsf(a.z));
    box_shell(m, mn, mx, (Color){40, 40, 42, 255}, (Color){40, 40, 42, 255});
    // Lens on the underside.
    V3 l0 = v3(mn.x, mn.y - 0.01f, mn.z), l1 = v3(mx.x, mn.y - 0.01f, mn.z), l2 = v3(mx.x, mn.y - 0.01f, mx.z), l3 = v3(mn.x, mn.y - 0.01f, mx.z);
    mbFlat = 1; mb_quad(&GLOW, l0, l1, l2, l3, LAMP, v3(0, 1e4f, 0)); mbFlat = 0;
    // Wash down the wall.
    V3 w0 = vadd(p, vscale(n, 0.05f));
    V3 b0 = vadd(vadd(w0, vscale(a, -2.6f)), v3(0, -y, 0)), b1 = vadd(vadd(w0, vscale(a, 2.6f)), v3(0, -y, 0));
    V3 s0 = vadd(w0, vscale(a, -0.9f)), s1 = vadd(w0, vscale(a, 0.9f));
    Color hot = scale_col(LAMP, 0.55f), off = {0, 0, 0, 255};
    mb_tri3(&POOL, s0, s1, b1, hot, hot, off);
    mb_tri3(&POOL, s0, b1, b0, hot, off, off);
    // Pool on the ground in front.
    V3 g = vadd(p, vscale(n, 1.8f));
    ground_pool(g, 4.2f, scale_col(LAMP, 0.42f));
}

// A street lamp at (x, z) with its arm reaching toward (dx, dz).
static void street_lamp(MeshB *m, float x, float z, float dx, float dz) {
    float g = terrain_height(x, z);
    Color pole = {58, 60, 64, 255};
    box_shell(m, v3(x - 0.09f, g - 0.3f, z - 0.09f), v3(x + 0.09f, g + 6.6f, z + 0.09f), pole, pole);
    float hx = x + dx * 1.5f, hz = z + dz * 1.5f;
    box_shell(m, v3(fminf(x, hx) - 0.05f, g + 6.45f, fminf(z, hz) - 0.05f), v3(fmaxf(x, hx) + 0.05f, g + 6.55f, fmaxf(z, hz) + 0.05f), pole, pole);
    box_shell(m, v3(hx - 0.32f, g + 6.3f, hz - 0.18f), v3(hx + 0.32f, g + 6.5f, hz + 0.18f), pole, pole);
    V3 l0 = v3(hx - 0.28f, g + 6.29f, hz - 0.14f), l1 = v3(hx + 0.28f, g + 6.29f, hz - 0.14f);
    V3 l2 = v3(hx + 0.28f, g + 6.29f, hz + 0.14f), l3 = v3(hx - 0.28f, g + 6.29f, hz + 0.14f);
    mbFlat = 1; mb_quad(&GLOW, l0, l1, l2, l3, LAMP, v3(0, 1e4f, 0)); mbFlat = 0;
    ground_pool(v3(hx, 0, hz), 8.0f, scale_col(LAMP, 0.36f));
}

static int lamp_spot_ok(const World *w, float x, float z) {
    float g = terrain_height(x, z);
    return road_at(x, z) == 0 && !world_point_solid(w, v3(x, g + 1.0f, z)) &&
           !world_point_solid(w, v3(x + 0.6f, g + 1.0f, z)) && !world_point_solid(w, v3(x - 0.6f, g + 1.0f, z)) &&
           !world_point_solid(w, v3(x, g + 1.0f, z + 0.6f)) && !world_point_solid(w, v3(x, g + 1.0f, z - 0.6f));
}

static void place_street_lamps(MeshB *m, const World *w) {
    for (int side = -1; side <= 1; side += 2) {
        for (float x = 16.5f; x < 110; x += 18) {                       // avenue, both kerbs
            if (fabsf(x - 40) < 7 || fabsf(x - 80) < 7) continue;
            if (lamp_spot_ok(w, side * x, 43.4f)) street_lamp(m, side * x, 43.4f, 0, 1);
            if (lamp_spot_ok(w, side * (x + 9), 52.6f)) street_lamp(m, side * (x + 9), 52.6f, 0, -1);
        }
        for (float x = 16.5f; x < 78; x += 18)                          // upper avenue
            if (fabsf(x - 40) > 7 && lamp_spot_ok(w, side * x, 93.4f)) street_lamp(m, side * x, 93.4f, 0, 1);
        for (float z = 18; z < 132; z += 20) {                          // side streets
            if (fabsf(z - 48) < 7 || fabsf(z - 97) < 6) continue;
            if (lamp_spot_ok(w, side * 35.4f, z)) street_lamp(m, side * 35.4f, z, side * 1.0f, 0);
            if (z < 92 && lamp_spot_ok(w, side * 75.4f, z + 10)) street_lamp(m, side * 75.4f, z + 10, side * 1.0f, 0);
        }
        for (float x = 20; x < 120; x += 30)                            // highway through the start area
            if (lamp_spot_ok(w, side * x, 8.0f)) street_lamp(m, side * x, 8.0f, 0, -1);
    }
    for (float x = 36; x < 76; x += 18)                                 // warehouse service road
        if (lamp_spot_ok(w, x, -13.6f)) street_lamp(m, x, -13.6f, 0, 1);
}

// ------------------------------------------------------------- New York ----

// Lamps along every avenue and cross-street kerb.
static void place_lamps_newyork(MeshB *m, const World *w) {
    const float av[3][2] = {{-8, 8}, {48, 60}, {-60, -48}};
    const float st[4][2] = {{44, 50}, {72, 78}, {100, 106}, {124, CITY_Z1}};
    for (int i = 0; i < 3; i++)
        for (float z = 16; z < CITY_Z1 - 4; z += 22) {
            if (lamp_spot_ok(w, av[i][0] - 0.7f, z)) street_lamp(m, av[i][0] - 0.7f, z, 1, 0);
            if (lamp_spot_ok(w, av[i][1] + 0.7f, z + 11)) street_lamp(m, av[i][1] + 0.7f, z + 11, -1, 0);
        }
    for (int j = 0; j < 4; j++)
        for (float x = -CITY_X + 4; x < CITY_X - 4; x += 24) {
            if (lamp_spot_ok(w, x, st[j][0] - 0.7f)) street_lamp(m, x, st[j][0] - 0.7f, 0, 1);
            if (j < 3 && lamp_spot_ok(w, x + 12, st[j][1] + 0.7f)) street_lamp(m, x + 12, st[j][1] + 0.7f, 0, -1);
        }
}

// The red grandstand: eight glowing steps rising to the north.
static void build_steps(MeshB *m, const Building *B) {
    Color red = {190, 26, 30, 255};
    float g = B->ground, len = B->mx.z - B->mn.z;
    for (int i = 0; i < 8; i++) {
        float z0 = B->mn.z + len * i / 8.0f, y1 = g + 0.42f * (i + 1);
        box_shell(m, v3(B->mn.x, B->mn.y, z0), v3(B->mx.x, y1, B->mx.z), red, (Color){150, 20, 24, 255});
        V3 r0 = v3(B->mn.x + 0.1f, y1 - 0.38f, z0 - 0.02f), r1 = v3(B->mx.x - 0.1f, y1 - 0.38f, z0 - 0.02f);
        V3 r2 = v3(B->mx.x - 0.1f, y1 - 0.04f, z0 - 0.02f), r3 = v3(B->mn.x + 0.1f, y1 - 0.04f, z0 - 0.02f);
        mbFlat = 1; mb_quad(&GLOW, r0, r1, r2, r3, (Color){255, 50, 50, 255}, v3((B->mn.x + B->mx.x) * 0.5f, y1, z0 + 1)); mbFlat = 0;
    }
    ground_pool(v3((B->mn.x + B->mx.x) * 0.5f, 0, B->mn.z - 3.0f), 7.0f, (Color){120, 18, 20, 255});
}

// The screen-clad wedge: a dark tower (the screens are a separate, animated mesh) with a mast on top.
static void build_wedge(MeshB *m, const Building *B) {
    box_shell(m, B->mn, B->mx, (Color){42, 44, 50, 255}, (Color){36, 38, 42, 255});
    V3 c = v3((B->mn.x + B->mx.x) * 0.5f, B->mx.y, (B->mn.z + B->mx.z) * 0.5f);
    box_shell(m, v3(c.x - 0.25f, c.y, c.z - 0.25f), v3(c.x + 0.25f, c.y + 9.0f, c.z + 0.25f), (Color){120, 122, 128, 255}, (Color){120, 122, 128, 255});
    box_shell(m, v3(c.x - 1.1f, c.y + 9.0f, c.z - 1.1f), v3(c.x + 1.1f, c.y + 11.2f, c.z + 1.1f), (Color){200, 205, 215, 255}, (Color){220, 225, 235, 255});
    mbFlat = 1;
    mb_quad(&GLOW, v3(c.x - 1.15f, c.y + 9.3f, c.z - 1.15f), v3(c.x + 1.15f, c.y + 9.3f, c.z - 1.15f),
            v3(c.x + 1.15f, c.y + 10.9f, c.z - 1.15f), v3(c.x - 1.15f, c.y + 10.9f, c.z - 1.15f), (Color){230, 240, 255, 255}, c);
    mbFlat = 0;
}

// Road paint: zebra crossings at every avenue/street junction and dashed lane lines.
static Model markings; static int hasMarkings;
static void paint_rect(MeshB *m, float x0, float x1, float z0, float z1) {
    V3 a = v3(x0, 0, z0), b = v3(x1, 0, z0), c = v3(x1, 0, z1), d = v3(x0, 0, z1);
    a.y = terrain_height(a.x, a.z) + 0.03f; b.y = terrain_height(b.x, b.z) + 0.03f;
    c.y = terrain_height(c.x, c.z) + 0.03f; d.y = terrain_height(d.x, d.z) + 0.03f;
    mb_quad(m, a, b, c, d, (Color){222, 222, 214, 255}, v3((x0 + x1) * 0.5f, -10, (z0 + z1) * 0.5f));
}
static void build_markings_airport(MeshB *m);
static void build_markings(const World *w) {
    if (hasMarkings) { UnloadModel(markings); hasMarkings = 0; }
    if (w->map == MAP_AIRPORT) { MeshB am = {0}; build_markings_airport(&am); markings = mb_model(&am); hasMarkings = 1; return; }
    if (w->map != MAP_NEWYORK) return;
    MeshB m = {0};
    const float av[3][2] = {{-8, 8}, {48, 60}, {-60, -48}};
    const float st[4][2] = {{44, 50}, {72, 78}, {100, 106}, {124, CITY_Z1}};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++) {
            for (int sd = 0; sd < 2; sd++) {
                // Crossing the street, either side of the avenue: stripes run along x.
                float bx0 = sd ? av[i][1] + 0.6f : av[i][0] - 3.6f;
                for (float z = st[j][0] + 0.4f; z + 0.45f < st[j][1] - 0.3f; z += 0.9f) paint_rect(&m, bx0, bx0 + 3.0f, z, z + 0.45f);
                // Crossing the avenue, either side of the street: stripes run along z.
                float bz0 = sd ? st[j][1] + 0.6f : st[j][0] - 3.6f;
                if (bz0 < CITY_Z0 + 1 || bz0 + 3 > CITY_Z1) continue;
                for (float x = av[i][0] + 0.4f; x + 0.45f < av[i][1] - 0.3f; x += 0.9f) paint_rect(&m, x, x + 0.45f, bz0, bz0 + 3.0f);
            }
        }
    for (int i = 0; i < 3; i++) {                       // lane lines along the avenues
        float wdt = av[i][1] - av[i][0];
        int lanes = (int)(wdt / 3.8f);
        for (int l = 1; l < lanes; l++) {
            float x = av[i][0] + wdt * l / lanes;
            for (float z = CITY_Z0 + 2; z < CITY_Z1 - 3; z += 9) {
                int junction = 0;
                for (int j = 0; j < 4; j++) if (z + 3 > st[j][0] - 4 && z < st[j][1] + 4) junction = 1;
                if (!junction && road_at(x, z) != 3) paint_rect(&m, x - 0.08f, x + 0.08f, z, z + 3.0f);
            }
        }
    }
    for (int j = 0; j < 4; j++)                          // centre lines along the streets
        for (float x = -CITY_X + 2; x < CITY_X - 3; x += 9) {
            float zc = (st[j][0] + st[j][1]) * 0.5f;
            int junction = 0;
            for (int i = 0; i < 3; i++) if (x + 3 > av[i][0] - 4 && x < av[i][1] + 4) junction = 1;
            if (!junction && road_at(x, zc) == 2 && road_at(x + 3, zc) == 2) paint_rect(&m, x, x + 3.0f, zc - 0.08f, zc + 0.08f);
        }
    markings = mb_model(&m);
    hasMarkings = 1;
}

// Animated LED screens on the facades around the bowtie (and all over the wedge tower).
static Model screens; static int hasScreens;
typedef struct { float *v, *t; unsigned char *c; int n, cap; } ScreenMesh;
static void sm_push(ScreenMesh *s, V3 p, float u, float v, const unsigned char c[4]) {
    if (s->n == s->cap) {
        s->cap = s->cap ? s->cap * 2 : 1024;
        s->v = MemRealloc(s->v, s->cap * 3 * sizeof(float));
        s->t = MemRealloc(s->t, s->cap * 2 * sizeof(float));
        s->c = MemRealloc(s->c, s->cap * 4);
    }
    s->v[s->n * 3] = p.x; s->v[s->n * 3 + 1] = p.y; s->v[s->n * 3 + 2] = p.z;
    s->t[s->n * 2] = u; s->t[s->n * 2 + 1] = v;
    memcpy(s->c + s->n * 4, c, 4);
    s->n++;
}
// A screen centred at ctr on a wall with outward normal n.
static void add_screen(ScreenMesh *s, V3 ctr, V3 n, float W, float H, int seed, int style) {
    V3 up = v3(0, 1, 0), right = vnorm(vcross(up, n));    // the viewer's right, so content reads left to right
    unsigned char c[4] = {(unsigned char)seed, (unsigned char)style, (unsigned char)fminf(255, W / 64 * 255), (unsigned char)fminf(255, H / 64 * 255)};
    V3 o = vsub(vsub(ctr, vscale(right, W * 0.5f)), vscale(up, H * 0.5f));
    V3 p1 = vadd(o, vscale(right, W)), p2 = vadd(p1, vscale(up, H)), p3 = vadd(o, vscale(up, H));
    sm_push(s, o, 0, 0, c); sm_push(s, p1, W, 0, c); sm_push(s, p2, W, H, c);
    sm_push(s, o, 0, 0, c); sm_push(s, p2, W, H, c); sm_push(s, p3, 0, H, c);
}
static void build_screens(const World *w) {
    if (hasScreens) { UnloadModel(screens); hasScreens = 0; }
    if (w->map != MAP_NEWYORK) return;
    ScreenMesh sm = {0};
    MeshB pools = {0};
    for (int bi = 0; bi < w->nbld; bi++) {
        const Building *B = &w->bld[bi];
        if (B->kind == BLD_WAREHOUSE || B->kind == BLD_STEPS || B->kind == BLD_HOUSE) continue;
        int wedge = B->kind == BLD_WEDGE;
        float Hb = B->mx.y - B->ground;
        for (int s = 0; s < 4; s++) {
            V3 o, a, n; float len;
            facade(B, s, &o, &a, &n, &len);
            V3 fc = vadd(o, vscale(a, len * 0.5f));
            V3 toC = v3(TS_X - fc.x, 0, TS_Z - fc.z);
            float d = vlen(toC), facing = d > 1e-3f ? vdot(n, vscale(toC, 1.0f / d)) : 1;
            if (!wedge && (d > 105 || (facing < 0.2f && d > 30))) continue;
            bseed = B->seed * 7u + (unsigned)s * 131u;
            int count = wedge ? 7 : (d < 50 ? 3 : d < 80 ? 2 : 1);
            float y = wedge ? 4.0f : 3.5f + brnd() * 3.0f, top = wedge ? 78.0f : fminf(Hb - 2.0f, 50.0f);
            for (int k = 0; k < count && y < top; k++) {
                float W = wedge ? len - 0.6f : fminf(len * (0.45f + 0.5f * brnd()), 28.0f);
                float H = wedge ? 7.0f + 4.0f * brnd() : 4.5f + 8.5f * brnd();
                if (W < 3.0f || y + H > top) break;
                float uc = len * 0.5f + (wedge ? 0.0f : (brnd() * 2 - 1) * (len - W) * 0.5f);
                V3 ctr = vadd(vadd(o, vscale(a, uc)), vadd(vscale(n, 0.18f), v3(0, B->ground + y + H * 0.5f, 0)));
                int seed = (int)(brnd() * 255), style = (int)(brnd() * 4) & 3;
                add_screen(&sm, ctr, n, W, H, seed, style);
                if (y < 16) {                              // the screen's glow spills onto the street
                    float hue = seed / 255.0f;
                    Color glow = {(unsigned char)(255 * (0.55f + 0.45f * cosf(6.2831f * hue))), (unsigned char)(255 * (0.55f + 0.45f * cosf(6.2831f * (hue - 0.33f)))),
                                  (unsigned char)(255 * (0.55f + 0.45f * cosf(6.2831f * (hue - 0.67f)))), 255};
                    V3 g = vadd(vadd(o, vscale(a, uc)), vscale(n, 5.0f));
                    MeshB keep = POOL; POOL = pools;
                    ground_pool(g, fmaxf(4.0f, W * 0.45f), scale_col(glow, 0.22f));
                    pools = POOL; POOL = keep;
                }
                y += H + 1.0f + 3.0f * brnd();
            }
        }
    }
    if (!sm.n) return;
    Mesh mesh = {0};
    mesh.vertexCount = sm.n; mesh.triangleCount = sm.n / 3;
    mesh.vertices = sm.v; mesh.texcoords = sm.t; mesh.colors = sm.c;
    UploadMesh(&mesh, false);
    screens = LoadModelFromMesh(mesh);
    screens.materials[0].shader = post_billboard_shader();
    hasScreens = 1;
    // Fold the street glow into the city's light pools.
    for (int i = 0; i < pools.n; i++) mb_push(&POOL, v3(pools.v[i * 3], pools.v[i * 3 + 1], pools.v[i * 3 + 2]),
                                              (Color){pools.c[i * 4], pools.c[i * 4 + 1], pools.c[i * 4 + 2], 255});
    if (pools.v) { MemFree(pools.v); MemFree(pools.c); }
}

// A window: glass, plus a warm glowing pane if someone is home.
static float glowK = 1.0f;                  // brightness of lit panes (shopfronts are big, so dimmer)
static void window(MeshB *m, const Building *B, int s, float u, float y, float w, float h, float litP, Color glass) {
    facade_rect(m, B, s, u, y, w, h, 0.04f, glass);
    if (brnd() < litP) { mbFlat = 1; facade_rect(&GLOW, B, s, u + 0.06f, y + 0.06f, w - 0.12f, h - 0.12f, 0.05f, scale_col(WARM, glowK * (0.75f + 0.25f * brnd()))); mbFlat = 0; }
}

// -------------------------------------------------------------- airport ----

static const Color LIVERY[] = {{200, 30, 40, 255}, {20, 50, 120, 255}, {0, 130, 140, 255}, {230, 110, 20, 255}, {40, 120, 60, 255}, {110, 40, 140, 255}};

// Terminal: pale walls, a glass curtain on both long faces, a canopy over the forecourt kerb.
static void build_terminal(MeshB *m, const Building *B) {
    bseed = B->seed;
    box_shell(m, B->mn, B->mx, (Color){212, 214, 218, 255}, (Color){150, 154, 160, 255});
    float H = B->mx.y - B->ground, len = B->mx.x - B->mn.x;
    Color glass = {52, 70, 90, 255};
    for (int s = 2; s <= 3; s++) {                         // +z (airside) and -z (landside) faces
        int nc = (int)(len / 3.0f);
        for (int i = 0; i < nc; i++)
            for (float y = 0.6f; y + 3.4f < H - 2.0f; y += 4.2f) window(m, B, s, i * 3.0f + 0.15f, y, 2.7f, 3.4f, 0.22f, glass);
    }
    for (int s = 0; s <= 1; s++) facade_rect(m, B, s, 2.0f, H - 3.0f, (B->mx.z - B->mn.z) - 4.0f, 1.6f, 0.04f, (Color){30, 90, 160, 255});
    float g = B->ground;
    box_shell(m, v3(B->mn.x - 2, g + 7.0f, B->mn.z - 4.5f), v3(B->mx.x + 2, g + 7.6f, B->mn.z), (Color){190, 194, 200, 255}, (Color){170, 174, 180, 255});
    for (float x = B->mn.x + 4; x < B->mx.x; x += 14)       // canopy columns
        box_shell(m, v3(x - 0.2f, g, B->mn.z - 4.0f), v3(x + 0.2f, g + 7.0f, B->mn.z - 3.6f), (Color){120, 124, 130, 255}, (Color){120, 124, 130, 255});
    for (float x = B->mn.x + 6; x < B->mx.x; x += 20)       // canopy downlights
        ground_pool(v3(x, 0, B->mn.z - 2.5f), 5.0f, scale_col(LAMP, 0.36f));
}

// Control tower: a concrete shaft, a wide glass cab that glows, a roof and a beacon mast.
static void build_atc(MeshB *m, const Building *B) {
    float g = B->ground, top = B->mx.y, cabLo = top - 7.0f, cabHi = top - 2.5f;
    V3 c = v3((B->mn.x + B->mx.x) * 0.5f, 0, (B->mn.z + B->mx.z) * 0.5f);
    Color conc = {196, 194, 188, 255};
    box_shell(m, v3(B->mn.x + 1, B->mn.y, B->mn.z + 1), v3(B->mx.x - 1, cabLo, B->mx.z - 1), conc, conc);
    box_shell(m, v3(c.x - 6, cabLo, c.z - 6), v3(c.x + 6, cabLo + 0.8f, c.z + 6), conc, conc);
    box_shell(m, v3(c.x - 5.6f, cabLo + 0.8f, c.z - 5.6f), v3(c.x + 5.6f, cabHi, c.z + 5.6f), (Color){44, 58, 72, 255}, conc);
    mbFlat = 1;
    for (int s = 0; s < 4; s++) {                           // lit cab glazing
        V3 n = s == 0 ? v3(1, 0, 0) : s == 1 ? v3(-1, 0, 0) : s == 2 ? v3(0, 0, 1) : v3(0, 0, -1);
        V3 a = v3(n.z, 0, -n.x);
        V3 base = vadd(c, vscale(n, 5.65f));
        V3 p0 = vadd(vsub(base, vscale(a, 5.2f)), v3(0, cabLo + 1.4f, 0)), p1 = vadd(vadd(base, vscale(a, 5.2f)), v3(0, cabLo + 1.4f, 0));
        mb_quad(&GLOW, p0, p1, vadd(p1, v3(0, cabHi - cabLo - 2.0f, 0)), vadd(p0, v3(0, cabHi - cabLo - 2.0f, 0)), (Color){170, 200, 225, 255}, c);
    }
    mbFlat = 0;
    box_shell(m, v3(c.x - 6.5f, cabHi, c.z - 6.5f), v3(c.x + 6.5f, cabHi + 0.8f, c.z + 6.5f), (Color){90, 92, 96, 255}, (Color){90, 92, 96, 255});
    box_shell(m, v3(c.x - 0.15f, cabHi + 0.8f, c.z - 0.15f), v3(c.x + 0.15f, top + 6.0f, c.z + 0.15f), (Color){180, 40, 40, 255}, (Color){180, 40, 40, 255});
    mbFlat = 1; box_shell(&GLOW, v3(c.x - 0.3f, top + 6.0f, c.z - 0.3f), v3(c.x + 0.3f, top + 6.6f, c.z + 0.3f), (Color){255, 40, 30, 255}, (Color){255, 40, 30, 255}); mbFlat = 0;
    (void)g;
}

// Hangar: corrugated walls, a low curved-look gable roof, huge doors with a lit gap.
static void build_hangar(MeshB *m, const Building *B) {
    Color wall = {168, 172, 178, 255};
    box_shell(m, B->mn, B->mx, wall, wall);
    float H = B->mx.y - B->ground, len = B->mx.x - B->mn.x;
    for (float u = 0.6f; u < len; u += 1.2f) facade_rect(m, B, 3, u, 0.0f, 0.12f, H, 0.03f, (Color){150, 154, 160, 255});   // ribs (back)
    facade_rect(m, B, 2, 1.5f, 0.0f, len - 3.0f, H * 0.82f, 0.04f, (Color){74, 78, 86, 255});                          // doors
    for (float u = 1.5f; u < len - 1.5f; u += (len - 3.0f) / 6.0f) facade_rect(m, B, 2, u, 0.0f, 0.12f, H * 0.82f, 0.05f, (Color){50, 52, 58, 255});
    mbFlat = 1; facade_rect(&GLOW, B, 2, len * 0.5f - 2.5f, 0.0f, 5.0f, H * 0.82f - 0.2f, 0.06f, scale_col(WARM, 0.55f)); mbFlat = 0;   // doors ajar
    V3 o, a, n; float l;
    facade(B, 2, &o, &a, &n, &l);
    ground_pool(vadd(vadd(o, vscale(a, len * 0.5f)), vscale(n, 7.0f)), 9.0f, scale_col(WARM, 0.3f));
    // Roof: gable whose ridge runs front to back.
    float e = 0.5f, wt = B->mx.y, rt = B->mx.y + B->roof, cx = (B->mn.x + B->mx.x) * 0.5f;
    V3 ref = v3(cx, B->mx.y - 2, (B->mn.z + B->mx.z) * 0.5f);
    Color roofc = {130, 134, 140, 255};
    mb_quad(m, v3(B->mn.x - e, wt, B->mn.z - e), v3(B->mn.x - e, wt, B->mx.z + e), v3(cx, rt, B->mx.z + e), v3(cx, rt, B->mn.z - e), roofc, ref);
    mb_quad(m, v3(B->mx.x + e, wt, B->mn.z - e), v3(B->mx.x + e, wt, B->mx.z + e), v3(cx, rt, B->mx.z + e), v3(cx, rt, B->mn.z - e), roofc, ref);
    mb_tri(m, v3(B->mn.x, wt, B->mn.z), v3(B->mx.x, wt, B->mn.z), v3(cx, rt, B->mn.z), wall, ref);
    mb_tri(m, v3(B->mn.x, wt, B->mx.z), v3(B->mx.x, wt, B->mx.z), v3(cx, rt, B->mx.z), wall, ref);
}

// Tall apron floodlight mast.
static void flood_mast(MeshB *m, float x, float z) {
    float g = terrain_height(x, z);
    Color pole = {120, 124, 130, 255};
    box_shell(m, v3(x - 0.25f, g - 0.3f, z - 0.25f), v3(x + 0.25f, g + 20.0f, z + 0.25f), pole, pole);
    box_shell(m, v3(x - 1.6f, g + 20.0f, z - 0.5f), v3(x + 1.6f, g + 21.2f, z + 0.5f), (Color){60, 62, 66, 255}, (Color){60, 62, 66, 255});
    mbFlat = 1;
    mb_quad(&GLOW, v3(x - 1.5f, g + 19.98f, z - 0.45f), v3(x + 1.5f, g + 19.98f, z - 0.45f), v3(x + 1.5f, g + 19.98f, z + 0.45f), v3(x - 1.5f, g + 19.98f, z + 0.45f), LAMP, v3(x, g + 30, z));
    mbFlat = 0;
    ground_pool(v3(x, 0, z), 22.0f, scale_col(LAMP, 0.30f));
}

// Static airport dressing (the parked airliners are soft bodies, drawn each frame): jet bridges, masts, landside lamps, parked cars, airfield lights.
static void build_airport_extras(MeshB *m, const World *w) {
    for (int i = 0; i < w->nplanes; i++) {
        const Plane *P = &w->planes[i];
        V3 nose = v3(P->x, terrain_height(P->x, P->z), P->z);
        if (P->dz > 0.5f && P->z < 100) {                  // at a gate: jet bridge from the terminal to the forward door
            Color jb = {170, 174, 180, 255};
            float bx = P->x - 6.0f, g = nose.y - 0.35f;     // the airframes sit 0.35 m low on their gear
            box_shell(m, v3(bx - 1.3f, g + 3.6f, AP_TERM_Z1), v3(bx + 1.3f, g + 6.2f, P->z + 2.5f), jb, jb);
            box_shell(m, v3(bx, g + 3.4f, P->z + 2.5f), v3(P->x - 1.9f, g + 6.0f, P->z + 6.5f), jb, jb);
            box_shell(m, v3(bx - 0.6f, g, P->z + 1.5f), v3(bx + 0.6f, g + 3.6f, P->z + 2.5f), (Color){90, 92, 96, 255}, jb);
        }
    }
    for (float x = -100; x <= 110; x += 30) flood_mast(m, x, 80.8f);
    flood_mast(m, -160, 137.5f); flood_mast(m, -130, 137.5f);
    for (float x = -60; x < 104; x += 18) {                // forecourt lamps
        if (lamp_spot_ok(w, x, 35.3f)) street_lamp(m, x, 35.3f, 0, 1);
    }
    for (float z = 12; z < 76; z += 20) {
        if (lamp_spot_ok(w, -64.7f, z)) street_lamp(m, -64.7f, z, 1, 0);
        if (lamp_spot_ok(w, -7.7f, z) && z < 34) street_lamp(m, -7.7f, z, 1, 0);
    }
    // Parked cars in the car parks: simple boxes, but they fill the bays.
    bseed = 99u;
    const float parks[2][4] = {{10, 90, 14, 34}, {-50, -12, 14, 34}};
    for (int p = 0; p < 2; p++)
        for (int row = 0; row < 2; row++)
            for (float x = parks[p][0] + 1.6f; x < parks[p][1] - 1.6f; x += 2.6f) {
                if (brnd() < 0.3f) continue;
                float z0 = row ? parks[p][3] - 5.0f : parks[p][2] + 0.6f, g = terrain_height(x, z0 + 2);
                Color c = scale_col(LIVERY[(int)(brnd() * 6)], 0.6f + 0.5f * brnd());
                if (brnd() < 0.4f) c = (Color){(unsigned char)(170 + 60 * brnd()), (unsigned char)(170 + 60 * brnd()), (unsigned char)(175 + 60 * brnd()), 255};
                box_shell(m, v3(x - 0.85f, g + 0.25f, z0 + 0.3f), v3(x + 0.85f, g + 0.95f, z0 + 4.4f), c, c);
                box_shell(m, v3(x - 0.75f, g + 0.95f, z0 + 1.4f), v3(x + 0.75f, g + 1.45f, z0 + 3.4f), (Color){45, 55, 68, 255}, c);
            }
    // Airfield lighting (glows at dusk): runway edges white, thresholds green, taxiway edges blue, approach bars.
    mbFlat = 1;
    #define LIGHT(x, z, col) box_shell(&GLOW, v3((x) - 0.18f, terrain_height(x, z) + 0.05f, (z) - 0.18f), v3((x) + 0.18f, terrain_height(x, z) + 0.4f, (z) + 0.18f), col, col)
    for (float x = AP_RWY_X0 + 2; x <= AP_RWY_X1 - 2; x += 30) { LIGHT(x, AP_RWY_Z0 + 0.6f, ((Color){255, 250, 230, 255})); LIGHT(x, AP_RWY_Z1 - 0.6f, ((Color){255, 250, 230, 255})); }
    for (float z = AP_RWY_Z0 + 1; z <= AP_RWY_Z1 - 1; z += 2.5f) { LIGHT(AP_RWY_X0 + 0.6f, z, ((Color){60, 255, 120, 255})); LIGHT(AP_RWY_X1 - 0.6f, z, ((Color){60, 255, 120, 255})); }
    for (float x = -AP_TWY_X; x <= AP_TWY_X; x += 25) { LIGHT(x, AP_TWY_Z0 - 0.6f, ((Color){60, 110, 255, 255})); LIGHT(x, AP_TWY_Z1 + 0.6f, ((Color){60, 110, 255, 255})); }
    for (float x = AP_RWY_X0 - 8; x > -296; x -= 9)
        for (float dz = -4; dz <= 4; dz += 2) LIGHT(x, (AP_RWY_Z0 + AP_RWY_Z1) * 0.5f + dz, ((Color){255, 250, 230, 255}));
    #undef LIGHT
    mbFlat = 0;
}

// Runway, taxiway, apron and car-park paint.
static void paint_c(MeshB *m, float x0, float x1, float z0, float z1, Color c) {
    V3 a = v3(x0, 0, z0), b = v3(x1, 0, z0), cc = v3(x1, 0, z1), d = v3(x0, 0, z1);
    a.y = terrain_height(a.x, a.z) + 0.03f; b.y = terrain_height(b.x, b.z) + 0.03f; cc.y = terrain_height(cc.x, cc.z) + 0.03f; d.y = terrain_height(d.x, d.z) + 0.03f;
    mb_quad(m, a, b, cc, d, c, v3((x0 + x1) * 0.5f, -10, (z0 + z1) * 0.5f));
}
// A 7-segment digit. (u, v): across and up in the reader's frame; the reader faces +x (east) or -x.
static void paint_digit(MeshB *m, int dgt, float x, float zLeft, int east) {
    static const unsigned char SEG[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};   // bits a b c d e f g
    const float W = 7.0f, H = 12.0f, t = 1.2f;
    const float r[7][4] = {{0, W, H - t, H}, {W - t, W, H / 2, H}, {W - t, W, 0, H / 2}, {0, W, 0, t}, {0, t, 0, H / 2}, {0, t, H / 2, H}, {0, W, H / 2 - t / 2, H / 2 + t / 2}};
    Color white = {225, 225, 220, 255};
    for (int sgm = 0; sgm < 7; sgm++) {
        if (!(SEG[dgt] >> sgm & 1)) continue;
        float u0 = r[sgm][0], u1 = r[sgm][1], v0 = r[sgm][2], v1 = r[sgm][3];
        if (east) paint_c(m, x + v0, x + v1, zLeft + u0, zLeft + u1, white);
        else paint_c(m, x - v1, x - v0, zLeft - u1, zLeft - u0, white);
    }
}
static void build_markings_airport(MeshB *m) {
    Color white = {225, 225, 220, 255}, yellow = {230, 190, 40, 255};
    float zc = (AP_RWY_Z0 + AP_RWY_Z1) * 0.5f;
    paint_c(m, AP_RWY_X0 + 1, AP_RWY_X1 - 1, AP_RWY_Z0 + 1.0f, AP_RWY_Z0 + 1.9f, white);       // edge lines
    paint_c(m, AP_RWY_X0 + 1, AP_RWY_X1 - 1, AP_RWY_Z1 - 1.9f, AP_RWY_Z1 - 1.0f, white);
    for (float x = AP_RWY_X0 + 40; x < AP_RWY_X1 - 70; x += 50) paint_c(m, x, x + 30, zc - 0.45f, zc + 0.45f, white);   // centreline
    for (int end = 0; end < 2; end++) {
        float x0 = end ? AP_RWY_X1 - 33 : AP_RWY_X0 + 3;
        for (int b = 0; b < 6; b++) {                         // threshold "piano keys"
            float z = zc + 2.5f + b * 2.4f;
            paint_c(m, x0, x0 + 30, z, z + 1.5f, white);
            paint_c(m, x0, x0 + 30, 2 * zc - z - 1.5f, 2 * zc - z, white);
        }
        float tz = end ? AP_RWY_X1 - 130 : AP_RWY_X0 + 100;  // touchdown zone bars
        paint_c(m, tz, tz + 22, zc + 4.0f, zc + 7.0f, white); paint_c(m, tz, tz + 22, zc - 7.0f, zc - 4.0f, white);
    }
    // "09" read by pilots heading east, "27" by pilots heading west.
    paint_digit(m, 0, AP_RWY_X0 + 40, zc - 8.0f, 1); paint_digit(m, 9, AP_RWY_X0 + 40, zc + 1.0f, 1);
    paint_digit(m, 2, AP_RWY_X1 - 40, zc + 8.0f, 0); paint_digit(m, 7, AP_RWY_X1 - 40, zc - 1.0f, 0);
    float tzc = (AP_TWY_Z0 + AP_TWY_Z1) * 0.5f;              // taxiway centreline and connectors
    paint_c(m, -AP_TWY_X, AP_TWY_X, tzc - 0.15f, tzc + 0.15f, yellow);
    const float cx[3] = {-235, 0, 235};
    for (int i = 0; i < 3; i++) {
        paint_c(m, cx[i] - 0.15f, cx[i] + 0.15f, tzc, AP_RWY_Z0, yellow);
        paint_c(m, cx[i] - 5.5f, cx[i] + 5.5f, AP_RWY_Z0 - 6.0f, AP_RWY_Z0 - 5.7f, yellow);    // holding position
        paint_c(m, cx[i] - 5.5f, cx[i] + 5.5f, AP_RWY_Z0 - 5.2f, AP_RWY_Z0 - 4.9f, yellow);
    }
    for (int g = 0; g < 4; g++) {                             // gate lead-in lines and stop bars
        paint_c(m, AP_GATES[g] - 0.15f, AP_GATES[g] + 0.15f, AP_NOSE_Z + 2, AP_APRON_Z1 - 1, yellow);
        paint_c(m, AP_GATES[g] - 2.0f, AP_GATES[g] + 2.0f, AP_NOSE_Z + 0.8f, AP_NOSE_Z + 1.1f, yellow);
    }
    paint_c(m, -110, 112, AP_APRON_Z1 - 0.6f, AP_APRON_Z1 - 0.3f, yellow);                      // apron edge
    const float parks[2][4] = {{10, 90, 14, 34}, {-50, -12, 14, 34}};
    for (int p = 0; p < 2; p++)                               // car-park bays
        for (float x = parks[p][0] + 0.3f; x < parks[p][1]; x += 2.6f) {
            paint_c(m, x, x + 0.12f, parks[p][2] + 0.3f, parks[p][2] + 5.3f, white);
            paint_c(m, x, x + 0.12f, parks[p][3] - 5.3f, parks[p][3] - 0.3f, white);
        }
}


static void build_building(MeshB *m, const Building *B) {
    if (B->kind == BLD_STEPS) { build_steps(m, B); return; }
    if (B->kind == BLD_WEDGE) { build_wedge(m, B); return; }
    if (B->kind == BLD_TERMINAL) { build_terminal(m, B); return; }
    if (B->kind == BLD_ATC) { build_atc(m, B); return; }
    if (B->kind == BLD_HANGAR) { build_hangar(m, B); return; }
    bseed = B->seed;
    int city = B->kind == BLD_APARTMENT || B->kind == BLD_TOWER;
    Color wall = B->kind == BLD_HOUSE ? WALLS_HOUSE[(int)(brnd() * 6)] : WALLS_CITY[(int)(brnd() * 6)];
    if (B->kind == BLD_WAREHOUSE) wall = (Color){160 + (int)(brnd() * 30), 162, 158, 255};
    if (B->kind == BLD_TOWER) wall = (Color){110, 124, 140, 255};
    Color roof = ROOFS[(int)(brnd() * 4)];
    Color glass = {48, 62, 80, 255}, glassLit = {92, 118, 140, 255}, frame = {70, 70, 72, 255};
    float H = B->mx.y - B->ground;                  // wall height above the street

    box_shell(m, B->mn, B->mx, wall, city || B->kind == BLD_SHOP ? (Color){96, 96, 100, 255} : roof);

    for (int s = 0; s < 4; s++) {
        V3 o, a, n; float len;
        facade(B, s, &o, &a, &n, &len);
        int front = s == B->front;
        switch (B->kind) {
        case BLD_TOWER: {                           // curtain wall: tall glass bands between slab edges
            int nc = (int)(len / 2.0f);
            float sp = len / nc;
            for (float y = 0.6f; y + 3.0f < H; y += 3.4f)
                for (int i = 0; i < nc; i++)
                    window(m, B, s, i * sp + 0.12f, y, sp - 0.24f, 2.6f, 0.22f, brnd() < 0.2f ? glassLit : glass);
            if (front) { facade_rect(m, B, s, len * 0.5f - 2.0f, 0.0f, 4.0f, 3.0f, 0.06f, frame); wall_lamp(m, B, s, len * 0.5f, 3.4f); }
        } break;
        case BLD_APARTMENT: case BLD_HOUSE: {
            int floors = B->kind == BLD_HOUSE ? 2 : (int)((H - 0.8f) / 3.2f);
            float fh = B->kind == BLD_HOUSE ? 2.8f : 3.2f;
            float ww = B->kind == BLD_HOUSE ? 1.1f : 1.4f, wh = B->kind == BLD_HOUSE ? 1.3f : 1.6f;
            int nc = (int)((len - 1.5f) / (B->kind == BLD_HOUSE ? 2.8f : 3.0f));
            if (nc < 1) nc = 1;
            float sp = len / nc;
            for (int f = 0; f < floors; f++)
                for (int i = 0; i < nc; i++) {
                    if (front && f == 0 && i == nc / 2) continue;           // the door goes here
                    float y = 0.9f + f * fh;
                    window(m, B, s, i * sp + (sp - ww) * 0.5f, y, ww, wh, 0.25f, brnd() < 0.15f ? glassLit : glass);
                }
            if (front) {
                float u = (nc / 2) * sp + sp * 0.5f;
                facade_rect(m, B, s, u - 0.6f, 0.0f, 1.2f, 2.2f, 0.04f, (Color){90, 60, 40, 255});
                wall_lamp(m, B, s, B->kind == BLD_HOUSE ? u + 1.1f : u, B->kind == BLD_HOUSE ? 2.3f : 3.0f);
            }
        } break;
        case BLD_SHOP:
            if (front) {                            // shopfront, sign band, door
                glowK = 0.42f; window(m, B, s, 0.8f, 0.5f, len - 1.6f, 2.5f, 0.75f, glass); glowK = 1.0f;
                wall_lamp(m, B, s, len * 0.5f, 3.3f);
                facade_rect(m, B, s, 0.4f, H - 1.4f, len - 0.8f, 0.9f, 0.06f, SIGNS[(int)(brnd() * 5)]);
                facade_rect(m, B, s, len * 0.5f - 0.8f, 0.0f, 1.6f, 2.4f, 0.07f, frame);
            } else if (len > 6) {
                for (float u = 2.0f; u + 1.2f < len - 1.0f; u += 4.0f) facade_rect(m, B, s, u, 2.0f, 1.2f, 1.0f, 0.04f, glass);
            }
            break;
        case BLD_WAREHOUSE:
            if (front)
                for (float u = 2.0f; u + 4.0f < len - 1.0f; u += 6.0f) {
                    facade_rect(m, B, s, u, 0.0f, 4.0f, 4.5f, 0.04f, (Color){58, 62, 68, 255});
                    for (float y = 0.5f; y < 4.4f; y += 0.5f) facade_rect(m, B, s, u, y, 4.0f, 0.04f, 0.05f, (Color){40, 42, 46, 255});
                    wall_lamp(m, B, s, u + 2.0f, 5.3f);
                }
            for (float u = 1.5f; u + 3.0f < len - 1.0f; u += 4.5f) facade_rect(m, B, s, u, H - 1.6f, 3.0f, 0.8f, 0.04f, glass);
            break;
        }
    }

    // Roofs.
    V3 mn = B->mn, mx = B->mx;
    if (B->roof > 0) {                              // gable, ridge along the long side, with eaves
        float e = 0.45f, wt = mx.y - 0.15f, rt = mx.y + B->roof;
        V3 ref = v3((mn.x + mx.x) * 0.5f, mx.y - 1.0f, (mn.z + mx.z) * 0.5f);
        if (mx.x - mn.x >= mx.z - mn.z) {
            float cz = (mn.z + mx.z) * 0.5f;
            mb_quad(m, v3(mn.x - e, wt, mn.z - e), v3(mx.x + e, wt, mn.z - e), v3(mx.x + e, rt, cz), v3(mn.x - e, rt, cz), roof, ref);
            mb_quad(m, v3(mn.x - e, wt, mx.z + e), v3(mx.x + e, wt, mx.z + e), v3(mx.x + e, rt, cz), v3(mn.x - e, rt, cz), roof, ref);
            mb_tri(m, v3(mn.x, mx.y, mn.z), v3(mn.x, mx.y, mx.z), v3(mn.x, rt, cz), wall, ref);
            mb_tri(m, v3(mx.x, mx.y, mn.z), v3(mx.x, mx.y, mx.z), v3(mx.x, rt, cz), wall, ref);
        } else {
            float cx = (mn.x + mx.x) * 0.5f;
            mb_quad(m, v3(mn.x - e, wt, mn.z - e), v3(mn.x - e, wt, mx.z + e), v3(cx, rt, mx.z + e), v3(cx, rt, mn.z - e), roof, ref);
            mb_quad(m, v3(mx.x + e, wt, mn.z - e), v3(mx.x + e, wt, mx.z + e), v3(cx, rt, mx.z + e), v3(cx, rt, mn.z - e), roof, ref);
            mb_tri(m, v3(mn.x, mx.y, mn.z), v3(mx.x, mx.y, mn.z), v3(cx, rt, mn.z), wall, ref);
            mb_tri(m, v3(mn.x, mx.y, mx.z), v3(mx.x, mx.y, mx.z), v3(cx, rt, mx.z), wall, ref);
        }
        if (B->kind == BLD_HOUSE && brnd() < 0.7f) {    // chimney
            float x = lerpf(mn.x + 1.5f, mx.x - 1.5f, brnd()), z = lerpf(mn.z + 1.5f, mx.z - 1.5f, brnd());
            box_shell(m, v3(x - 0.35f, mx.y, z - 0.35f), v3(x + 0.35f, rt + 0.6f, z + 0.35f), (Color){150, 80, 64, 255}, (Color){60, 50, 48, 255});
        }
    } else {                                        // flat: parapet and rooftop plant
        Color pc = {(unsigned char)(wall.r * 0.85f), (unsigned char)(wall.g * 0.85f), (unsigned char)(wall.b * 0.85f), 255};
        float t = 0.3f, py = mx.y + 0.6f;
        box_shell(m, v3(mn.x, mx.y, mn.z), v3(mx.x, py, mn.z + t), pc, pc);
        box_shell(m, v3(mn.x, mx.y, mx.z - t), v3(mx.x, py, mx.z), pc, pc);
        box_shell(m, v3(mn.x, mx.y, mn.z), v3(mn.x + t, py, mx.z), pc, pc);
        box_shell(m, v3(mx.x - t, mx.y, mn.z), v3(mx.x, py, mx.z), pc, pc);
        if (city || B->kind == BLD_WAREHOUSE) {
            int units = 1 + (int)(brnd() * 3);
            for (int i = 0; i < units; i++) {
                float x = lerpf(mn.x + 2.5f, mx.x - 4.5f, brnd()), z = lerpf(mn.z + 2.5f, mx.z - 4.5f, brnd());
                box_shell(m, v3(x, mx.y, z), v3(x + 2.0f, mx.y + 1.3f, z + 1.6f), (Color){150, 152, 156, 255}, (Color){120, 122, 126, 255});
            }
        }
    }
}

static Model build_city(const World *w, Model *glow, Model *pool) {
    MeshB m = {0};
    for (int i = 0; i < w->nbld; i++) build_building(&m, &w->bld[i]);
    if (w->map == MAP_NEWYORK) place_lamps_newyork(&m, w);
    else if (w->map == MAP_AIRPORT) build_airport_extras(&m, w);
    else place_street_lamps(&m, w);
    for (int i = 0; i < traffic_signal_count(); i++) {   // traffic signals: pole, mast arm, housing
        V3 head, face, pole; int st;
        traffic_signal(i, &head, &face, &pole, &st);
        Color metal = {48, 50, 54, 255}, housing = {30, 30, 32, 255};
        float top = pole.y + 6.0f;
        box_shell(&m, v3(pole.x - 0.1f, pole.y - 0.3f, pole.z - 0.1f), v3(pole.x + 0.1f, top, pole.z + 0.1f), metal, metal);
        box_shell(&m, v3(fminf(pole.x, head.x) - 0.05f, top - 0.25f, fminf(pole.z, head.z) - 0.05f),
                  v3(fmaxf(pole.x, head.x) + 0.05f, top - 0.15f, fmaxf(pole.z, head.z) + 0.05f), metal, metal);
        V3 side = v3(-face.z, 0, face.x);
        V3 c0 = vsub(vsub(head, vscale(side, 0.2f)), vscale(face, 0.14f)), c1 = vadd(vadd(head, vscale(side, 0.2f)), vscale(face, 0.14f));
        box_shell(&m, v3(fminf(c0.x, c1.x), head.y - 0.55f, fminf(c0.z, c1.z)), v3(fmaxf(c0.x, c1.x), head.y + 0.55f, fmaxf(c0.z, c1.z)), housing, housing);
    }
    build_screens(w);                                   // also adds the screens' street glow to POOL
    *glow = mb_model(&GLOW);
    *pool = mb_model(&POOL);
    // Crash wall, pillars and barriers are boxes too (not buildings): plain concrete.
    for (int b = 0; b < w->nbox; b++) {
        int isB = 0;
        for (int i = 0; i < w->nbld && !isB; i++) isB = w->bld[i].mn.x == w->box[b].mn.x && w->bld[i].mn.z == w->box[b].mn.z;
        if (!isB) box_shell(&m, w->box[b].mn, w->box[b].mx, (Color){175, 172, 165, 255}, (Color){190, 188, 182, 255});
    }
    return mb_model(&m);
}

// ---------------------------------------------------------------- trees ----

static Model treeModel[2], stumpModel;
#define CONIFER_H 10.0f
#define BROADLEAF_H 8.0f

static void mb_cone(MeshB *m, float y0, float y1, float r0, float r1, int sides, Color col, float jitter) {
    for (int i = 0; i < sides; i++) {
        float a0 = 6.2831853f * i / sides, a1 = 6.2831853f * (i + 1) / sides;
        V3 b0 = v3(cosf(a0) * r0, y0, sinf(a0) * r0), b1 = v3(cosf(a1) * r0, y0, sinf(a1) * r0);
        V3 t0 = v3(cosf(a0) * r1, y1, sinf(a0) * r1), t1 = v3(cosf(a1) * r1, y1, sinf(a1) * r1);
        Color c = col;
        float j = 1.0f + jitter * ((i * 37 % 11) / 11.0f - 0.5f);
        c.r = (unsigned char)fminf(255, c.r * j); c.g = (unsigned char)fminf(255, c.g * j);
        V3 ref = v3(0, (y0 + y1) * 0.5f - 0.3f, 0);
        if (r1 > 0.001f) mb_quad(m, b0, b1, t1, t0, c, ref);
        else mb_tri(m, b0, b1, t0, c, ref);
        mb_tri(m, b0, b1, v3(0, y0, 0), (Color){c.r * 6 / 10, c.g * 6 / 10, c.b * 6 / 10, 255}, v3(0, y0 + 1, 0));   // underside
    }
}

// Faceted canopy: icosahedron, squashed and jittered.
static void mb_blob(MeshB *m, V3 ctr, float rx, float ry, Color col, unsigned seed) {
    const float t = 1.618034f;
    V3 v[12] = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t}, {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
    static const int f[20][3] = {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
                                 {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
    for (int i = 0; i < 12; i++) {
        seed = seed * 1664525u + 1013904223u;
        float j = 0.85f + 0.3f * ((seed >> 8) / 16777216.0f);
        V3 d = vnorm(v[i]);
        v[i] = vadd(ctr, v3(d.x * rx * j, d.y * ry * j, d.z * rx * j));
    }
    for (int i = 0; i < 20; i++) mb_tri(m, v[f[i][0]], v[f[i][1]], v[f[i][2]], col, ctr);
}

// Prototype meshes are kept on the CPU so standing trees can be baked into one mesh per
// 50 m patch (one draw call each instead of one per tree). Fallen trees draw individually.
static MeshB treeProto[2];

static void mb_copy(MeshB *dst, const MeshB *src) {
    memset(dst, 0, sizeof *dst);
    dst->n = dst->cap = src->n;
    dst->v = MemAlloc(src->n * 3 * sizeof(float));
    dst->c = MemAlloc(src->n * 4);
    memcpy(dst->v, src->v, src->n * 3 * sizeof(float));
    memcpy(dst->c, src->c, src->n * 4);
}

static void build_trees(void) {
    MeshB m = {0};
    Color bark = {92, 66, 46, 255};
    if (treeProto[0].v) { MemFree(treeProto[0].v); MemFree(treeProto[0].c); MemFree(treeProto[1].v); MemFree(treeProto[1].c); }
    mb_cone(&m, 0, 3.0f, 0.25f, 0.18f, 6, bark, 0);                                   // conifer
    mb_cone(&m, 1.6f, 6.0f, 2.6f, 0.0f, 9, (Color){46, 92, 52, 255}, 0.25f);
    mb_cone(&m, 3.8f, 8.2f, 2.0f, 0.0f, 9, (Color){52, 102, 56, 255}, 0.25f);
    mb_cone(&m, 6.0f, CONIFER_H, 1.3f, 0.0f, 8, (Color){60, 112, 60, 255}, 0.25f);
    mb_copy(&treeProto[TREE_CONIFER], &m);
    treeModel[TREE_CONIFER] = mb_model(&m);
    mb_cone(&m, 0, 3.6f, 0.22f, 0.16f, 6, bark, 0);                                   // broadleaf
    mb_blob(&m, v3(0, 5.2f, 0), 2.6f, 2.2f, (Color){84, 132, 58, 255}, 7);
    mb_blob(&m, v3(0.9f, 6.3f, 0.5f), 1.6f, 1.4f, (Color){96, 146, 64, 255}, 11);
    mb_copy(&treeProto[TREE_BROADLEAF], &m);
    treeModel[TREE_BROADLEAF] = mb_model(&m);
    mb_cone(&m, -0.3f, 0.5f, 1.0f, 0.85f, 7, bark, 0);                               // stump (scaled by trunk radius)
    mb_cone(&m, 0.5f, 0.5f, 0.85f, 0.0f, 7, (Color){200, 170, 120, 255}, 0);
    stumpModel = mb_model(&m);
}

#define TCH 12                                  // tree patches per side (50 m each)
#define TCH_SIZE 50.0f
static Model treePatch[TCH * TCH], shadowPatch[TCH * TCH];
static unsigned char patchHas[TCH * TCH], treeBaked[MAX_TREES];

static int tree_patch(const Tree *t) {
    int gx = (int)((t->x + TCH * TCH_SIZE * 0.5f) / TCH_SIZE), gz = (int)((t->z + TCH * TCH_SIZE * 0.5f) / TCH_SIZE);
    gx = gx < 0 ? 0 : (gx >= TCH ? TCH - 1 : gx); gz = gz < 0 ? 0 : (gz >= TCH ? TCH - 1 : gz);
    return gz * TCH + gx;
}

static Color tree_tint(int i) { unsigned char t = (unsigned char)(215 + (i * 97 % 40)); return (Color){t, t, (unsigned char)(t - 10), 255}; }

static void build_tree_patch(const World *w, int pi) {
    if (patchHas[pi]) { UnloadModel(treePatch[pi]); UnloadModel(shadowPatch[pi]); patchHas[pi] = 0; }
    MeshB m = {0}, sh = {0};
    unsigned char sa = (unsigned char)(55 * fminf(1.0f, 0.35f + SUN.y));
    for (int i = 0; i < w->ntree; i++) {
        const Tree *t = &w->tree[i];
        if (tree_patch(t) != pi) continue;
        treeBaked[i] = !t->broken;
        if (t->broken) continue;
        const MeshB *P = &treeProto[t->kind];
        float s = t->h / (t->kind == TREE_CONIFER ? CONIFER_H : BROADLEAF_H);
        float a = (float)(i * 47 % 360) * DEG2RAD, ca = cosf(a), sn = sinf(a);
        Color tint = tree_tint(i);
        for (int k = 0; k < P->n; k++) {
            float x = P->v[k * 3] * s, y = P->v[k * 3 + 1] * s, z = P->v[k * 3 + 2] * s;
            V3 p = v3(t->x + x * ca + z * sn, t->y + y, t->z - x * sn + z * ca);
            Color c = {(unsigned char)(P->c[k * 4] * tint.r / 255), (unsigned char)(P->c[k * 4 + 1] * tint.g / 255), (unsigned char)(P->c[k * 4 + 2] * tint.b / 255), 255};
            mb_push(&m, p, c);
        }
        // Shadow disc, offset away from the sun and laid on the local slope.
        float R = t->h * (t->kind == TREE_CONIFER ? 0.24f : 0.33f);
        V3 c = v3(t->x - SUN.x * t->h * 0.25f, 0, t->z - SUN.z * t->h * 0.25f);
        c.y = terrain_height(c.x, c.z) + 0.05f;
        V3 n = terrain_normal(c.x, c.z);
        Color col = {0, 0, 0, 255};
        V3 prev = {0};
        for (int k = 0; k <= 8; k++) {
            float an = 6.2831853f * k / 8;
            V3 p = v3(c.x + cosf(an) * R, 0, c.z + sinf(an) * R);
            p.y = c.y - (n.x * (p.x - c.x) + n.z * (p.z - c.z)) / n.y;
            if (k) { mb_push(&sh, c, col); mb_push(&sh, p, col); mb_push(&sh, prev, col); }
            prev = p;
        }
    }
    if (!m.n) { MemFree(m.v); MemFree(m.c); MemFree(sh.v); MemFree(sh.c); return; }
    for (int k = 0; k < sh.n; k++) sh.c[k * 4 + 3] = sa;
    treePatch[pi] = mb_model(&m);
    shadowPatch[pi] = mb_model(&sh);
    patchHas[pi] = 1;
}

static void build_tree_patches(const World *w) { for (int i = 0; i < TCH * TCH; i++) build_tree_patch(w, i); }

// Rebuild any patch whose baked trees have since been knocked down.
static void refresh_tree_patches(const World *w) {
    static int lastDown = -1;
    if (w->treesDown == lastDown) return;
    lastDown = w->treesDown;
    for (int i = 0; i < w->ntree; i++)
        if (w->tree[i].broken && treeBaked[i]) build_tree_patch(w, tree_patch(&w->tree[i]));
}

static void draw_trees(const World *w, Vector3 eye) {
    for (int i = 0; i < TCH * TCH; i++) if (patchHas[i]) DrawModel(treePatch[i], (Vector3){0, 0, 0}, 1.0f, WHITE);
    for (int i = 0; i < w->ntree; i++) {
        const Tree *t = &w->tree[i];
        if (!t->broken) continue;
        float dx = t->x - eye.x, dz = t->z - eye.z;
        if (dx * dx + dz * dz > 300.0f * 300.0f) continue;
        float s = t->h / (t->kind == TREE_CONIFER ? CONIFER_H : BROADLEAF_H);
        V3 axis = vcross(v3(0, 1, 0), t->fall);
        Vector3 base = {t->x, t->y, t->z};
        Vector3 top = {t->x + t->fall.x * 0.5f * t->r, t->y + 0.5f * s, t->z + t->fall.z * 0.5f * t->r};
        DrawModelEx(treeModel[t->kind], top, RV(axis), t->tilt * RAD2DEG, (Vector3){s, s, s}, tree_tint(i));
        float sr = t->r / 0.25f * (t->kind == TREE_CONIFER ? 1.0f : 0.25f / 0.22f);
        DrawModelEx(stumpModel, base, (Vector3){0, 1, 0}, 0, (Vector3){sr * 0.25f, 1, sr * 0.25f}, WHITE);
    }
}

static void draw_tree_shadows(void) {
    if (post_shadow_active()) return;
    for (int i = 0; i < TCH * TCH; i++) if (patchHas[i]) DrawModel(shadowPatch[i], (Vector3){0, 0, 0}, 1.0f, WHITE);
}

// ---------------------------------------------------------- skid marks ----

#define MAX_SKID 6000
typedef struct { Vector3 a, b, c, d; } Skid;
static Skid skids[MAX_SKID];
static int skidCount, skidHead;
static V3 skidLast[MAX_CARS][MAX_WHEELS];
static int skidHas[MAX_CARS][MAX_WHEELS];

static int snowTracks;                          // set while snowing: all rolling wheels leave tracks

static void skids_forget(int ci) { for (int i = 0; i < MAX_WHEELS; i++) skidHas[ci][i] = 0; }

static void update_skids(World *w) {
    for (int ci = 0; ci < w->ncars; ci++) {
        Car *c = &w->cars[ci];
        for (int i = 0; i < c->nw; i++) {
            Wheel *W = &c->w[i];
            int track = snowTracks && vlen(c->vel) > 0.8f;
            if (!c->active || !W->contact || (W->slip < 0.12f && !track)) { skidHas[ci][i] = 0; continue; }
            V3 n = terrain_normal(W->cp.x, W->cp.z);
            V3 cur = vadd(W->cp, vscale(n, 0.03f));
            if (skidHas[ci][i]) {
                float gap = vlen(vsub(cur, skidLast[ci][i]));
                if (gap < 0.3f) continue;
                if (gap > 2.0f) { skidLast[ci][i] = cur; continue; }   // car was rebuilt or teleported: start a new mark
                V3 s = vscale(W->side, 0.11f), l = skidLast[ci][i];
                skids[skidHead] = (Skid){RV(vsub(l, s)), RV(vadd(l, s)), RV(vadd(cur, s)), RV(vsub(cur, s))};
                skidHead = (skidHead + 1) % MAX_SKID;
                if (skidCount < MAX_SKID) skidCount++;
            }
            skidLast[ci][i] = cur; skidHas[ci][i] = 1;
        }
    }
}

static void draw_skids(void) {
    Color col = snowTracks ? (Color){70, 76, 88, 150} : (Color){20, 20, 20, 150};
    for (int i = 0; i < skidCount; i++) {
        Skid *s = &skids[i];
        DrawTriangle3D(s->a, s->c, s->b, col);
        DrawTriangle3D(s->a, s->d, s->c, col);
    }
}

// ----------------------------------------------------------------- car ----

static Color brighten(Color c, float s) {
    return (Color){(unsigned char)fminf(255, c.r + s * 255), (unsigned char)fminf(255, c.g + s * 255), (unsigned char)fminf(255, c.b + s * 255), c.a};
}

// Emissive strip along an edge: top L/R nodes q[0..1], fading toward q[2..3] by `t`.
// A light: the stretch [u0, u1] of the edge q0-q1, reaching fraction t of the way toward edge q2-q3.
static void draw_lightbar(const Car *c, const LightStrip *L, Color col) {
    V3 e0 = c->n[L->q[0]].p, e1 = c->n[L->q[1]].p, f0 = c->n[L->q[2]].p, f1 = c->n[L->q[3]].p;
    V3 a = vlerp(e0, e1, L->u0), b = vlerp(e0, e1, L->u1);
    V3 al = vlerp(a, vlerp(f0, f1, L->u0), L->t), bl = vlerp(b, vlerp(f0, f1, L->u1), L->t);
    V3 off = vscale(vnorm(vcross(vsub(e1, e0), vsub(al, a))), L->out * 0.008f);
    a = vadd(a, off); b = vadd(b, off); al = vadd(al, off); bl = vadd(bl, off);
    DrawTriangle3D(RV(a), RV(al), RV(bl), col);
    DrawTriangle3D(RV(a), RV(bl), RV(b), col);
}

// ---- materials for the paint shader
enum { MAT_UNLIT = 255, MAT_PAINT = 250, MAT_FLAME = 245, MAT_CHROME = 240, MAT_GLASS = 235, MAT_SATIN = 230, MAT_MATTE = 225, MAT_STEEL = 220, MAT_GLOSS = 215 };
static int pbrOn;                               // drawing between post_paint_begin/end
static int depthOnly;                           // drawing the shadow map: skip decals
static Color pbr(Color base, V3 n, int mat) { rlNormal3f(n.x, n.y, n.z); base.a = (unsigned char)mat; return base; }

// Paint shading for one point. Under the paint shader this just hands over the normal and a material;
// otherwise (CPU fallback) baked-style sun/ambient, a sky reflection toward grazing angles, a highlight.
static Color shade_paint(Color base, V3 n, V3 p, V3 eye, float gloss) {
    if (pbrOn) return pbr(base, n, gloss >= 0.95f ? MAT_PAINT : gloss >= 0.5f ? MAT_GLOSS : gloss > 0.05f ? MAT_SATIN : MAT_MATTE);
    if (depthOnly) return base;                 // shadow map: colour is irrelevant
    Color col = lit(base, n, 1.0f);
    V3 v = vnorm(vsub(eye, p));
    float fres = 0.10f + 0.55f * powf(1.0f - fmaxf(vdot(n, v), 0.0f), 3.0f);
    float up = fmaxf(n.y, 0.0f);
    V3 sk = vlerp(v3(ENV->horizonAway.x, ENV->horizonAway.y, ENV->horizonAway.z), v3(ENV->zenith.x, ENV->zenith.y, ENV->zenith.z), up);
    Color skc = {(unsigned char)fminf(255, 255 * powf(sk.x * 1.6f, 0.4545f)), (unsigned char)fminf(255, 255 * powf(sk.y * 1.6f, 0.4545f)), (unsigned char)fminf(255, 255 * powf(sk.z * 1.6f, 0.4545f)), 255};
    float f = fres * gloss;
    col = (Color){(unsigned char)(col.r + (skc.r - col.r) * f), (unsigned char)(col.g + (skc.g - col.g) * f), (unsigned char)(col.b + (skc.b - col.b) * f), 255};
    V3 hv = vnorm(vadd(SUN, v));
    float spec = powf(fmaxf(vdot(n, hv), 0.0f), 60.0f) * 0.6f * gloss + powf(fmaxf(vdot(n, hv), 0.0f), 8.0f) * 0.08f;
    return brighten(col, spec);
}

// An oriented box (centre, three half-axis vectors), lit per face.
static float oboxGloss = 0.6f;                  // set to 0 for matte cabin trim
static void draw_obox(V3 c, V3 ax, V3 ay, V3 az, Color col, V3 eye) {
    V3 axes[3] = {ax, ay, az};
    for (int a = 0; a < 3; a++)
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            V3 n = vscale(axes[a], (float)sgn), u = axes[(a + 1) % 3], w = axes[(a + 2) % 3];
            V3 fc = vadd(c, n);
            V3 q0 = vsub(vsub(fc, u), w), q1 = vsub(vadd(fc, u), w), q2 = vadd(vadd(fc, u), w), q3 = vadd(vsub(fc, u), w);
            Color k = oboxGloss > 0 ? shade_paint(col, vnorm(n), fc, eye, oboxGloss) : pbrOn ? pbr(col, vnorm(n), MAT_MATTE) : lit(col, vnorm(n), 1.15f);
            DrawTriangle3D(RV(q0), RV(q1), RV(q2), k); DrawTriangle3D(RV(q0), RV(q2), RV(q3), k);
        }
}

// A triangle with one normal and material.
static void ptri(V3 a, V3 b, V3 d, V3 n, Color col, int mat) {
    Color k = pbrOn ? pbr(col, n, mat) : col;
    DrawTriangle3D(RV(a), RV(b), RV(d), k);
}
// A box given its centre and three half-axis vectors.
static void pbox(V3 c, V3 ax, V3 ay, V3 az, Color col, int mat) {
    V3 axes[3] = {ax, ay, az};
    for (int a = 0; a < 3; a++)
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            V3 n = vscale(axes[a], (float)sgn), u = axes[(a + 1) % 3], w = axes[(a + 2) % 3];
            V3 fc = vadd(c, n), nn = vnorm(n);
            V3 q0 = vsub(vsub(fc, u), w), q1 = vsub(vadd(fc, u), w), q2 = vadd(vadd(fc, u), w), q3 = vadd(vsub(fc, u), w);
            ptri(q0, q1, q2, nn, col, mat); ptri(q0, q2, q3, nn, col, mat);
        }
}
// A cylinder or cone frustum from a (radius r0) to b (radius r1), smooth-shaded; caps: 1 = at a, 2 = at b.
static void pcyl(V3 a, V3 b, float r0, float r1, int sides, Color col, int mat, int caps) {
    V3 ax = vnorm(vsub(b, a));
    V3 u = vnorm(vcross(ax, fabsf(ax.y) < 0.9f ? v3(0, 1, 0) : v3(1, 0, 0))), w = vcross(ax, u);
    float len = vlen(vsub(b, a)), slope = len > 1e-5f ? (r0 - r1) / len : 0.0f;
    if (len < 0.004f) slope = r0 > r1 ? 50.0f : -50.0f;          // a flat ring: normal along the axis
    unsigned char al = (unsigned char)(pbrOn ? mat : 255);
    rlBegin(RL_TRIANGLES);
    rlColor4ub(col.r, col.g, col.b, al);
    for (int i = 0; i < sides; i++) {
        float a0 = i * 6.2831853f / sides, a1 = (i + 1) * 6.2831853f / sides;
        V3 d0 = vadd(vscale(u, cosf(a0)), vscale(w, sinf(a0))), d1 = vadd(vscale(u, cosf(a1)), vscale(w, sinf(a1)));
        V3 n0 = vnorm(vadd(d0, vscale(ax, slope))), n1 = vnorm(vadd(d1, vscale(ax, slope)));
        V3 p0 = vadd(a, vscale(d0, r0)), p1 = vadd(a, vscale(d1, r0)), q0 = vadd(b, vscale(d0, r1)), q1 = vadd(b, vscale(d1, r1));
        rlNormal3f(n0.x, n0.y, n0.z); rlVertex3f(p0.x, p0.y, p0.z); rlVertex3f(q0.x, q0.y, q0.z);
        rlNormal3f(n1.x, n1.y, n1.z); rlVertex3f(q1.x, q1.y, q1.z);
        rlNormal3f(n0.x, n0.y, n0.z); rlVertex3f(p0.x, p0.y, p0.z);
        rlNormal3f(n1.x, n1.y, n1.z); rlVertex3f(q1.x, q1.y, q1.z); rlVertex3f(p1.x, p1.y, p1.z);
        if (caps & 1) { rlNormal3f(-ax.x, -ax.y, -ax.z); rlVertex3f(a.x, a.y, a.z); rlVertex3f(p1.x, p1.y, p1.z); rlVertex3f(p0.x, p0.y, p0.z); }
        if (caps & 2) { rlNormal3f(ax.x, ax.y, ax.z); rlVertex3f(b.x, b.y, b.z); rlVertex3f(q0.x, q0.y, q0.z); rlVertex3f(q1.x, q1.y, q1.z); }
    }
    rlEnd();
}

// ---- render-only parts carried by body nodes
#define RN(r, p) ((r) * 9 + (p))                // body node: ring r, perimeter point p (C FL RKL BLL TPL TPR BLR RKR FR)
static V3 part(const Car *c, int node, float x, float y, float z) {   // a car-local point, moved with a nearby node
    V3 d = vsub(v3(x, y, z), c->rest[node]);
    return vadd(c->n[node].p, vadd(vadd(vscale(c->fwd, d.x), vscale(c->up, d.y)), vscale(c->right, d.z)));
}
static void lbox(const Car *c, int node, float x0, float x1, float y0, float y1, float z0, float z1, Color col, int mat) {
    pbox(part(c, node, 0.5f * (x0 + x1), 0.5f * (y0 + y1), 0.5f * (z0 + z1)), vscale(c->fwd, 0.5f * (x1 - x0)), vscale(c->up, 0.5f * (y1 - y0)), vscale(c->right, 0.5f * (z1 - z0)), col, mat);
}

static const Color CHROME = {236, 238, 242, 255}, FLAME_RED = {190, 22, 14, 255};

// The semi's brightwork and fenders: bumper, grille, lamps, air cleaners, stacks, tanks, visor, mirrors.
static void draw_semi_parts(const Car *c) {
    const Color dark = {20, 20, 22, 255}, steel = {70, 72, 78, 255};
    const float NOSE = c->def->st[0].x;
    Color blue = {(unsigned char)(c->color.x * 255), (unsigned char)(c->color.y * 255), (unsigned char)(c->color.z * 255), 255};
    int n0 = RN(0, 0);
    // Bumper: a deep chrome blade across the full width, with a lower lip.
    lbox(c, n0, 3.84f, 4.02f, -0.14f, 0.34f, -1.23f, 1.23f, CHROME, MAT_CHROME);
    lbox(c, n0, 3.80f, 3.98f, -0.20f, -0.14f, -1.18f, 1.18f, CHROME, MAT_CHROME);
    // Grille: dark core behind vertical chrome bars, inside the chrome shell (the nose face).
    lbox(c, n0, 3.800f, 3.812f, 0.54f, 1.30f, -0.50f, 0.50f, dark, MAT_MATTE);
    for (int i = 0; i < 15; i++) {
        float z = -0.47f + i * (0.94f / 14.0f);
        lbox(c, n0, 3.806f, 3.832f, 0.55f, 1.29f, z - 0.013f, z + 0.013f, CHROME, MAT_CHROME);
    }
    lbox(c, n0, 3.80f, 3.84f, 1.30f, 1.36f, -0.56f, 0.56f, CHROME, MAT_CHROME);     // shell header and sill
    lbox(c, n0, 3.80f, 3.84f, 0.48f, 0.54f, -0.60f, 0.60f, CHROME, MAT_CHROME);
    for (int sd = -1; sd <= 1; sd += 2) {
        float z = (float)sd;
        int rk0 = RN(0, sd < 0 ? 2 : 7), ax = RN(3, sd < 0 ? 2 : 7), cowl = RN(6, sd < 0 ? 3 : 6);
        int door = RN(8, sd < 0 ? 3 : 6), cabR = RN(10, sd < 0 ? 3 : 6), cabRT = RN(10, sd < 0 ? 4 : 5), tankN = RN(9, sd < 0 ? 2 : 7);
        // Twin-lamp headlight pod on a bracket beside the grille.
        lbox(c, rk0, 3.60f, 3.80f, 0.60f, 0.86f, z * 0.68f, z * 1.14f, CHROME, MAT_CHROME);
        lbox(c, rk0, 3.50f, 3.62f, 0.66f, 0.80f, z * 0.60f, z * 0.72f, steel, MAT_SATIN);
        // Front fender: an arc over the tyre, flame-painted.
        {
            const float cx = 2.95f, cy = -0.02f, R = 0.70f, zi = 0.63f, zo = 1.25f;
            V3 prevI = {0}, prevO = {0}, prevL = {0};
            for (int k = 0; k <= 14; k++) {
                float a = (-22.0f + k * (222.0f / 14.0f)) * DEG2RAD;
                float x = cx + R * cosf(a), y = cy + R * sinf(a);
                V3 pi = part(c, ax, x, y, z * zi), po = part(c, ax, x, y, z * zo);
                V3 pl = part(c, ax, cx + (R - 0.09f) * cosf(a), cy + (R - 0.09f) * sinf(a), z * (zo + 0.005f));
                if (k) {
                    V3 n = vnorm(vadd(vscale(c->fwd, cosf(a)), vscale(c->up, sinf(a))));
                    Color col = pbrOn ? pbr(FLAME_RED, n, MAT_FLAME) : FLAME_RED;
                    rlBegin(RL_TRIANGLES);
                    rlColor4ub(col.r, col.g, col.b, col.a);
                    #define FV(p, zz, xx, yy) rlTexCoord2f(NOSE - (xx) + 0.9f, (yy) + 1.3f - (zz)); rlVertex3f((p).x, (p).y, (p).z)
                    float xp = cx + R * cosf(a - 222.0f / 14.0f * DEG2RAD), yp = cy + R * sinf(a - 222.0f / 14.0f * DEG2RAD);
                    FV(prevI, zi, xp, yp); FV(prevO, zo, xp, yp); FV(po, zo, x, y);
                    FV(prevI, zi, xp, yp); FV(po, zo, x, y); FV(pi, zi, x, y);
                    V3 no = vscale(c->right, z);
                    rlNormal3f(no.x, no.y, no.z);                           // outer lip
                    FV(prevO, zo, xp, yp); FV(prevL, zo + 0.3f, xp, yp); FV(pl, zo + 0.3f, x, y);
                    FV(prevO, zo, xp, yp); FV(pl, zo + 0.3f, x, y); FV(po, zo, x, y);
                    #undef FV
                    rlEnd();
                }
                prevI = pi; prevO = po; prevL = pl;
            }
        }
        // Air cleaner on the cowl.
        pcyl(part(c, cowl, 1.88f, 0.78f, z * 0.90f), part(c, cowl, 1.88f, 1.50f, z * 0.90f), 0.17f, 0.17f, 18, CHROME, MAT_CHROME, 0);
        pcyl(part(c, cowl, 1.88f, 1.50f, z * 0.90f), part(c, cowl, 1.88f, 1.58f, z * 0.90f), 0.17f, 0.09f, 18, CHROME, MAT_CHROME, 2);
        pcyl(part(c, cowl, 1.88f, 0.70f, z * 0.90f), part(c, cowl, 1.88f, 0.78f, z * 0.90f), 0.10f, 0.17f, 18, CHROME, MAT_CHROME, 1);
        lbox(c, cowl, 1.80f, 1.96f, 1.05f, 1.15f, z * 0.62f, z * 0.76f, steel, MAT_SATIN);
        // Exhaust stack: heat-shielded lower pipe, slim upper pipe; bends with the cab.
        V3 s0 = part(c, cabR, 0.20f, 0.45f, z * 1.14f), s1 = part(c, cabR, 0.20f, 1.95f, z * 1.14f), s2 = part(c, cabRT, 0.20f, 3.42f, z * 1.14f);
        pcyl(s0, s1, 0.105f, 0.105f, 16, CHROME, MAT_CHROME, 0);
        pcyl(s1, vadd(s1, vscale(vnorm(vsub(s2, s1)), 0.08f)), 0.105f, 0.07f, 16, CHROME, MAT_CHROME, 0);
        pcyl(s1, s2, 0.07f, 0.07f, 14, CHROME, MAT_CHROME, 0);
        pcyl(vadd(s2, vscale(c->up, -0.002f)), s2, 0.062f, 0.062f, 14, dark, MAT_MATTE, 2);
        lbox(c, cabR, 0.14f, 0.26f, 1.20f, 1.26f, z * 1.03f, z * 1.12f, steel, MAT_SATIN);
        // Fuel tank under the cab with its straps and a step; toolbox behind it.
        pcyl(part(c, tankN, 1.28f, 0.02f, z * 0.93f), part(c, tankN, -0.50f, 0.02f, z * 0.93f), 0.33f, 0.33f, 22, CHROME, MAT_CHROME, 3);
        for (int k = 0; k < 2; k++) {
            float x = 0.95f - k * 1.10f;
            pcyl(part(c, tankN, x + 0.03f, 0.02f, z * 0.93f), part(c, tankN, x - 0.03f, 0.02f, z * 0.93f), 0.337f, 0.337f, 22, steel, MAT_SATIN, 0);
        }
        lbox(c, tankN, 0.55f, 1.25f, 0.355f, 0.385f, z * 0.96f, z * 1.24f, CHROME, MAT_STEEL);
        lbox(c, RN(11, sd < 0 ? 2 : 7), -1.38f, -0.62f, -0.26f, 0.32f, z * 0.62f, z * 1.20f, CHROME, MAT_STEEL);
        // Door mirror on a tube bracket.
        V3 m0 = part(c, door, 1.40f, 1.56f, z * 1.06f), m1 = part(c, door, 1.42f, 1.74f, z * 1.36f), m2 = part(c, door, 1.42f, 2.18f, z * 1.36f), m3 = part(c, door, 1.40f, 2.30f, z * 1.02f);
        pcyl(m0, m1, 0.014f, 0.014f, 6, CHROME, MAT_CHROME, 0); pcyl(m1, m2, 0.014f, 0.014f, 6, CHROME, MAT_CHROME, 0); pcyl(m2, m3, 0.014f, 0.014f, 6, CHROME, MAT_CHROME, 0);
        lbox(c, door, 1.395f, 1.445f, 1.76f, 2.16f, z * 1.37f, z * 1.55f, CHROME, MAT_CHROME);
        // Rear fender over the tandem: flat top with rolled ends, painted red.
        {
            static const float PX[8] = {-2.02f, -2.08f, -2.24f, -2.50f, -4.30f, -4.56f, -4.72f, -4.78f};
            static const float PY[8] = {0.05f, 0.36f, 0.56f, 0.64f, 0.64f, 0.56f, 0.36f, 0.05f};
            int fn = RN(17, sd < 0 ? 3 : 6);
            for (int k = 0; k + 1 < 8; k++) {
                V3 a = part(c, fn, PX[k], PY[k], z * 0.50f), b = part(c, fn, PX[k], PY[k], z * 1.26f);
                V3 a2 = part(c, fn, PX[k + 1], PY[k + 1], z * 0.50f), b2 = part(c, fn, PX[k + 1], PY[k + 1], z * 1.26f);
                V3 n = vnorm(vcross(vsub(a2, a), vsub(b, a)));
                if (vdot(n, c->up) < 0 && k > 0 && k < 6) n = vscale(n, -1);
                rlTexCoord2f(0.3f, 1.0f + 0.2f * k);
                ptri(a, b, b2, n, FLAME_RED, MAT_PAINT); ptri(a, b2, a2, n, FLAME_RED, MAT_PAINT);
                V3 l = part(c, fn, PX[k], PY[k] - 0.10f, z * 1.262f), l2 = part(c, fn, PX[k + 1], PY[k + 1] - 0.10f, z * 1.262f);
                V3 no = vscale(c->right, z);
                ptri(b, l, l2, no, FLAME_RED, MAT_PAINT); ptri(b, l2, b2, no, FLAME_RED, MAT_PAINT);
            }
            lbox(c, RN(19, sd < 0 ? 2 : 7), -4.82f, -4.80f, -0.42f, 0.10f, z * 0.60f, z * 1.22f, dark, MAT_MATTE);      // mud flap
        }
        // Air horn on the roof.
        pcyl(part(c, RN(8, sd < 0 ? 4 : 5), 0.95f, 2.53f, z * 0.34f), part(c, RN(8, sd < 0 ? 4 : 5), 1.42f, 2.55f, z * 0.34f), 0.025f, 0.065f, 10, CHROME, MAT_CHROME, 0);
    }
    // Sun visor over the windshield, raked down.
    {
        V3 f = vnorm(vadd(vscale(c->fwd, 0.90f), vscale(c->up, -0.42f))), u = vnorm(vcross(c->right, f));
        V3 ctr = vscale(vadd(part(c, RN(8, 4), 1.60f, 2.37f, -0.5f), part(c, RN(8, 5), 1.60f, 2.37f, 0.5f)), 0.5f);
        pbox(ctr, vscale(f, 0.19f), vscale(u, 0.018f), vscale(c->right, 0.99f), CHROME, MAT_CHROME);
    }
    // Roof marker lamp bases.
    for (int k = 0; k < 5; k++) lbox(c, RN(8, k < 2 ? 4 : 5), 1.20f, 1.34f, 2.44f, 2.50f, -0.76f + k * 0.38f - 0.04f, -0.76f + k * 0.38f + 0.04f, CHROME, MAT_CHROME);
    // Windshield centre post.
    pcyl(part(c, RN(7, 4), 1.555f, 1.58f, 0), part(c, RN(8, 4), 1.365f, 2.42f, 0), 0.022f, 0.022f, 6, blue, MAT_PAINT, 0);
    // Fifth wheel on the frame deck.
    pcyl(part(c, RN(17, 0), -3.40f, 0.46f, 0), part(c, RN(17, 0), -3.40f, 0.56f, 0), 0.50f, 0.50f, 24, steel, MAT_SATIN, 2);
    lbox(c, RN(17, 0), -4.05f, -3.70f, 0.47f, 0.53f, -0.30f, 0.30f, steel, MAT_SATIN);
}

// One wheel: tyre with rounded shoulders, then the wheel face in the vehicle's style.
static void draw_wheel(const Car *c, int i) {
    const Wheel *W = &c->w[i];
    const VehicleDef *D = c->def;
    const Color rubber = {30, 30, 32, 255};
    float R = D->wheelR, wid = (i >= 2 && D->wheelWR > 0) ? D->wheelWR : D->wheelW;
    V3 axis = vrot(c->right, c->up, W->steerAngle);
    float out = (i % 2 == 0) ? -1.0f : 1.0f;                  // left wheels sit at -right
    V3 n = vscale(axis, out), ctr = W->hub;
    V3 a = vsub(ctr, vscale(n, wid * 0.5f)), b = vadd(ctr, vscale(n, wid * 0.5f));
    float sh = fminf(0.05f, wid * 0.2f);
    float rimR = R * (D->wheelStyle == 2 ? 0.58f : D->wheelStyle == 1 ? 0.74f : 0.68f);
    int dual = D->wheelStyle == 2 && i >= 2;
    pcyl(a, vadd(a, vscale(n, sh)), R * 0.92f, R, 26, rubber, MAT_MATTE, 1);
    if (dual) {                                               // two tyres with a groove between them
        float tw = (wid - 0.05f) * 0.5f;
        pcyl(vadd(a, vscale(n, sh)), vadd(a, vscale(n, tw)), R, R, 26, rubber, MAT_MATTE, 0);
        pcyl(vadd(a, vscale(n, tw)), vadd(a, vscale(n, tw + 0.05f)), R * 0.86f, R * 0.86f, 26, (Color){12, 12, 13, 255}, MAT_MATTE, 0);
        pcyl(vadd(a, vscale(n, tw)), vadd(a, vscale(n, tw + 0.001f)), R, R * 0.86f, 26, rubber, MAT_MATTE, 0);
        pcyl(vadd(a, vscale(n, tw + 0.049f)), vadd(a, vscale(n, tw + 0.05f)), R * 0.86f, R, 26, rubber, MAT_MATTE, 0);
        pcyl(vadd(a, vscale(n, tw + 0.05f)), vsub(b, vscale(n, sh)), R, R, 26, rubber, MAT_MATTE, 0);
    } else pcyl(vadd(a, vscale(n, sh)), vsub(b, vscale(n, sh)), R, R, 26, rubber, MAT_MATTE, 0);
    pcyl(vsub(b, vscale(n, sh)), b, R, R * 0.92f, 26, rubber, MAT_MATTE, 0);
    pcyl(b, vadd(b, vscale(n, 0.001f)), R * 0.92f, rimR, 26, rubber, MAT_MATTE, 0);       // sidewall
    V3 fw = vcross(c->up, axis);
    if (D->wheelStyle == 2) {                                 // polished truck wheels
        if (dual) {                                           // deep dish with a hub in the bottom
            V3 in = vsub(b, vscale(n, 0.16f));
            pcyl(in, b, rimR * 0.50f, rimR, 26, CHROME, MAT_CHROME, 0);
            pcyl(in, vadd(in, vscale(n, 0.001f)), rimR * 0.50f, 0.0f, 20, CHROME, MAT_STEEL, 0);
            pcyl(in, vadd(in, vscale(n, 0.13f)), R * 0.20f, R * 0.14f, 14, CHROME, MAT_CHROME, 2);
            for (int k = 0; k < 10; k++) {
                V3 d = vrot(fw, axis, W->spin + k * 0.628318f), p = vadd(in, vscale(d, rimR * 0.40f));
                pcyl(p, vadd(p, vscale(n, 0.035f)), 0.016f, 0.016f, 6, CHROME, MAT_CHROME, 2);
            }
        } else {                                              // front: shallow dish, domed hub cap
            V3 f1 = vadd(b, vscale(n, 0.035f));
            pcyl(b, f1, rimR, rimR * 0.82f, 26, CHROME, MAT_CHROME, 0);
            pcyl(f1, vsub(f1, vscale(n, 0.03f)), rimR * 0.82f, rimR * 0.40f, 26, CHROME, MAT_CHROME, 0);
            pcyl(vsub(f1, vscale(n, 0.03f)), vadd(f1, vscale(n, 0.09f)), R * 0.24f, R * 0.13f, 16, CHROME, MAT_CHROME, 2);
            for (int k = 0; k < 10; k++) {
                V3 d = vrot(fw, axis, W->spin + k * 0.628318f), p = vadd(vsub(f1, vscale(n, 0.02f)), vscale(d, rimR * 0.56f));
                pcyl(p, vadd(p, vscale(n, 0.035f)), 0.016f, 0.016f, 6, CHROME, MAT_CHROME, 2);
            }
            for (int k = 0; k < 5; k++) {                     // hand holes show the wheel turning
                V3 d = vrot(fw, axis, W->spin + 0.3f + k * 1.25664f), p = vadd(vsub(f1, vscale(n, 0.012f)), vscale(d, rimR * 0.70f));
                pcyl(p, vadd(p, vscale(n, 0.002f)), 0.035f, 0.035f, 8, (Color){14, 14, 16, 255}, MAT_MATTE, 2);
            }
        }
        return;
    }
    if (D->wheelStyle == 1) {                                 // dark alloy: barrel, ten spokes, lip, cap
        V3 f1 = vadd(b, vscale(n, 0.002f)), f2 = vadd(b, vscale(n, 0.007f)), f3 = vadd(b, vscale(n, 0.014f));
        pcyl(b, f1, rimR, rimR, 28, (Color){96, 98, 104, 255}, MAT_STEEL, 2);
        pcyl(f1, vadd(f1, vscale(n, 0.001f)), rimR * 0.93f, rimR * 0.93f, 28, (Color){12, 12, 14, 255}, MAT_MATTE, 2);
        V3 nn = vnorm(n);
        for (int k = 0; k < 10; k++) {
            float ang = W->spin + (k >> 1) * 1.25664f + (k & 1 ? 0.16f : -0.16f);
            V3 d = vrot(fw, axis, ang), sd = vcross(axis, d);
            float w0 = R * 0.055f, w1 = R * 0.030f;
            V3 r0 = vadd(f2, vscale(d, R * 0.13f)), r1 = vadd(f2, vscale(d, R * 0.69f));
            V3 q0 = vsub(r0, vscale(sd, w0)), q1 = vadd(r0, vscale(sd, w0)), q2 = vadd(r1, vscale(sd, w1)), q3 = vsub(r1, vscale(sd, w1));
            ptri(q0, q1, q2, nn, (Color){150, 152, 160, 255}, MAT_STEEL); ptri(q0, q2, q3, nn, (Color){150, 152, 160, 255}, MAT_STEEL);
        }
        pcyl(f2, f3, R * 0.17f, R * 0.14f, 14, (Color){50, 50, 56, 255}, MAT_SATIN, 2);
        return;
    }
    pcyl(b, vadd(b, vscale(n, 0.006f)), rimR, rimR * 0.97f, 24, (Color){168, 170, 176, 255}, MAT_STEEL, 2);   // flat aero cover
    for (int s = 0; s < 6; s++) {                             // cover segments show rotation
        V3 dir = vrot(fw, axis, W->spin + s * 1.0472f), sd = vcross(axis, dir);
        V3 f0 = vadd(b, vscale(n, 0.008f)), e = vadd(f0, vscale(dir, rimR * 0.96f));
        ptri(vsub(f0, vscale(sd, 0.012f)), vadd(f0, vscale(sd, 0.012f)), vadd(e, vscale(sd, 0.012f)), vnorm(n), (Color){74, 76, 80, 255}, MAT_SATIN);
        ptri(vsub(f0, vscale(sd, 0.012f)), vadd(e, vscale(sd, 0.012f)), vsub(e, vscale(sd, 0.012f)), vnorm(n), (Color){74, 76, 80, 255}, MAT_SATIN);
    }
}


// A soft-body airliner, drawn from its current node positions so dents and bends show.
static V3 afN[MAX_NODES];
// Skin whose frame has pulled apart is gone: don't stretch a triangle across the gap.
static int af_torn(const Car *c, const Tri *t) {
    for (int e = 0; e < 3; e++) {
        V3 d = vsub(c->n[t->v[e]].p, c->n[t->v[(e + 1) % 3]].p);
        if (vdot(d, d) > 2.1f * t->e0[e] * t->e0[e]) return 1;       // an edge 45% longer than built
    }
    return 0;
}
static int af_split(const Car *c, int a, int b) {                   // have two nodes moved well apart?
    V3 d = vsub(c->n[a].p, c->n[b].p), r = vsub(c->rest[a], c->rest[b]);
    return vdot(d, d) > 2.1f * vdot(r, r);
}
static V3 af_lerp_out(const Airframe *A, int a, int b, float t, float out) {
    V3 p = vlerp(A->c.n[a].p, A->c.n[b].p, t);
    return vadd(p, vscale(vnorm(vlerp(afN[a], afN[b], t)), out));
}
static void draw_airframe(const Airframe *A, int beamView, V3 eye) {
    const Car *c = &A->c;
    if (beamView) {
        for (int i = 0; i < c->nb; i++) {
            const Beam *B = &c->b[i];
            if (B->broken) continue;
            float d = fminf(fabsf(B->rest - B->rest0) / B->rest0 * 12.0f, 1.0f), s = fminf(fabsf(B->stress), 1.0f), t = fmaxf(s * 0.5f, d);
            DrawLine3D(RV(c->n[B->a].p), RV(c->n[B->b].p), (Color){(unsigned char)(60 + 195 * t), (unsigned char)(220 - 170 * t), (unsigned char)(90 - 60 * t), 255});
        }
        return;
    }
    Color liv = LIVERY[A->seed % 6];
    const Color pal[7] = {{236, 237, 240, 255}, {196, 198, 202, 255}, liv, {44, 56, 72, 255}, {205, 207, 212, 255}, {176, 178, 184, 255}, {36, 36, 40, 255}};
    // Blob shadow of the belly, wings and stabilisers (only without real shadows).
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    for (int i = 0; i < c->nt && !post_shadow_active() && !depthOnly; i++) {
        if (A->paint[i] != 1 || af_torn(c, &c->t[i])) continue;
        const Tri *t = &c->t[i];
        Vector3 p[3];
        for (int k = 0; k < 3; k++) { V3 v = c->n[t->v[k]].p; p[k] = (Vector3){v.x, terrain_height(v.x, v.z) + 0.05f, v.z}; }
        DrawTriangle3D(p[0], p[1], p[2], (Color){0, 0, 0, 70});
        DrawTriangle3D(p[0], p[2], p[1], (Color){0, 0, 0, 70});
    }
    rlDrawRenderBatchActive();
    rlEnableDepthMask();
    // Fuselage: smooth normals from the fuselage skin only.
    for (int i = 0; i < c->nn; i++) afN[i] = v3(0, 0, 0);
    for (int i = 0; i < A->nFusTris; i++) {
        const Tri *t = &c->t[i];
        V3 a = c->n[t->v[0]].p, fn = vcross(vsub(c->n[t->v[1]].p, a), vsub(c->n[t->v[2]].p, a));
        for (int k = 0; k < 3; k++) afN[t->v[k]] = vadd(afN[t->v[k]], fn);
    }
    for (int i = 0; i < c->nn; i++) afN[i] = vnorm(afN[i]);
    rlBegin(RL_TRIANGLES);
    for (int i = 0; i < A->nFusTris; i++) {
        const Tri *t = &c->t[i];
        if (af_torn(c, t)) continue;
        Color base = pal[A->paint[i]];
        float gloss = A->paint[i] == 3 ? 1.0f : 0.55f;
        for (int k = 0; k < 3; k++) {
            V3 p = c->n[t->v[k]].p;
            Color col = shade_paint(base, afN[t->v[k]], p, eye, gloss);
            rlColor4ub(col.r, col.g, col.b, col.a);
            rlVertex3f(p.x, p.y, p.z);
        }
    }
    rlEnd();
    // Wings, engines and tail: flat-shaded, both faces of the thin parts.
    for (int i = A->nFusTris; i < c->nt; i++) {
        const Tri *t = &c->t[i];
        if (af_torn(c, t)) continue;
        V3 p[3] = {c->n[t->v[0]].p, c->n[t->v[1]].p, c->n[t->v[2]].p};
        V3 nrm = vnorm(vcross(vsub(p[1], p[0]), vsub(p[2], p[0])));
        Color col = A->paint[i] == 6 ? (pbrOn ? pbr(pal[6], nrm, MAT_MATTE) : pal[6]) : A->paint[i] == 5 && pbrOn ? pbr(pal[5], nrm, MAT_STEEL) : shade_paint(pal[A->paint[i]], nrm, p[0], eye, 0.6f);
        DrawTriangle3D(RV(p[0]), RV(p[1]), RV(p[2]), col);
        if (A->paint[i] == 2) DrawTriangle3D(RV(p[0]), RV(p[2]), RV(p[1]), shade_paint(pal[2], vscale(nrm, -1), p[0], eye, 0.6f));
    }
    // Window band and livery cheatline, laid on the deforming skin between frames 2 and 13.
    for (int side = 0; side < 2; side++) {
        int wl = side ? 5 : 0, wh = side ? 4 : 1, cl = side ? 6 : 9, ch = side ? 5 : 0;
        for (int i = 2; i < 13; i++) {
            const int *r0 = A->ring[i], *r1 = A->ring[i + 1];
            if (af_split(c, r0[wl], r1[wl]) || af_split(c, r0[wh], r1[wh]) || af_split(c, r0[cl], r1[cl]) || af_split(c, r0[wl], r0[wh]) || af_split(c, r1[wl], r1[wh]) || af_split(c, r0[cl], r0[ch])) continue;
            V3 a0 = af_lerp_out(A, r0[wl], r0[wh], 0.30f, 0.02f), a1 = af_lerp_out(A, r0[wl], r0[wh], 0.52f, 0.02f);
            V3 b0 = af_lerp_out(A, r1[wl], r1[wh], 0.30f, 0.02f), b1 = af_lerp_out(A, r1[wl], r1[wh], 0.52f, 0.02f);
            Color g = shade_paint(pal[3], afN[r0[wl]], a0, eye, 1.0f);
            DrawTriangle3D(RV(a0), RV(b0), RV(b1), g); DrawTriangle3D(RV(a0), RV(b1), RV(a1), g);
            DrawTriangle3D(RV(a0), RV(b1), RV(b0), g); DrawTriangle3D(RV(a0), RV(a1), RV(b1), g);
            V3 c0 = af_lerp_out(A, r0[cl], r0[ch], 0.62f, 0.02f), c1 = af_lerp_out(A, r0[cl], r0[ch], 0.86f, 0.02f);
            V3 d0 = af_lerp_out(A, r1[cl], r1[ch], 0.62f, 0.02f), d1 = af_lerp_out(A, r1[cl], r1[ch], 0.86f, 0.02f);
            Color lv = shade_paint(liv, afN[r0[cl]], c0, eye, 0.55f);
            DrawTriangle3D(RV(c0), RV(d0), RV(d1), lv); DrawTriangle3D(RV(c0), RV(d1), RV(c1), lv);
            DrawTriangle3D(RV(c0), RV(d1), RV(d0), lv); DrawTriangle3D(RV(c0), RV(c1), RV(d1), lv);
        }
    }
    // Landing gear: struts and wheels, following the nodes.
    V3 fwd = vnorm(vsub(c->n[A->noseNode].p, c->n[A->tailNode].p)), ax = vnorm(vcross(fwd, v3(0, 1, 0)));
    Color strut = {150, 150, 156, 255}, tyre = {30, 30, 32, 255};
    for (int g = 0; g < 3; g++) {
        V3 w0 = c->n[A->gearWheel[g][0]].p, w1 = c->n[A->gearWheel[g][1]].p, top = c->n[A->gear[g][1]].p;
        if (af_split(c, A->gearWheel[g][0], A->gear[g][1]) || af_split(c, A->gearWheel[g][0], A->gearWheel[g][1])) continue;   // leg torn off
        float r = g ? 0.55f : 0.4f;
        V3 hub = vadd(vscale(vadd(w0, w1), 0.5f), v3(0, r, 0));
        pcyl(hub, top, 0.14f, 0.14f, 8, strut, MAT_STEEL, 0);
        for (int k = 0; k < 2; k++) {
            V3 wc = vadd(k ? w1 : w0, v3(0, r, 0));
            V3 off = g ? vscale(ax, 0.45f) : v3(0, 0, 0);
            for (int sgn = -1; sgn <= (g ? 1 : -1); sgn += 2) {
                V3 cc = vadd(wc, vscale(off, (float)sgn));
                pcyl(vsub(cc, vscale(ax, 0.17f)), vadd(cc, vscale(ax, 0.17f)), r, r, 14, tyre, MAT_MATTE, 3);
            }
        }
    }
}

static int flying;                              // the player is flying the airliner (Tab on the airport map)

// Navigation lights on the flyable airliner (emissive pass): green right tip, red left tip, white tail,
// a flashing red beacon, strobes, and landing lights at the wing roots when it's dark or you're flying.
static void draw_airliner_lights(const Airframe *A, float time, int landing) {
    const Car *c = &A->c;
    V3 rt = vadd(c->n[A->wing[0][3][0]].p, vscale(A->right, 0.15f)), lt = vsub(c->n[A->wing[1][3][0]].p, vscale(A->right, 0.15f));
    V3 tail = vsub(c->n[A->tailNode].p, vscale(A->fwd, 0.1f));
    V3 top = vadd(vscale(vadd(c->n[A->ring[6][2]].p, c->n[A->ring[6][3]].p), 0.5f), vscale(A->up, 0.12f));
    post_emissive_begin(3.0f);
    DrawCube(RV(rt), 0.3f, 0.3f, 0.3f, (Color){60, 255, 100, 255});
    DrawCube(RV(lt), 0.3f, 0.3f, 0.3f, (Color){255, 40, 30, 255});
    DrawCube(RV(tail), 0.25f, 0.25f, 0.25f, WHITE);
    if (fmodf(time, 1.0f) < 0.07f) { DrawCube(RV(rt), 0.55f, 0.55f, 0.55f, WHITE); DrawCube(RV(lt), 0.55f, 0.55f, 0.55f, WHITE); }
    if (fmodf(time + 0.5f, 1.2f) < 0.15f) DrawCube(RV(top), 0.4f, 0.4f, 0.4f, (Color){255, 30, 20, 255});
    post_emissive_end();
    if (landing) {
        post_emissive_begin(5.0f);
        for (int sd = 0; sd < 2; sd++) {
            V3 p = vadd(c->n[A->wing[sd][0][1]].p, vscale(A->fwd, 0.25f));
            DrawCube(RV(p), 0.35f, 0.35f, 0.35f, (Color){255, 250, 235, 255});
        }
        post_emissive_end();
    }
}

static V3 nodeN[MAX_NODES];

static const Color INTERIOR = {66, 66, 70, 255};          // headliner / pillar trim

// From inside the cabin: dashboard, instrument binnacle, steering wheel (turned with the front wheels)
// and rear-view mirror, all placed in car-local coordinates so they move with the body.
static void draw_interior(const Car *c, V3 eye) {
    const VehicleDef *d = c->def;
    V3 F = c->fwd, U = c->up, Rt = c->right, S = d->seat;
    Color dash = {48, 48, 52, 255}, dark = {28, 28, 30, 255};
    oboxGloss = 0.0f;
    V3 dc = car_local_to_world(c, v3(d->dashX - 0.28f, d->dashY - 0.10f, 0));
    draw_obox(dc, vscale(F, 0.28f), vscale(U, 0.10f), vscale(Rt, d->beltHalf * 0.92f), dash, eye);
    V3 bz = car_local_to_world(c, v3(S.x + 0.66f, d->dashY + 0.035f, S.z));
    draw_obox(bz, vscale(F, 0.09f), vscale(U, 0.045f), vscale(Rt, 0.19f), dark, eye);
    // Steering wheel on a column tilted up toward the driver.
    V3 hub = car_local_to_world(c, v3(S.x + 0.44f, S.y - 0.33f, S.z));
    V3 ax = vnorm(vadd(vscale(F, -0.94f), vscale(U, 0.34f)));
    float turn = c->steer * 14.0f;                       // ~14:1 steering ratio
    V3 u = vrot(Rt, ax, turn), v = vcross(ax, u);
    const float R = 0.19f;
    Color rim = {26, 26, 28, 255};
    for (int i = 0; i < 24; i++) {
        float a0 = 6.2831853f * i / 24, a1 = 6.2831853f * (i + 1) / 24;
        V3 p0 = vadd(hub, vadd(vscale(u, cosf(a0) * R), vscale(v, sinf(a0) * R)));
        V3 p1 = vadd(hub, vadd(vscale(u, cosf(a1) * R), vscale(v, sinf(a1) * R)));
        DrawCylinderEx(RV(p0), RV(p1), 0.017f, 0.017f, 6, rim);
    }
    const float spokes[3] = {0.0f, 3.14159f, -1.5708f};
    for (int i = 0; i < 3; i++) {
        V3 e = vadd(hub, vadd(vscale(u, cosf(spokes[i]) * R), vscale(v, sinf(spokes[i]) * R)));
        DrawCylinderEx(RV(hub), RV(e), 0.022f, 0.014f, 6, (Color){34, 34, 37, 255});
    }
    DrawCylinderEx(RV(vsub(hub, vscale(ax, 0.02f))), RV(vadd(hub, vscale(ax, 0.03f))), 0.055f, 0.05f, 12, (Color){40, 40, 44, 255});
    DrawCylinderEx(RV(vsub(hub, vscale(ax, 0.02f))), RV(vsub(hub, vscale(ax, 0.30f))), 0.03f, 0.03f, 8, dark);   // column
    // Rear-view mirror at the top of the windshield.
    V3 rm = car_local_to_world(c, v3(S.x + 0.62f, S.y + 0.14f, 0));
    draw_obox(rm, vscale(F, 0.012f), vscale(U, 0.026f), vscale(Rt, 0.085f), dark, eye);
    draw_obox(vsub(rm, vscale(F, 0.014f)), vscale(F, 0.002f), vscale(U, 0.020f), vscale(Rt, 0.078f), (Color){90, 100, 112, 255}, eye);
    DrawCylinderEx(RV(rm), RV(vadd(rm, vadd(vscale(U, 0.07f), vscale(F, 0.03f)))), 0.008f, 0.008f, 6, dark);
    oboxGloss = 0.6f;
}

// A pillar segment: a flattened bar between two body nodes, nudged toward the cabin centre.
static void pillar_seg(const Car *c, V3 a, V3 b, float w, V3 eye) {
    V3 mid = vscale(vadd(a, b), 0.5f), ax = vscale(vsub(b, a), 0.5f);
    V3 in = vsub(c->pos, mid); in = vnorm(vsub(in, vscale(c->up, vdot(in, c->up))));   // horizontal, toward the centre
    V3 side = vnorm(vcross(ax, in));
    mid = vadd(mid, vscale(in, 0.025f));
    draw_obox(mid, vadd(ax, vscale(vnorm(ax), 0.02f)), vscale(side, w), vscale(in, 0.03f), INTERIOR, eye);
}

// Seats: cushion, reclined backrest and headrest, in car-local coordinates around (x, z).
static void draw_seat(const Car *c, float x, float z, float halfW, V3 eye) {
    const VehicleDef *d = c->def;
    Color fabric = {84, 82, 80, 255}, insert = {104, 100, 96, 255};
    V3 F = c->fwd, U = c->up, Rt = c->right;
    float cy = d->seat.y - 0.66f;                                       // cushion centre height
    draw_obox(car_local_to_world(c, v3(x + 0.04f, cy, z)), vscale(F, 0.25f), vscale(U, 0.06f), vscale(Rt, halfW), fabric, eye);
    draw_obox(car_local_to_world(c, v3(x + 0.05f, cy + 0.065f, z)), vscale(F, 0.20f), vscale(U, 0.005f), vscale(Rt, halfW * 0.6f), insert, eye);
    V3 bF = vrot(F, Rt, -0.26f), bU = vrot(U, Rt, -0.26f);              // backrest leans back ~15 degrees
    draw_obox(car_local_to_world(c, v3(x - 0.27f, cy + 0.34f, z)), vscale(bF, 0.06f), vscale(bU, 0.31f), vscale(Rt, halfW), fabric, eye);
    draw_obox(car_local_to_world(c, v3(x - 0.20f, cy + 0.34f, z)), vscale(bF, 0.005f), vscale(bU, 0.24f), vscale(Rt, halfW * 0.6f), insert, eye);
    draw_obox(car_local_to_world(c, v3(x - 0.36f, cy + 0.74f, z)), vscale(bF, 0.05f), vscale(bU, 0.09f), vscale(Rt, fminf(halfW, 0.13f)), fabric, eye);
}

// Cabin furniture that shows from inside, or through a broken window.
static void draw_cabin(const Car *c, V3 eye) {
    oboxGloss = 0.0f;
    for (int s = 0; s < 2; s++)                                      // A-pillars
        for (int i = 0; i + 1 < c->naPillar; i++)
            pillar_seg(c, c->n[c->aPillar[s][i]].p, c->n[c->aPillar[s][i + 1]].p, 0.045f, eye);
    pillar_seg(c, c->n[c->bPillar[0]].p, c->n[c->bPillar[1]].p, 0.06f, eye);   // B-pillars
    pillar_seg(c, c->n[c->bPillar[2]].p, c->n[c->bPillar[3]].p, 0.06f, eye);
    const VehicleDef *d = c->def;
    draw_seat(c, d->seat.x, d->seat.z, 0.24f, eye);                   // driver
    draw_seat(c, d->seat.x, -d->seat.z, 0.24f, eye);                  // front passenger
    draw_seat(c, d->rearSeatX + 0.24f, 0, d->beltHalf * 0.78f, eye);  // rear bench (headrest at the centre)
    oboxGloss = 0.6f;
}

// Signal lamps: the lit one bright (it blooms), the others dim.
static void draw_signal_lamps(int lit) {
    static const Color LAMP_COL[3] = {{255, 40, 30, 255}, {255, 170, 20, 255}, {40, 255, 120, 255}};
    for (int i = 0; i < traffic_signal_count(); i++) {
        V3 head, face, pole; int st;
        traffic_signal(i, &head, &face, &pole, &st);
        for (int k = 0; k < 3; k++) {                     // k: 0 red (top), 1 amber, 2 green
            int state = k == 0 ? LIGHT_RED : k == 1 ? LIGHT_YELLOW : LIGHT_GREEN;
            if ((state == st) != lit) continue;
            V3 c = vadd(vadd(head, vscale(face, 0.145f)), v3(0, 0.33f - 0.33f * k, 0));
            DrawCylinderEx(RV(c), RV(vadd(c, vscale(face, 0.01f))), 0.12f, 0.12f, 12, lit ? LAMP_COL[k] : (Color){LAMP_COL[k].r / 5, LAMP_COL[k].g / 5, LAMP_COL[k].b / 5, 255});
        }
    }
}

// Instrument dials glow in the dark (emissive pass).
static void draw_gauges(const Car *c) {
    const VehicleDef *d = c->def;
    for (int k = -1; k <= 1; k += 2) {
        V3 g = car_local_to_world(c, v3(d->seat.x + 0.565f, d->dashY + 0.03f, d->seat.z + k * 0.08f));
        DrawCylinderEx(RV(g), RV(vsub(g, vscale(c->fwd, 0.004f))), 0.045f, 0.045f, 16, (Color){255, 150, 60, 255});
        DrawCylinderEx(RV(vsub(g, vscale(c->fwd, 0.005f))), RV(vsub(g, vscale(c->fwd, 0.006f))), 0.036f, 0.036f, 16, (Color){20, 14, 10, 255});
    }
}

static void draw_car(const Car *c, int beamView, V3 eye, int interior) {
    Color body = {(unsigned char)(c->color.x * 255), (unsigned char)(c->color.y * 255), (unsigned char)(c->color.z * 255), 255};
    Color glass = {34, 46, 60, 255}, under = {35, 35, 38, 255}, tonneau = {52, 54, 58, 255}, trim = {30, 31, 33, 255};

    // Blob shadow: project the underside onto the ground.
    Color sh = {0, 0, 0, 90};
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    for (int i = 0; i < c->nt && !post_shadow_active() && !depthOnly; i++) {
        const Tri *t = &c->t[i];
        if (t->kind != SURF_UNDER) continue;
        Vector3 p[3];
        for (int k = 0; k < 3; k++) { V3 v = c->n[t->v[k]].p; p[k] = (Vector3){v.x, terrain_height(v.x, v.z) + 0.04f, v.z}; }
        DrawTriangle3D(p[0], p[1], p[2], sh);
    }
    rlDrawRenderBatchActive();
    rlEnableDepthMask();

    if (!beamView && c->def->smooth) {
        // Smooth-shaded paint: average face normals at each node, light per vertex.
        for (int i = 0; i < c->nn; i++) nodeN[i] = v3(0, 0, 0);
        for (int i = 0; i < c->nt; i++) {
            const Tri *t = &c->t[i];
            if (t->kind != SURF_BODY && t->kind != SURF_TRIM) continue;
            V3 a = c->n[t->v[0]].p, b = c->n[t->v[1]].p, d = c->n[t->v[2]].p;
            V3 fn = vcross(vsub(b, a), vsub(d, a));
            for (int k = 0; k < 3; k++) nodeN[t->v[k]] = vadd(nodeN[t->v[k]], fn);
        }
        for (int i = 0; i < c->nn; i++) nodeN[i] = vnorm(nodeN[i]);
        rlBegin(RL_TRIANGLES);
        for (int i = 0; i < c->nt; i++) {
            const Tri *t = &c->t[i];
            if (t->kind != SURF_BODY && t->kind != SURF_TRIM) continue;
            Color base = t->kind == SURF_TRIM ? trim : body;
            float gloss = t->kind == SURF_TRIM ? 0.25f : 1.0f;
            int inside = 0;
            if (interior) {
                V3 a = c->n[t->v[0]].p, fn = vcross(vsub(c->n[t->v[1]].p, a), vsub(c->n[t->v[2]].p, a));
                inside = vdot(fn, vsub(eye, a)) < 0;
            }
            for (int k = 0; k < 3; k++) {
                V3 p = c->n[t->v[k]].p;
                Color col = inside ? INTERIOR : shade_paint(base, nodeN[t->v[k]], p, eye, gloss);
                rlColor4ub(col.r, col.g, col.b, col.a);
                rlVertex3f(p.x, p.y, p.z);
            }
        }
        rlEnd();
    }
    if (!beamView) {
        for (int i = 0; i < c->nt; i++) {
            const Tri *t = &c->t[i];
            if (t->panel >= 0 && c->glassBroken[t->panel]) continue;
            if (interior && t->kind == SURF_GLASS) continue;          // looking out through our own windows
            if (c->def->smooth && (t->kind == SURF_BODY || t->kind == SURF_TRIM)) {   // drawn above; keep shut lines
                if (t->edges) {
                    Color edge = {28, 26, 26, 255};
                    for (int e = 0; e < 3; e++)
                        if (t->edges >> e & 1) {
                            int a = t->v[e], b = t->v[(e + 1) % 3];
                            DrawLine3D(RV(vadd(c->n[a].p, vscale(nodeN[a], 0.006f))), RV(vadd(c->n[b].p, vscale(nodeN[b], 0.006f))), edge);
                        }
                }
                continue;
            }
            V3 p[3] = {c->n[t->v[0]].p, c->n[t->v[1]].p, c->n[t->v[2]].p};
            V3 nrm = vnorm(vcross(vsub(p[1], p[0]), vsub(p[2], p[0])));
            Color base = t->kind == SURF_GLASS ? glass : t->kind == SURF_UNDER ? under : t->kind == SURF_CHROME ? CHROME :
                         t->kind == SURF_TONNEAU ? tonneau : t->kind == SURF_TRIM ? trim : body;
            Color col = lit(base, nrm, 1.0f);
            if (interior && vdot(nrm, vsub(eye, p[0])) < 0) { DrawTriangle3D(RV(p[0]), RV(p[1]), RV(p[2]), INTERIOR); continue; }
            int semi = c->type == VEH_SEMI;
            if (pbrOn && semi && t->kind == SURF_BODY) {
                // Flame livery: paint coordinates are metres behind the nose and height around the body.
                // Behind the sleeper the bare frame is dark.
                const V3 *r0 = &c->rest[t->v[0]];
                if (r0->x < -1.50f && c->rest[t->v[1]].x < -1.50f && c->rest[t->v[2]].x < -1.50f) {
                    col = pbr((Color){38, 38, 42, 255}, nrm, MAT_SATIN);
                    DrawTriangle3D(RV(p[0]), RV(p[1]), RV(p[2]), col);
                } else {
                    int top = 1;
                    for (int k = 0; k < 3; k++) { int pp = t->v[k] % 9; if (t->v[k] >= c->def->nr * 9 || (pp != 4 && pp != 5)) top = 0; }
                    col = pbr(base, nrm, MAT_FLAME);
                    rlBegin(RL_TRIANGLES);
                    rlColor4ub(col.r, col.g, col.b, col.a);
                    for (int k = 0; k < 3; k++) {
                        const V3 *r = &c->rest[t->v[k]];
                        rlTexCoord2f(c->def->st[0].x - r->x, r->y + 1.2f - (top ? r->z : fabsf(r->z)));
                        rlVertex3f(p[k].x, p[k].y, p[k].z);
                    }
                    rlEnd();
                }
                continue;
            }
            if (pbrOn) {
                col = pbr(base, nrm, t->kind == SURF_GLASS ? MAT_GLASS : t->kind == SURF_CHROME ? MAT_CHROME : t->kind == SURF_UNDER ? MAT_MATTE :
                                     t->kind == SURF_BODY ? (c->type == VEH_PICKUP ? MAT_STEEL : MAT_PAINT) : MAT_SATIN);
            } else if (t->kind == SURF_BODY) {                     // brushed steel picks up the sky
                float f = 0.18f + 0.22f * fmaxf(nrm.y, 0.0f);
                Color sk = {(unsigned char)(255 * powf(ENV->horizonAway.x, 0.4545f)), (unsigned char)(255 * powf(ENV->horizonAway.y, 0.4545f)), (unsigned char)(255 * powf(ENV->horizonAway.z, 0.4545f)), 255};
                col = (Color){(unsigned char)(col.r + (sk.r - col.r) * f), (unsigned char)(col.g + (sk.g - col.g) * f), (unsigned char)(col.b + (sk.b - col.b) * f), 255};
            }
            if (!pbrOn && (t->kind == SURF_BODY || t->kind == SURF_GLASS)) {   // flat-faceted stainless / glass highlight
                V3 hv = vnorm(vadd(SUN, vnorm(vsub(eye, p[0]))));
                float s = powf(fmaxf(vdot(nrm, hv), 0.0f), t->kind == SURF_GLASS ? 40.0f : 14.0f);
                col = brighten(col, s * (t->kind == SURF_GLASS ? 0.55f : 0.30f));
            }
            DrawTriangle3D(RV(p[0]), RV(p[1]), RV(p[2]), col);
            if (t->edges) {
                Color edge = c->def->smooth ? (Color){20, 20, 22, 255} : lit(base, nrm, 0.45f);
                V3 lift = vscale(nrm, c->def->smooth ? 0.006f : 0.0f);
                for (int e = 0; e < 3; e++)
                    if (t->edges >> e & 1) DrawLine3D(RV(vadd(p[e], lift)), RV(vadd(p[(e + 1) % 3], lift)), edge);
            }
        }
    } else {
        for (int i = 0; i < c->nb; i++) {
            const Beam *B = &c->b[i];
            if (B->broken) continue;
            Color col;
            if (B->type == BEAM_NORMAL) {
                float s = fminf(fabsf(B->stress), 1.0f);
                float d = fminf(fabsf(B->rest - B->rest0) / B->rest0 * 6.0f, 1.0f);
                float t = fmaxf(s * 0.6f, d);
                col = (Color){(unsigned char)(60 + 195 * t), (unsigned char)(220 - 170 * t), (unsigned char)(90 - 60 * t), 255};
            } else {
                col = B->type == BEAM_CABLE ? (Color){120, 160, 255, 255} : (Color){255, 120, 255, 255};   // straps / joints
            }
            DrawLine3D(RV(c->n[B->a].p), RV(c->n[B->b].p), col);
        }
        for (int i = 0; i < c->nn; i++) DrawSphere(RV(c->n[i].p), 0.03f, (Color){255, 230, 120, 255});
    }

    if (interior && !beamView) draw_interior(c, eye);
    if ((interior || c->glassBrokenCount) && !beamView) draw_cabin(c, eye);
    if (!beamView && c->def->smooth) {               // black grille + lower intake up front, diffuser at the back
        // Corner-lamp cars: lights[0] / lights[2] span the nose / tail ring as {top L, top R, belt L, belt R}.
        // Within a ring, floor-edge nodes are two places from the belt nodes (P_BLL - 2 = P_FL, P_BLR + 2 = P_FR).
        int bL = c->lights[0].q[2], bR = c->lights[0].q[3];
        LightStrip grille = {{bL, bR, bL - 2, bR + 2}, 0.45f, 0.30f, 0.70f, 1.0f, 0};
        LightStrip intake = {{bL - 2, bR + 2, bL, bR}, 0.40f, 0.12f, 0.88f, 1.0f, 0};
        int tL = c->lights[2].q[2], tR = c->lights[2].q[3];
        LightStrip diffuser = {{tL - 2, tR + 2, tL, tR}, 0.45f, 0.08f, 0.92f, -1.0f, 0};
        draw_lightbar(c, &grille, (Color){16, 16, 18, 255});
        draw_lightbar(c, &intake, (Color){24, 24, 26, 255});
        draw_lightbar(c, &diffuser, (Color){22, 22, 24, 255});
    }
    if (!beamView && c->mirrorNode[0] >= 0) {        // door mirrors on the A-pillar bases
        for (int s = 0; s < 2; s++) {
            V3 p = c->n[c->mirrorNode[s]].p, q = c->n[c->mirrorNode[s + 2]].p;
            V3 f = vnorm(vsub(p, q)), out = vscale(c->right, s ? 1.0f : -1.0f), up = c->up;
            V3 ctr = vadd(vadd(p, vscale(out, 0.12f)), vadd(vscale(up, 0.07f), vscale(f, -0.12f)));
            draw_obox(vadd(p, vadd(vscale(out, 0.04f), vadd(vscale(up, 0.03f), vscale(f, -0.10f)))), vscale(f, 0.04f), vscale(up, 0.015f), vscale(out, 0.04f), trim, eye);
            draw_obox(ctr, vscale(f, 0.045f), vscale(up, 0.048f), vscale(out, 0.08f), body, eye);
            draw_obox(vadd(ctr, vscale(f, -0.047f)), vscale(f, 0.004f), vscale(up, 0.040f), vscale(out, 0.07f), (Color){150, 160, 170, 255}, eye);   // mirror glass
        }
    }
    if (!beamView && c->spoilerNode[0] >= 0) {       // roof spoiler over the hatch
        V3 a = c->n[c->spoilerNode[0]].p, b = c->n[c->spoilerNode[1]].p;
        V3 back = vnorm(vsub(vscale(vadd(a, b), 0.5f), vscale(vadd(c->n[c->spoilerNode[2]].p, c->n[c->spoilerNode[3]].p), 0.5f)));
        back = vnorm(vsub(back, vscale(c->up, vdot(back, c->up) + 0.15f)));      // nearly level, a touch down
        V3 lift = vscale(c->up, 0.02f);
        V3 a2 = vadd(vadd(a, vscale(back, 0.24f)), lift), b2 = vadd(vadd(b, vscale(back, 0.24f)), lift);
        V3 a3 = vsub(a2, vscale(c->up, 0.05f)), b3 = vsub(b2, vscale(c->up, 0.05f));
        V3 n = vnorm(vcross(vsub(b, a), vsub(a2, a)));
        if (vdot(n, c->up) < 0) n = vscale(n, -1);
        Color top = shade_paint(body, n, a2, eye, 1.0f), lip = shade_paint(trim, back, a3, eye, 0.3f);
        DrawTriangle3D(RV(a), RV(b), RV(b2), top); DrawTriangle3D(RV(a), RV(b2), RV(a2), top);
        DrawTriangle3D(RV(a2), RV(b2), RV(b3), lip); DrawTriangle3D(RV(a2), RV(b3), RV(a3), lip);
        DrawTriangle3D(RV(a), RV(a2), RV(a3), lip); DrawTriangle3D(RV(b), RV(b2), RV(b3), lip);
    }

    if (!beamView && c->type == VEH_SEMI) draw_semi_parts(c);
    for (int i = 0; i < c->nw; i++) draw_wheel(c, i);
}

// Light bars, drawn in the emissive pass so they bloom.
static void draw_car_lights(const Car *c, int on) {
    float front = on ? 3.2f : 0.9f, rear = (on ? 2.2f : 1.4f) * (c->ctl.brake > 0.1f ? 2.0f : 1.0f);
    if (c->def->lightStyle == 2) {                  // the semi: twin headlamps each side, amber roof markers, tail lamps
        post_emissive_begin(front);
        for (int sd = -1; sd <= 1; sd += 2)
            for (int k = 0; k < 2; k++) {
                float z = sd * (0.80f + 0.22f * k);
                int nd = RN(0, sd < 0 ? 2 : 7);
                pcyl(part(c, nd, 3.800f, 0.73f, z), part(c, nd, 3.812f, 0.73f, z), 0.088f, 0.088f, 14, (Color){235, 242, 255, 255}, MAT_UNLIT, 2);
            }
        post_emissive_end();
        post_emissive_begin(on ? 2.4f : 0.8f);
        for (int k = 0; k < 5; k++) {
            float z = -0.76f + k * 0.38f;
            lbox(c, RN(8, k < 2 ? 4 : 5), 1.335f, 1.36f, 2.445f, 2.495f, z - 0.035f, z + 0.035f, (Color){255, 150, 30, 255}, MAT_UNLIT);
        }
        post_emissive_end();
        post_emissive_begin(rear);
        for (int sd = -1; sd <= 1; sd += 2) lbox(c, RN(19, 0), -4.475f, -4.45f, 0.22f, 0.36f, sd * 0.50f - 0.10f, sd * 0.50f + 0.10f, (Color){255, 26, 20, 255}, MAT_UNLIT);
        post_emissive_end();
        return;
    }
    for (int i = 0; i < c->nlights; i++) {
        const LightStrip *L = &c->lights[i];
        if (L->rear && c->tailgate == TG_GONE && c->def->hasTailgate) continue;   // went with the tailgate
        post_emissive_begin(L->rear ? rear : front);
        draw_lightbar(c, L, L->rear ? (Color){255, 26, 20, 255} : (Color){235, 242, 255, 255});
        post_emissive_end();
    }
}

// Headlight spill on the road ahead (additive), following the terrain.
static void draw_headlights(const Car *c) {
    V3 f = vnorm(v3(c->fwd.x, 0, c->fwd.z)), r = v3(-f.z, 0, f.x);
    V3 o = vadd(c->pos, vscale(f, c->def->st[0].x - c->comLocal.x - 0.2f));
    for (int i = 0; i < 12; i++) {
        float d0 = 1.0f + i * 2.4f, d1 = d0 + 2.4f;
        float w0 = 1.1f + d0 * 0.32f, w1 = 1.1f + d1 * 0.32f;
        float k0 = 0.55f / (1.0f + d0 * 0.09f), k1 = 0.55f / (1.0f + d1 * 0.09f);
        V3 pts[6] = {vadd(o, vadd(vscale(f, d0), vscale(r, -w0))), vadd(o, vscale(f, d0)), vadd(o, vadd(vscale(f, d0), vscale(r, w0))),
                     vadd(o, vadd(vscale(f, d1), vscale(r, -w1))), vadd(o, vscale(f, d1)), vadd(o, vadd(vscale(f, d1), vscale(r, w1)))};
        for (int k = 0; k < 6; k++) pts[k].y = terrain_height(pts[k].x, pts[k].z) + 0.06f;
        Color c0 = {(unsigned char)(255 * k0), (unsigned char)(246 * k0), (unsigned char)(230 * k0), 255};
        Color c1 = {(unsigned char)(255 * k1), (unsigned char)(246 * k1), (unsigned char)(230 * k1), 255}, off = {0, 0, 0, 255};
        rlBegin(RL_TRIANGLES);
        #define V(p, c) rlColor4ub(c.r, c.g, c.b, 255); rlVertex3f(p.x, p.y, p.z)
        V(pts[0], off); V(pts[3], off); V(pts[4], c1);   V(pts[0], off); V(pts[4], c1); V(pts[1], c0);
        V(pts[1], c0); V(pts[4], c1); V(pts[5], off);   V(pts[1], c0); V(pts[5], off); V(pts[2], off);
        #undef V
        rlEnd();
    }
}

// ------------------------------------------------------- glass shards ----

#define MAX_SHARD 1600
typedef struct { V3 p, v; float life, s; } Shard;
static Shard shards[MAX_SHARD];
static int shardHead, shardCount;

static float frand(void) { return (float)GetRandomValue(0, 10000) / 10000.0f; }

static void spawn_glass(const Car *c, unsigned mask) {
    for (int i = 0; i < c->nt; i++) {
        const Tri *t = &c->t[i];
        if (t->panel < 0 || !(mask >> t->panel & 1)) continue;
        V3 a = c->n[t->v[0]].p, b = c->n[t->v[1]].p, d = c->n[t->v[2]].p;
        V3 v = vscale(vadd(vadd(c->n[t->v[0]].v, c->n[t->v[1]].v), c->n[t->v[2]].v), 1.0f / 3.0f);
        for (int k = 0; k < 14; k++) {
            float u = frand(), w = frand();
            if (u + w > 1) { u = 1 - u; w = 1 - w; }
            Shard *s = &shards[shardHead];
            s->p = vadd(a, vadd(vscale(vsub(b, a), u), vscale(vsub(d, a), w)));
            s->v = vadd(v, v3((frand() - 0.5f) * 5, frand() * 2.5f, (frand() - 0.5f) * 5));
            s->life = 3.0f + 3.0f * frand();
            s->s = 0.02f + 0.04f * frand();
            shardHead = (shardHead + 1) % MAX_SHARD;
            if (shardCount < MAX_SHARD) shardCount++;
        }
    }
}

static void update_shards(float dt) {
    for (int i = 0; i < shardCount; i++) {
        Shard *s = &shards[i];
        if (s->life <= 0) continue;
        s->life -= dt;
        s->v.y -= 9.81f * dt;
        s->p = vadd(s->p, vscale(s->v, dt));
        float gh = terrain_height(s->p.x, s->p.z) + 0.01f;
        if (s->p.y < gh) { s->p.y = gh; s->v.y *= -0.25f; s->v.x *= 0.5f; s->v.z *= 0.5f; }
    }
}

static void draw_shards(void) {
    for (int i = 0; i < shardCount; i++) {
        const Shard *s = &shards[i];
        if (s->life <= 0) continue;
        unsigned char a = (unsigned char)(220 * fminf(1.0f, s->life));
        DrawCubeV(RV(s->p), (Vector3){s->s, s->s * 0.3f, s->s}, (Color){200, 225, 240, a});
    }
}

// -------------------------------------------------------------- weather ----
// O cycles dry / wet / rain. Wet roads reflect (render.c) and cost ~12-20% tyre grip;
// rain adds falling streaks and ripples; any wet road throws spray off the tyres.

static int weather = 1;                         // 0 dry, 1 wet, 2 rain, 3 snow
#define WEATHER_COUNT 4
static const char *WEATHER_NAME[] = {"DRY", "WET", "RAIN", "SNOW"};
static float weather_wet(void) { return weather == 1 ? 0.75f : (weather == 2 ? 1.0f : 0.0f); }
static float weather_rain(void) { return weather == 2 ? 1.0f : 0.0f; }
static float weather_grip(void) { return weather == 3 ? 0.5f : (weather == 2 ? 0.8f : (weather == 1 ? 0.85f : 1.0f)); }

#define MAX_DROPS 2600
static V3 drops[MAX_DROPS];
static int dropsInit;
static const V3 RAIN_VEL = {1.4f, -11.0f, 0.9f};

static void respawn_drop(V3 *d, V3 cam, int anywhere) {
    float span = weather == 3 ? 30.0f : 56.0f;         // snow: keep the flakes close, where you can see them
    d->x = cam.x + (frand() - 0.5f) * span;
    d->z = cam.z + (frand() - 0.5f) * span;
    if (weather == 3) d->y = cam.y + (anywhere ? (frand() - 0.35f) * 16.0f : 5.0f + frand() * 6.0f);   // flakes fall slowly: start close
    else d->y = cam.y + (anywhere ? (frand() - 0.4f) * 30.0f : 14.0f + frand() * 6.0f);
}

static void update_rain(V3 cam, float dt) {
    if (!dropsInit) { for (int i = 0; i < MAX_DROPS; i++) respawn_drop(&drops[i], cam, 1); dropsInit = 1; }
    float t = (float)GetTime();
    for (int i = 0; i < MAX_DROPS; i++) {
        V3 *d = &drops[i];
        if (weather == 3)                                // flakes: slow fall, each drifting on its own wobble
            *d = vadd(*d, vscale(v3(0.5f + 0.6f * sinf(t * 0.9f + i * 1.7f), -1.1f - 0.4f * ((i * 37) % 10) / 10.0f, 0.3f + 0.6f * cosf(t * 0.7f + i * 2.3f)), dt));
        else *d = vadd(*d, vscale(RAIN_VEL, dt));
        float lim = weather == 3 ? 16.0f : 30.0f;
        if (d->y < terrain_height(d->x, d->z) || fabsf(d->x - cam.x) > lim || fabsf(d->z - cam.z) > lim || d->y < cam.y - 14)
            respawn_drop(d, cam, 0);
    }
}

// Thin camera-facing streaks along the fall direction.
static void draw_rain(V3 cam) {
    V3 dir = vnorm(RAIN_VEL);
    rlBegin(RL_TRIANGLES);
    for (int i = 0; i < MAX_DROPS; i++) {
        V3 a = drops[i], b = vadd(a, vscale(dir, 0.55f));
        V3 side = vscale(vnorm(vcross(dir, vsub(a, cam))), 0.011f);
        unsigned char al = (unsigned char)(40 + (i * 37 % 50));
        rlColor4ub(170, 185, 210, al);
        rlVertex3f(a.x - side.x, a.y - side.y, a.z - side.z); rlVertex3f(b.x - side.x, b.y - side.y, b.z - side.z); rlVertex3f(b.x + side.x, b.y + side.y, b.z + side.z);
        rlVertex3f(a.x - side.x, a.y - side.y, a.z - side.z); rlVertex3f(b.x + side.x, b.y + side.y, b.z + side.z); rlVertex3f(a.x + side.x, a.y + side.y, a.z + side.z);
    }
    rlEnd();
}

// Snowflakes: small soft sprites (softTex is created with the spray, below).
static Texture2D softTex;
static void draw_snow(Camera3D cam) {
    for (int i = 0; i < MAX_DROPS; i++) {
        float sz = 0.07f + 0.07f * ((i * 53) % 10) / 10.0f;
        DrawBillboard(cam, softTex, RV(drops[i]), sz, (Color){240, 244, 252, 230});
    }
}

#define MAX_SPRAY 2400
typedef struct { V3 p, v; float life, max, size; } Spray;
static Spray spray[MAX_SPRAY];
static int sprayHead;

static void init_spray(void) {
    Image im = GenImageGradientRadial(64, 64, 0.0f, (Color){255, 255, 255, 255}, (Color){255, 255, 255, 0});
    softTex = LoadTextureFromImage(im);
    UnloadImage(im);
    SetTextureFilter(softTex, TEXTURE_FILTER_BILINEAR);
}

// Mist thrown up behind each tyre on a wet road, more with speed.
static void emit_spray(const World *w, float dt) {
    if (!weather) return;
    for (int ci = 0; ci < w->ncars; ci++) {
        const Car *c = &w->cars[ci];
        if (!c->active) continue;
        float sp = vlen(c->vel);
        if (sp < 4.0f) continue;
        for (int i = 0; i < c->nw; i++) {
            const Wheel *W = &c->w[i];
            if (!W->contact) continue;
            float n = sp * dt * (weather == 2 ? 2.2f : 1.4f) * (road_at(W->cp.x, W->cp.z) ? 1.0f : 0.35f);
            int k = (int)n + (frand() < n - (int)n);
            for (int j = 0; j < k; j++) {
                Spray *s = &spray[sprayHead];
                sprayHead = (sprayHead + 1) % MAX_SPRAY;
                V3 back = vscale(c->fwd, -c->def->wheelR * 0.9f);
                s->p = vadd(vadd(W->cp, back), v3(0, 0.12f, 0));
                s->v = vadd(vscale(c->vel, 0.55f + 0.2f * frand()),
                            v3((frand() - 0.5f) * 2.5f, 1.0f + frand() * 2.2f, (frand() - 0.5f) * 2.5f));
                s->v = vadd(s->v, vscale(c->right, (i % 2 ? 1.0f : -1.0f) * frand() * 1.5f));
                s->max = s->life = 0.5f + 0.7f * frand();
                s->size = 0.25f + 0.25f * frand();
            }
        }
    }
}

static void update_spray(float dt) {
    for (int i = 0; i < MAX_SPRAY; i++) {
        Spray *s = &spray[i];
        if (s->life <= 0) continue;
        s->life -= dt;
        s->v = vscale(s->v, 1.0f - fminf(dt * 2.6f, 0.9f));     // air drag stops the mist quickly
        s->v.y -= 2.5f * dt;
        s->p = vadd(s->p, vscale(s->v, dt));
        s->size += dt * 1.6f;
        float g = terrain_height(s->p.x, s->p.z) + 0.05f;
        if (s->p.y < g) { s->p.y = g; s->v.y = 0; }
    }
}

static void draw_spray(Camera3D cam) {
    for (int i = 0; i < MAX_SPRAY; i++) {
        const Spray *s = &spray[i];
        if (s->life <= 0) continue;
        float a = s->life / s->max;
        unsigned char al = (unsigned char)(70 * a * a);
        DrawBillboard(cam, softTex, RV(s->p), s->size, (Color){200, 208, 220, al});
    }
}

// ---------------------------------------------------------- crash effects ----
// Dust and sparks where wreckage scrapes the ground, spray where it hits the water, and a fuel fire
// with smoke once an airliner has broken up on land.
enum { FX_DUST, FX_SPLASH, FX_SMOKE, FX_FIRE, FX_SPARK };
typedef struct { V3 p, v; float life, max, size; unsigned char kind; } Fx;
#define MAX_FX 4000
static Fx fx[MAX_FX];
static int fxHead;
static float fireT[MAX_PLANES];
static int wasWrecked[MAX_PLANES];

static void fx_emit(int kind, V3 p, V3 v, float life, float size) {
    Fx *f = &fx[fxHead];
    fxHead = (fxHead + 1) % MAX_FX;
    f->kind = (unsigned char)kind; f->p = p; f->v = v; f->life = f->max = life; f->size = size;
}
static V3 frand3(float s) { return v3((frand() - 0.5f) * 2 * s, (frand() - 0.5f) * 2 * s, (frand() - 0.5f) * 2 * s); }
static void fx_clear(void) { for (int i = 0; i < MAX_FX; i++) fx[i].life = 0; for (int i = 0; i < MAX_PLANES; i++) { fireT[i] = 0; wasWrecked[i] = 0; } }

static void emit_crash_fx(World *w, float dt) {
    if (dt <= 0) return;
    for (int ai = 0; ai < w->nplanes; ai++) {
        Airframe *A = &w->air[ai];
        const Car *c = &A->c;
        if (!A->wrecked) { wasWrecked[ai] = 0; fireT[ai] = 0; }
        if (A->asleep) continue;
        int onLand = 0, touching = 0;
        for (int i = 0; i < c->nn; i++) {
            const Node *N = &c->n[i];
            float sp = vlen(N->v);
            if (sp < 4.0f) continue;
            float g = terrain_height(N->p.x, N->p.z);
            int sea = g < AP_SEA_Y - 0.2f && w->map == MAP_AIRPORT;
            if (sea) {
                if (N->p.y > AP_SEA_Y + 0.5f || N->p.y < AP_SEA_Y - 2.5f) continue;
                touching++;
                float n = sp * 0.5f * dt;
                for (int k = (int)n + (frand() < n - (int)n); k > 0; k--)
                    fx_emit(FX_SPLASH, v3(N->p.x, AP_SEA_Y + 0.1f, N->p.z), vadd(v3(N->v.x * 0.25f, 3.0f + sp * (0.10f + 0.22f * frand()), N->v.z * 0.25f), frand3(2.0f + sp * 0.05f)), 1.2f + 1.3f * frand(), 0.8f + 1.6f * frand());
            } else {
                if (N->p.y > g + 0.3f) continue;
                touching++; onLand++;
                float n = sp * 0.22f * dt;
                for (int k = (int)n + (frand() < n - (int)n); k > 0; k--)
                    fx_emit(FX_DUST, v3(N->p.x, g + 0.2f, N->p.z), vadd(v3(N->v.x * 0.2f, 1.0f + 2.5f * frand(), N->v.z * 0.2f), frand3(1.5f)), 1.5f + 2.0f * frand(), 1.2f + 1.5f * frand());
                n = sp * 0.30f * dt;
                for (int k = (int)n + (frand() < n - (int)n); k > 0; k--)
                    fx_emit(FX_SPARK, v3(N->p.x, g + 0.1f, N->p.z), vadd(v3(N->v.x * 0.5f, 2.0f + 5.0f * frand(), N->v.z * 0.5f), frand3(4.0f)), 0.4f + 0.6f * frand(), 0.10f + 0.10f * frand());
            }
        }
        // Breaking up starts a fuel fire (not if it went gently into the water).
        if (A->wrecked && !wasWrecked[ai]) {
            wasWrecked[ai] = 1;
            if (onLand || A->nbroken > 60) {
                fireT[ai] = 75.0f;
                for (int k = 0; k < 90; k++) fx_emit(FX_FIRE, vadd(A->pos, frand3(6.0f)), vadd(vscale(A->vel, 0.3f), vadd(frand3(9.0f), v3(0, 7, 0))), 0.7f + 0.9f * frand(), 3.0f + 4.0f * frand());
                for (int k = 0; k < 50; k++) fx_emit(FX_SMOKE, vadd(A->pos, frand3(7.0f)), vadd(vscale(A->vel, 0.2f), vadd(frand3(5.0f), v3(0, 6, 0))), 4.0f + 4.0f * frand(), 4.0f + 4.0f * frand());
            }
        }
        if (fireT[ai] > 0) {
            fireT[ai] -= dt;
            float k = fminf(fireT[ai] / 20.0f, 1.0f);                    // dies down over the last 20 s
            const int hot[6] = {A->eng[0][0][1], A->eng[1][0][1], A->wing[0][0][0], A->wing[1][0][0], A->wing[0][1][2], A->wing[1][1][2]};
            for (int q = 0; q < 6; q++) {
                V3 p = c->n[hot[q]].p;
                if (p.y < AP_SEA_Y + 0.2f && terrain_height(p.x, p.z) < AP_SEA_Y) continue;    // under water: out
                float n = 26.0f * k * dt;
                for (int j = (int)n + (frand() < n - (int)n); j > 0; j--)
                    fx_emit(FX_FIRE, vadd(p, frand3(1.2f)), vadd(vscale(c->n[hot[q]].v, 0.5f), v3((frand() - 0.5f) * 1.5f, 2.5f + 2.5f * frand(), (frand() - 0.5f) * 1.5f)), 0.5f + 0.6f * frand(), 1.6f + 1.8f * frand());
                n = 7.0f * (0.4f + 0.6f * k) * dt;
                for (int j = (int)n + (frand() < n - (int)n); j > 0; j--)
                    fx_emit(FX_SMOKE, vadd(p, v3((frand() - 0.5f) * 1.5f, 1.5f, (frand() - 0.5f) * 1.5f)), v3(1.2f + frand(), 3.5f + 2.0f * frand(), 0.6f * frand()), 6.0f + 5.0f * frand(), 2.0f + 1.5f * frand());
            }
        }
        (void)touching;
    }
}

static void update_fx(float dt) {
    for (int i = 0; i < MAX_FX; i++) {
        Fx *f = &fx[i];
        if (f->life <= 0) continue;
        f->life -= dt;
        switch (f->kind) {
        case FX_DUST:   f->v = vscale(f->v, 1.0f - fminf(dt * 1.8f, 0.9f)); f->v.y += 0.4f * dt; f->size += 2.2f * dt; break;
        case FX_SPLASH: f->v.y -= 9.81f * dt; f->v = vscale(f->v, 1.0f - fminf(dt * 0.5f, 0.9f)); f->size += 1.5f * dt;
                        if (f->p.y < AP_SEA_Y - 0.2f && f->v.y < 0) f->life = 0; break;
        case FX_SMOKE:  f->v = vscale(f->v, 1.0f - fminf(dt * 0.6f, 0.9f)); f->v.y += 1.2f * dt; f->v.x += 0.5f * dt; f->size += 1.9f * dt; break;
        case FX_FIRE:   f->v = vscale(f->v, 1.0f - fminf(dt * 1.5f, 0.9f)); f->v.y += 6.0f * dt; f->size *= 1.0f - fminf(dt * 0.7f, 0.9f); break;
        default:        f->v.y -= 9.81f * dt;
                        { float g = terrain_height(f->p.x, f->p.z) + 0.03f; if (f->p.y < g) { f->p.y = g; f->v.y = fabsf(f->v.y) * 0.4f; } } break;
        }
        f->p = vadd(f->p, vscale(f->v, dt));
    }
}

// Soft sprites: dust, spray and smoke with ordinary blending; flames and sparks added on top (so they bloom).
static void draw_fx(Camera3D cam, int glow) {
    for (int i = 0; i < MAX_FX; i++) {
        const Fx *f = &fx[i];
        if (f->life <= 0) continue;
        float a = f->life / f->max;
        if (!glow) {
            if (f->kind == FX_DUST) DrawBillboard(cam, softTex, RV(f->p), f->size, (Color){150, 134, 110, (unsigned char)(120 * a)});
            else if (f->kind == FX_SPLASH) DrawBillboard(cam, softTex, RV(f->p), f->size, (Color){236, 242, 248, (unsigned char)(170 * a)});
            else if (f->kind == FX_SMOKE) DrawBillboard(cam, softTex, RV(f->p), f->size, (Color){26, 25, 25, (unsigned char)(190 * fminf(a * 1.6f, 1.0f) * fminf((1.0f - a) * 8.0f + 0.25f, 1.0f))});
        } else {
            if (f->kind == FX_FIRE) DrawBillboard(cam, softTex, RV(f->p), f->size, (Color){255, (unsigned char)(90 + 130 * a), (unsigned char)(20 + 60 * a * a), (unsigned char)(200 * fminf(a * 2.0f, 1.0f))});
            else if (f->kind == FX_SPARK) DrawBillboard(cam, softTex, RV(f->p), f->size, (Color){255, 200, 110, (unsigned char)(255 * a)});
        }
    }
}

// ---------------------------------------------------------- world meshes ----

static Model city, cityGlow, cityPool;
static int meshesBuilt;

static void build_world_meshes(const World *w) {
    if (meshesBuilt) {
        for (int i = 0; i < CH_N * CH_N; i++) UnloadModel(chunks[i]);
        UnloadModel(city); UnloadModel(cityGlow); UnloadModel(cityPool);
        UnloadModel(treeModel[0]); UnloadModel(treeModel[1]); UnloadModel(stumpModel);
    }
    for (int z = 0; z < CH_N; z++) for (int x = 0; x < CH_N; x++) chunks[z * CH_N + x] = build_chunk(x, z);
    for (int i = 0; i < nExt; i++) UnloadModel(extChunk[i]);
    if (hasFar) UnloadModel(farTerrain);
    nExt = 0; hasFar = 0;
    if (w->map == MAP_AIRPORT) {
        for (float z = EXT_Z0; z < EXT_Z1; z += CH_CELLS * CELL)
            for (float x = EXT_X0; x < EXT_X1 && nExt < EXT_MAX; x += CH_CELLS * CELL) extChunk[nExt++] = build_terrain(x, z, CH_CELLS, CELL, 0);
        farTerrain = build_terrain(-AP_FAR, -AP_FAR, (int)(2 * AP_FAR / FAR_CELL), FAR_CELL, 1);
        hasFar = 1;
    }
    city = build_city(w, &cityGlow, &cityPool);
    build_markings(w);
    cityGlow.materials[0].shader = post_emissive_shader();
    cityPool.materials[0].shader = post_emissive_shader();
    build_trees();
    build_tree_patches(w);
    meshesBuilt = 1;
}

// ------------------------------------------------------------------- HUD ----

static Font hudFont;
static const Color HUD_ORANGE = {255, 140, 32, 255}, HUD_BG = {10, 12, 16, 200}, HUD_DIM = {160, 166, 176, 255}, HUD_LINE = {255, 255, 255, 28};

static void hud_load_font(void) {
    const char *cand[] = {"/System/Library/Fonts/Supplemental/DIN Alternate Bold.ttf", "/System/Library/Fonts/Supplemental/DIN Condensed Bold.ttf",
                          "/System/Library/Fonts/Supplemental/Arial Bold.ttf"};
    hudFont = GetFontDefault();
    for (int i = 0; i < 3; i++)
        if (FileExists(cand[i])) {
            hudFont = LoadFontEx(cand[i], 96, NULL, 0);
            GenTextureMipmaps(&hudFont.texture);
            SetTextureFilter(hudFont.texture, TEXTURE_FILTER_TRILINEAR);
            break;
        }
}
static void txt(const char *s, float x, float y, float size, Color c) { DrawTextEx(hudFont, s, (Vector2){x, y}, size, size * 0.06f, c); }
static float txtw(const char *s, float size) { return MeasureTextEx(hudFont, s, size, size * 0.06f).x; }
static void slash(float x, float y, float w, float h, Color c) {       // slanted accent bar
    Vector2 A = {x + h * 0.4f, y}, B = {x + h * 0.4f + w, y}, C = {x + w, y + h}, D = {x, y + h};
    DrawTriangle(A, D, C, c); DrawTriangle(A, C, B, c);
}
static void hud_panel(float x, float y, float w, float h) {
    DrawRectangle((int)x, (int)y, (int)w, (int)h, HUD_BG);
    DrawRectangleLines((int)x, (int)y, (int)w, (int)h, HUD_LINE);
}
static const char *zone_name(V3 p, int map) {
    if (p.x > 28 && p.x < 80 && p.z < -12 && p.z > -42) return "WAREHOUSE DISTRICT";
    if (map == MAP_AIRPORT && p.z > 8) {
        if (p.x > AP_RWY_X0 && p.x < AP_RWY_X1 && p.z > AP_RWY_Z0 && p.z < AP_RWY_Z1) return "RUNWAY 09/27";
        int t = road_at(p.x, p.z);
        if (t == 4 && p.z > AP_TWY_Z0 - 1) return "TAXIWAY";
        if (t == 5) return "APRON";
        if (t == 4) return "CAR PARK";
        if (p.z > 34 && p.z < 82 && p.x > -70 && p.x < 130) return "TERMINAL";
        if (p.z < 230 && fabsf(p.x) < 268) return "AIRFIELD";
    }
    if (map == MAP_NEWYORK && p.z > CITY_Z0 && p.z < CITY_Z1 + 6 && fabsf(p.x) < CITY_X + 6) {
        float dx = p.x - TS_X, dz = p.z - TS_Z;
        return (dx * dx + dz * dz < 40 * 40 || road_at(p.x, p.z) == 3) ? "TIMES SQUARE" : "MIDTOWN";
    }
    if (p.z > 12 && p.z < 140 && fabsf(p.x) < 116) return fabsf(p.x) < 9 ? "CROSS ROAD" : "TOWN";
    if (p.x * p.x + p.z * p.z > 100 * 100) return "HILLS";
    return "PROVING GROUND";
}

static void draw_hud(const Car *pc, const World *w, float vF, Controls ctl, int slowmo, int paused, int help) {
    float sw = (float)GetScreenWidth(), sh = (float)GetScreenHeight();
    const char *zn = zone_name(pc->pos, w->map);
    hud_panel(16, 16, 74 + txtw(zn, 20), 40);
    slash(26, 26, 5, 20, HUD_ORANGE); slash(35, 26, 5, 20, HUD_ORANGE);
    txt(zn, 58, 26, 20, RAYWHITE);
    const char *tr = TextFormat("%s [M]   %s [L]   %s [O]     %d FPS", map_name(w->map), ENV->name, WEATHER_NAME[weather], GetFPS());
    txt(tr, sw - 22 - txtw(tr, 15), 22, 15, HUD_DIM);

    float px = 16, py = sh - 116, pw = 470, ph = 100;
    txt(TextFormat("%s   [V] change", pc->def->name), px + 2, py - 22, 15, HUD_DIM);
    hud_panel(px, py, pw, ph);
    slash(px + 12, py + 16, 4, 26, HUD_ORANGE);
    txt("DAMAGE", px + 30, py + 14, 28, RAYWHITE);
    Color dc = pc->damage < 15 ? HUD_ORANGE : (pc->damage < 60 ? (Color){255, 110, 30, 255} : (Color){240, 60, 50, 255});
    txt(TextFormat("%.0f%%", pc->damage), px + 34 + txtw("DAMAGE", 28) + 10, py + 14, 28, dc);
    DrawRectangle((int)px + 30, (int)py + 56, 230, 8, (Color){50, 52, 58, 255});
    DrawRectangle((int)px + 30, (int)py + 56, (int)(230 * pc->damage / 100.0f), 8, dc);
    txt(TextFormat("TORN BEAMS  %d", pc->brokenBeams), px + 30, py + 72, 14, HUD_DIM);
    DrawLine((int)px + 290, (int)py + 14, (int)px + 290, (int)(py + ph - 14), HUD_LINE);
    static const char *TG[] = {"LATCHED", "OPEN", "HANGING", "GONE", "N/A"};
    const char *lab[3] = {"GLASS", "TAILGATE", "TREES"};
    char v0[16], v2[16];
    snprintf(v0, sizeof v0, "%d/%d", pc->glassBrokenCount, pc->npanels);
    snprintf(v2, sizeof v2, "%d", w->treesDown);
    const char *val[3] = {v0, TG[pc->tailgate], v2};
    for (int i = 0; i < 3; i++) {
        txt(lab[i], px + 306, py + 14 + i * 26, 17, HUD_DIM);
        txt(val[i], px + 396, py + 14 + i * 26, 17, HUD_ORANGE);
    }

    float bw = 210, bx = sw - bw - 16, by = sh - 84;
    hud_panel(bx, by, bw, 68);
    slash(bx + 14, by + 14, 5, 40, HUD_ORANGE);
    char spd[16];
    snprintf(spd, sizeof spd, "%.0f", fabsf(vF) * 3.6f);
    txt(spd, bx + bw - 76 - txtw(spd, 48), by + 10, 48, RAYWHITE);
    txt("km/h", bx + bw - 66, by + 32, 20, HUD_DIM);
    char st[48];
    snprintf(st, sizeof st, "%s%s%s", slowmo ? "SLOW-MO  " : "", paused ? "PAUSED  " : "", ctl.handbrake ? "HANDBRAKE" : "");
    if (st[0]) txt(st, sw - 16 - txtw(st, 16), by - 24, 16, HUD_ORANGE);

    if (help) {
        const char *lines[] = {
            "W/S  throttle / brake / reverse", "A/D  steer      SPACE  handbrake", "MOUSE  orbit camera, wheel to zoom",
            "C  dash cam      B  beam view      V  choose vehicle", "N  spawn truck ahead      X  clear trucks", "Q  recover in place      R  reset to start",
            "T  slow-mo      P  pause      K  clear skids      H  help", "G  drop tailgate      L  lighting      O  weather",
            "M  map: Default / New York / Airport      J  traffic",
            "TAB  fly the airliner on the runway (Airport map)",
            "F2  sun shadows + ambient occlusion on / off",
            "", "Ramp ahead, crash wall behind you, warehouses to the", "right, forest on the hills. Default: towns to the left.",
            "New York: Times Square up 7th Avenue, straight ahead."};
        int n = sizeof lines / sizeof lines[0];
        hud_panel(16, 66, 420, 20 + n * 21);
        for (int i = 0; i < n; i++) txt(lines[i], 30, 76 + i * 21, 16, i < 11 ? RAYWHITE : HUD_DIM);
    }
}

// Flight instruments while you fly the airliner.
static void draw_flight_hud(const Airframe *A, const World *w, int slowmo, int paused, int help) {
    float sw = (float)GetScreenWidth(), sh = (float)GetScreenHeight();
    const char *zn = sea_dist(A->pos.x, A->pos.z) > 0 ? "OVER THE BAY" : A->agl > 30 ? "AIRBORNE" : zone_name(A->pos, w->map);
    hud_panel(16, 16, 74 + txtw(zn, 20), 40);
    slash(26, 26, 5, 20, HUD_ORANGE); slash(35, 26, 5, 20, HUD_ORANGE);
    txt(zn, 58, 26, 20, RAYWHITE);
    const char *tr = TextFormat("%s [M]   %s [L]   %s [O]     %d FPS", map_name(w->map), ENV->name, WEATHER_NAME[weather], GetFPS());
    txt(tr, sw - 22 - txtw(tr, 15), 22, 15, HUD_DIM);

    float px = 16, py = sh - 116, pw = 560, ph = 100;
    txt("AIRLINER   [Tab] back to the car   [R] reset to the runway", px + 2, py - 22, 15, HUD_DIM);
    hud_panel(px, py, pw, ph);
    slash(px + 12, py + 16, 4, 26, HUD_ORANGE);
    float hdg = atan2f(A->fwd.z, A->fwd.x) * RAD2DEG + 90.0f;          // runway 09 points east (+x)
    while (hdg < 0) hdg += 360;
    while (hdg >= 360) hdg -= 360;
    int dmg = (int)(100.0f * airframe_damaged_beams(A) / (float)A->c.nb + 0.5f);
    const char *lab[8] = {"ALT", "V/S", "HDG", "AOA", "THR", "FLAPS", "GEAR", "DAMAGE"};
    char val[8][24];
    snprintf(val[0], 24, "%.0f ft", (A->pos.y - AP_SEA_Y - 3.0f) * 3.281f);
    snprintf(val[1], 24, "%+.0f fpm", A->vel.y * 196.85f);
    snprintf(val[2], 24, "%03.0f", hdg);
    snprintf(val[3], 24, "%.1f deg", A->aoa * RAD2DEG);
    snprintf(val[4], 24, "%.0f%%", A->thr * 100);
    snprintf(val[5], 24, "%s", A->flaps < 0.25f ? "UP" : A->flaps < 0.75f ? "TAKEOFF" : "FULL");
    snprintf(val[6], 24, "%s", A->onGround ? (A->brake > 0.5f ? "BRAKES" : "ROLLING") : "DOWN");
    snprintf(val[7], 24, "%d%%", dmg);
    for (int i = 0; i < 8; i++) {
        float x = px + 30 + (i / 4) * 265, y = py + 10 + (i % 4) * 21;
        txt(lab[i], x, y, 17, HUD_DIM);
        txt(val[i], x + 90, y, 17, (i == 7 && dmg > 3) ? (Color){240, 60, 50, 255} : HUD_ORANGE);
    }
    float bw = 230, bx = sw - bw - 16, by = sh - 84;
    hud_panel(bx, by, bw, 68);
    slash(bx + 14, by + 14, 5, 40, HUD_ORANGE);
    char spd[16];
    snprintf(spd, sizeof spd, "%.0f", A->airspeed * 1.944f);
    txt(spd, bx + bw - 76 - txtw(spd, 48), by + 10, 48, RAYWHITE);
    txt("kt", bx + bw - 62, by + 32, 20, HUD_DIM);
    char st[64];
    snprintf(st, sizeof st, "%s%s%s", slowmo ? "SLOW-MO  " : "", paused ? "PAUSED  " : "", A->wrecked ? "WRECKED - R to reset" : A->stall ? "STALL" : "");
    if (st[0]) txt(st, sw - 16 - txtw(st, 18), by - 26, 18, (A->stall || A->wrecked) ? (Color){240, 60, 50, 255} : HUD_ORANGE);
    if (help) {
        const char *lines[] = {
            "SHIFT / CTRL  throttle up / down", "S / W  pitch up / down (pull back to climb)", "A / D  roll left / right", "Q / E  rudder, and nose-wheel steering on the ground",
            "F  flaps: up / take-off / full      SPACE  wheel brakes", "C  nose camera      B  beam view      R  reset to the runway", "TAB  back to the car      H  help",
            "", "Take-off: flaps to TAKEOFF, full throttle, hold the", "centreline with Q/E, pull back gently at 135 kt.",
            "Landing: flaps FULL, about 140 kt, idle and brake."};
        int n = sizeof lines / sizeof lines[0];
        hud_panel(16, 66, 440, 20 + n * 21);
        for (int i = 0; i < n; i++) txt(lines[i], 30, 76 + i * 21, 16, i < 7 ? RAYWHITE : HUD_DIM);
    }
}

// -------------------------------------------------------- vehicle picker ----

static int pickerOpen, pickerSel;

// Side silhouette straight from the body's station table, nose to the right.
static void draw_profile(const VehicleDef *d, float cx, float cy, float scale, Color col) {
    float clear = d->spawnH - 0.06f;                      // floor height above the ground at rest (approx.)
    Vector2 pts[2 * NR_MAX + 1];
    const int NR = d->nr;
    int n = 0;
    for (int r = 0; r < NR; r++) pts[n++] = (Vector2){cx + d->st[r].x * scale, cy - (d->st[r].top + clear) * scale};
    for (int r = NR - 1; r >= 0; r--) pts[n++] = (Vector2){cx + d->st[r].x * scale, cy - (fmaxf(d->st[r].floorY, d->st[r].rkY * 0.4f) + clear) * scale};
    pts[n++] = pts[0];
    for (int i = 0; i + 1 < n; i++) DrawLineEx(pts[i], pts[i + 1], 2.5f, col);
    // Glass: the top line across glazed gaps.
    for (int r = 0; r + 1 < NR; r++)
        if (d->st[r].topPanel >= 0 || d->st[r].sidePanel >= 0) {
            Vector2 a = {cx + d->st[r].x * scale, cy - (d->st[r].top + clear - 0.04f) * scale};
            Vector2 b = {cx + d->st[r + 1].x * scale, cy - (d->st[r + 1].top + clear - 0.04f) * scale};
            Vector2 a2 = {a.x, cy - (d->st[r].belt + clear) * scale}, b2 = {b.x, cy - (d->st[r + 1].belt + clear) * scale};
            DrawTriangle(a, a2, b2, (Color){120, 170, 220, 90}); DrawTriangle(a, b2, b, (Color){120, 170, 220, 90});
        }
    for (int w = 0; w < (d->axleR2 >= 0 ? 3 : 2); w++) {
        float x = d->st[w == 2 ? d->axleR2 : w ? d->axleR : d->axleF].x;
        Vector2 c = {cx + x * scale, cy - d->wheelR * scale};
        DrawCircleV(c, d->wheelR * scale, (Color){20, 22, 26, 255});
        DrawCircleLinesV(c, d->wheelR * scale, col);
        DrawCircleLinesV(c, d->wheelR * scale * 0.6f, (Color){col.r, col.g, col.b, 140});
    }
    DrawLineEx((Vector2){cx - 3.2f * scale, cy}, (Vector2){cx + 3.2f * scale, cy}, 1.0f, HUD_LINE);
}

static void draw_picker(void) {
    float sw = (float)GetScreenWidth(), sh = (float)GetScreenHeight();
    DrawRectangle(0, 0, (int)sw, (int)sh, (Color){0, 0, 0, 120});
    const char *title = "CHOOSE YOUR VEHICLE";
    slash(sw * 0.5f - txtw(title, 30) * 0.5f - 34, sh * 0.5f - 238, 6, 30, HUD_ORANGE);
    txt(title, sw * 0.5f - txtw(title, 30) * 0.5f, sh * 0.5f - 238, 30, RAYWHITE);
    float cw = 420, ch = 380, gap = 30, x0 = sw * 0.5f - (VEH_COUNT * cw + (VEH_COUNT - 1) * gap) * 0.5f, y0 = sh * 0.5f - 180;
    for (int i = 0; i < VEH_COUNT; i++) {
        const VehicleDef *d = vehicle_def(i);
        float x = x0 + i * (cw + gap);
        int sel = i == pickerSel;
        hud_panel(x, y0, cw, ch);
        if (sel) { DrawRectangleLinesEx((Rectangle){x - 1, y0 - 1, cw + 2, ch + 2}, 3, HUD_ORANGE); }
        int NR = d->nr;
        float scale = fminf(64.0f, (cw - 60) / (d->st[0].x - d->st[NR - 1].x));
        draw_profile(d, x + cw * 0.5f - 0.5f * (d->st[0].x + d->st[NR - 1].x) * scale, y0 + 150, scale, sel ? RAYWHITE : HUD_DIM);
        txt(d->name, x + 22, y0 + 172, 26, sel ? RAYWHITE : HUD_DIM);
        txt(d->blurb, x + 22, y0 + 204, 15, sel ? HUD_ORANGE : HUD_DIM);
        char line[5][64];
        snprintf(line[0], 64, "LENGTH       %.2f m", d->st[0].x - d->st[NR - 1].x);
        snprintf(line[1], 64, "WHEELBASE    %.2f m", d->st[d->axleF].x - d->st[d->axleR].x);
        snprintf(line[2], 64, "MASS         %.0f kg", d->mass);
        snprintf(line[3], 64, "POWER        %.0f hp", d->enginePower / 745.7f);
        snprintf(line[4], 64, "DRIVE        %s", d->driveFront >= 0.99f ? "FRONT" : d->driveFront <= 0.01f ? "REAR" : "ALL WHEELS");
        for (int k = 0; k < 5; k++) txt(line[k], x + 22, y0 + 236 + k * 26, 17, sel ? RAYWHITE : HUD_DIM);
    }
    const char *hint = "LEFT / RIGHT  choose      ENTER  drive it      ESC  cancel";
    txt(hint, sw * 0.5f - txtw(hint, 16) * 0.5f, y0 + ch + 24, 16, HUD_DIM);
}

// ---------------------------------------------------------------- main ----

static const V3 PALETTE[] = {{0.71f, 0.72f, 0.74f},   // bare stainless for the player
                             {0.10f, 0.35f, 0.85f}, {0.95f, 0.75f, 0.10f}, {0.15f, 0.65f, 0.30f},
                             {0.92f, 0.92f, 0.92f}, {0.55f, 0.20f, 0.75f}, {0.95f, 0.45f, 0.10f}, {0.20f, 0.20f, 0.22f}};

static int playerType = VEH_PICKUP;
static V3 player_paint(void) { return vehicle_def(playerType)->paint; }

// Start: facing the ramp on Default; facing up 7th Avenue toward Times Square on New York.
static void spawn_player(World *w) {
    skids_forget(0);
    // In the right-hand lane, clear of the junction (traffic turns through junctions).
    float x = w->map == MAP_NEWYORK ? -4.0f : w->map == MAP_AIRPORT ? -3.5f : -20.0f;
    float z = w->map == MAP_NEWYORK ? 15.0f : w->map == MAP_AIRPORT ? 16.0f : 3.5f;
    float yaw = w->map == MAP_DEFAULT ? 0.0f : -PI / 2;
    car_build_vehicle(&w->cars[0], playerType, v3(x, terrain_height(x, z) + vehicle_def(playerType)->spawnH, z), yaw, player_paint());
}

static const V3 TAXI = {0.96f, 0.74f, 0.08f};

// Build a map: world, meshes, road data for the renderer, player and parked vehicles.
static int trafficOn = 1;
#define TRAFFIC_CARS 8

static void load_map(World *w, int map, int *nextSlot) {
    traffic_clear(w);
    traffic_set_map(map);
    world_init(w, map);
    build_world_meshes(w);
    float rr[16][4]; int nh;
    int n = road_rects(rr, 16, &nh);
    post_set_roads((const float (*)[4])rr, n, nh);
    if (map_has_broadway()) post_set_broadway(TS_X - BW_DX * TS_Z, BW_DX, BW_HALF * sqrtf(1 + BW_DX * BW_DX), 0, CITY_Z0, CITY_Z1);
    else post_set_broadway(0, 0, 0, 0, 1, 0);
    for (int i = 0; i < MAX_CARS; i++) skids_forget(i);
    skidCount = skidHead = 0; shardCount = 0; dropsInit = 0;
    for (int i = 0; i < MAX_SPRAY; i++) spray[i].life = 0;
    fx_clear();
    spawn_player(w);
    if (map == MAP_AIRPORT) {                           // cabs waiting at the departures kerb
        w->ncars = 4;
        car_build_vehicle(&w->cars[1], VEH_HATCH, v3(-20.0f, vehicle_def(VEH_HATCH)->spawnH, 47.0f), 0, TAXI);
        car_build_vehicle(&w->cars[2], VEH_HATCH, v3(-12.0f, vehicle_def(VEH_HATCH)->spawnH, 47.0f), 0, TAXI);
        car_build_vehicle(&w->cars[3], VEH_PICKUP, v3(60.0f, CAR_SPAWN_H, 47.0f), 0, PALETTE[2]);
    } else if (map == MAP_NEWYORK) {                    // yellow cabs parked along 7th Avenue, and a pickup
        w->ncars = 4;
        car_build_vehicle(&w->cars[1], VEH_HATCH, v3(-6.8f, vehicle_def(VEH_HATCH)->spawnH, 24), -PI / 2, TAXI);   // at the kerb,
        car_build_vehicle(&w->cars[2], VEH_HATCH, v3(6.8f, vehicle_def(VEH_HATCH)->spawnH, 58), PI / 2, TAXI);     // clear of the lanes
        car_build_vehicle(&w->cars[3], VEH_PICKUP, v3(-6.8f, CAR_SPAWN_H, 114), -PI / 2, PALETTE[1]);
    } else {
        w->ncars = 3;
        car_build_vehicle(&w->cars[1], VEH_HATCH, v3(-5.8f, vehicle_def(VEH_HATCH)->spawnH, 26), PI / 2, PALETTE[1]);
        car_build_vehicle(&w->cars[2], VEH_PICKUP, v3(5.8f, CAR_SPAWN_H, 30), -PI / 2, PALETTE[2]);
    }
    for (int i = 1; i < w->ncars; i++) w->cars[i].ctl = (Controls){0, 0.6f, 0, 1};
    if (trafficOn) traffic_init(w, TRAFFIC_CARS, w->cars[0].pos);
    *nextSlot = w->ncars;
}

static float wrap_pi(float a) { while (a > PI) a -= 2 * PI; while (a < -PI) a += 2 * PI; return a; }

int main(void) {
    set_env(0);
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
    InitWindow(1440, 900, "BeamLite — soft-body driving");
    SetExitKey(KEY_NULL);
    SetTargetFPS(120);

    post_init();
    hud_load_font();
    init_spray();

    static World world;
    int nextSlot = 3;
    load_map(&world, MAP_DEFAULT, &nextSlot);

    Camera3D cam = {0};
    cam.up = (Vector3){0, 1, 0};
    cam.fovy = 62;
    cam.projection = CAMERA_PERSPECTIVE;
    float camHeading = 0, camYawOff = 0, camPitch = 0.28f, camDist = 10.5f, idleOrbit = 0;
    int camMode = 0, beamView = 0, paused = 0, slowmo = 0, help = 1, fancy = 1;
    Vector3 camPos = {-10, 4, 0};

    const float H = 1.0f / SIM_HZ;
    float acc = 0;

    while (!WindowShouldClose()) {
        float dt = fminf(GetFrameTime(), 1.0f / 20.0f);
        Car *pc = &world.cars[0];
        Airframe *ap = world.flyable >= 0 ? &world.air[world.flyable] : NULL;
        if (!ap) flying = 0;

        // ---- input
        if (IsKeyPressed(KEY_TAB) && ap && !pickerOpen) {      // hop between the car and the airliner on the runway
            flying = !flying;
            ap->pilot = flying;
            if (!flying) { ap->thr = 0; ap->pitch = ap->roll = ap->yaw = 0; ap->brake = 1; }
            camYawOff = 0;
        }
        if (IsKeyPressed(KEY_R)) { if (flying) { world_respawn_plane(&world); ap->thr = 0; } else spawn_player(&world); }
        if (IsKeyPressed(KEY_Q) && !flying) {       // recover: upright where you are, keep heading, fresh car
            float hdg = atan2f(pc->fwd.z, pc->fwd.x);
            V3 p = pc->pos;
            skids_forget(0);
            car_build_vehicle(pc, playerType, v3(p.x, terrain_height(p.x, p.z) + vehicle_def(playerType)->spawnH + 0.2f, p.z), -hdg, player_paint());
        }
        if (IsKeyPressed(KEY_N) && !flying) {       // drop a parked truck ahead, side-on (unless that's inside a building)
            float hdg = atan2f(pc->fwd.z, pc->fwd.x);
            V3 p = vadd(pc->pos, vscale(vnorm(v3(pc->fwd.x, 0, pc->fwd.z)), 20.0f));
            p.y = terrain_height(p.x, p.z);
            if (!world_point_solid(&world, vadd(p, v3(0, 1.0f, 0)))) {
                int slot = world.ncars < MAX_CARS ? world.ncars++ : nextSlot;
                if (slot == nextSlot) nextSlot = nextSlot + 1 >= MAX_CARS ? 1 : nextSlot + 1;
                int ty = slot % VEH_COUNT;
                traffic_forget_slot(slot);
                skids_forget(slot);
                car_build_vehicle(&world.cars[slot], ty, v3(p.x, p.y + vehicle_def(ty)->spawnH + 0.1f, p.z), -hdg + PI / 2, PALETTE[slot % 8]);
                world.cars[slot].ctl = (Controls){0, 0.6f, 0, 1};
            }
        }
        if (IsKeyPressed(KEY_X)) { traffic_clear(&world); trafficOn = 0; world.ncars = 1; nextSlot = 1; }
        if (IsKeyPressed(KEY_J)) {
            trafficOn = !trafficOn;
            if (trafficOn) traffic_init(&world, TRAFFIC_CARS, pc->pos); else traffic_clear(&world);
            nextSlot = world.ncars;
        }
        if (IsKeyPressed(KEY_M)) { flying = 0; load_map(&world, (world.map + 1) % MAP_COUNT, &nextSlot); ap = world.flyable >= 0 ? &world.air[world.flyable] : NULL; }
        if (IsKeyPressed(KEY_B)) beamView = !beamView;
        if (IsKeyPressed(KEY_C)) camMode = !camMode;
        if (IsKeyPressed(KEY_P)) paused = !paused;
        if (IsKeyPressed(KEY_T)) slowmo = !slowmo;
        if (IsKeyPressed(KEY_H) || IsKeyPressed(KEY_F1)) help = !help;
        if (IsKeyPressed(KEY_F2)) fancy = !fancy;           // sun shadows + ambient occlusion
        if (IsKeyPressed(KEY_K)) { skidCount = 0; skidHead = 0; }
        if (IsKeyPressed(KEY_G) && !flying) car_pop_tailgate(pc);
        if (IsKeyPressed(KEY_O)) {
            int was = weather == 3;
            weather = (weather + 1) % WEATHER_COUNT;
            dropsInit = 0;                               // reseed precipitation for the new weather
            if ((weather == 3) != was) { snowCover = weather == 3 ? 1.0f : 0.0f; build_world_meshes(&world); }
        }
        if (IsKeyPressed(KEY_V) && !flying) { pickerOpen = !pickerOpen; pickerSel = playerType; }
        else if (pickerOpen) {
            if (IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_A)) pickerSel = (pickerSel + VEH_COUNT - 1) % VEH_COUNT;
            if (IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_D)) pickerSel = (pickerSel + 1) % VEH_COUNT;
            if (IsKeyPressed(KEY_ESCAPE)) pickerOpen = 0;
            if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) {      // swap in place, keeping the heading
                playerType = pickerSel; pickerOpen = 0;
                float hdg = atan2f(pc->fwd.z, pc->fwd.x);
                V3 p = pc->pos;
                skids_forget(0);
                car_build_vehicle(pc, playerType, v3(p.x, terrain_height(p.x, p.z) + vehicle_def(playerType)->spawnH + 0.2f, p.z), -hdg, player_paint());
            }
        }
        if (IsKeyPressed(KEY_L)) { set_env((envIdx + 1) % (int)(sizeof ENVS / sizeof ENVS[0])); build_world_meshes(&world); }

        int up = IsKeyDown(KEY_W) || IsKeyDown(KEY_UP);
        int down = IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN);
        int left = IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT);
        int right = IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT);
        float vF = vdot(pc->vel, pc->fwd);
        Controls ctl = {0, 0, (float)(left - right), IsKeyDown(KEY_SPACE)};
        if (up) { if (vF < -1.0f) ctl.brake = 1; else ctl.throttle = 1; }
        if (down) { if (vF > 1.0f) ctl.brake = 1; else ctl.throttle = -1; }
        if (pickerOpen) ctl = (Controls){0, 0, 0, 1};
        if (flying) {
            // Stick, rudder, throttle lever, flaps and brakes go to the airframe; the car waits with its handbrake on.
            ctl = (Controls){0, 0, 0, 1};
            ap->pitch = (float)(down - up);
            ap->roll = (float)(right - left);
            ap->yaw = (float)(IsKeyDown(KEY_E) - IsKeyDown(KEY_Q));
            ap->thr = fminf(fmaxf(ap->thr + 0.5f * dt * (float)((IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) - (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL))), 0.0f), 1.0f);
            if (IsKeyPressed(KEY_F)) ap->flaps = ap->flaps < 0.25f ? 0.5f : ap->flaps < 0.75f ? 1.0f : 0.0f;
            ap->brake = IsKeyDown(KEY_SPACE) ? 1.0f : 0.0f;
            if (ap->c.exploded) { world_respawn_plane(&world); ap->thr = 0; }
        }
        pc->ctl = ctl;

        traffic_update(&world, paused ? 0.0f : dt * (slowmo ? 0.2f : 1.0f), pc->pos);

        // ---- physics (fixed substeps)
        if (!paused) acc += dt * (slowmo ? 0.2f : 1.0f);
        int steps = 0;
        while (acc >= H && steps < 80) { world_substep(&world, H); acc -= H; steps++; }
        if (steps >= 80) acc = 0;
        for (int i = 0; i < world.ncars; i++) {
            Car *c = &world.cars[i];
            car_update_frame(c);
            car_update_damage(c);
            if (c->exploded) {
                if (i == 0) spawn_player(&world);
                else c->active = 0;
            }
        }
        for (int i = 0; i < world.ncars; i++)
            if (world.cars[i].glassNew) { spawn_glass(&world.cars[i], world.cars[i].glassNew); world.cars[i].glassNew = 0; }
        update_shards(paused ? 0.0f : dt * (slowmo ? 0.2f : 1.0f));
        if (steps) update_skids(&world);
        refresh_tree_patches(&world);
        for (int i = 0; i < world.ncars; i++) world.cars[i].grip = weather_grip();
        snowTracks = weather == 3;
        {
            float sdt = paused ? 0.0f : dt * (slowmo ? 0.2f : 1.0f);
            if (weather >= 2) update_rain(v3(camPos.x, camPos.y, camPos.z), sdt);
            emit_spray(&world, sdt);
            update_spray(sdt);
            emit_crash_fx(&world, sdt);
            update_fx(sdt);
        }

        // ---- camera
        Vector2 md = GetMouseDelta();
        int dragging = IsMouseButtonDown(MOUSE_BUTTON_LEFT) || IsMouseButtonDown(MOUSE_BUTTON_RIGHT);
        if (dragging) { camYawOff -= md.x * 0.006f; camPitch = fminf(fmaxf(camPitch + md.y * 0.004f, 0.02f), 1.4f); idleOrbit = 0; }
        else if ((idleOrbit += dt) > 2.5f) camYawOff *= 1.0f - fminf(dt * 1.5f, 1.0f);
        camDist = fminf(fmaxf(camDist * (1.0f - GetMouseWheelMove() * 0.08f), 3.5f), 40.0f);

        V3 tgt = vadd(pc->pos, v3(0, 0.8f, 0));
        float hdg = atan2f(pc->fwd.z, pc->fwd.x);
        float hRate = fminf(1.0f, dt * (2.0f + fabsf(vF) * 0.08f));
        if (flying) {
            airframe_update_frame(ap);
            tgt = vadd(ap->pos, v3(0, 2.5f, 0));
            hdg = atan2f(ap->fwd.z, ap->fwd.x);
            hRate = fminf(1.0f, dt * 3.0f);
        }
        camHeading += wrap_pi(hdg - camHeading) * hRate;
        if (flying && camMode) {                         // nose camera
            V3 eye = vadd(ap->c.n[ap->noseNode].p, vadd(vscale(ap->up, 1.3f), vscale(ap->fwd, 0.4f)));
            V3 look = vrot(ap->fwd, ap->up, camYawOff);
            look = vrot(look, vnorm(vcross(look, ap->up)), -(camPitch - 0.28f) * 0.8f);
            cam.position = RV(eye);
            cam.target = RV(vadd(eye, vscale(look, 10.0f)));
            cam.up = RV(ap->up);
            camPos = cam.position;
        } else if (flying) {                             // chase: well back, rigidly behind (no lag at 250 kt)
            float a = camHeading + PI + camYawOff, dist = camDist * 4.6f;
            V3 want = vadd(tgt, v3(cosf(a) * cosf(camPitch) * dist, sinf(camPitch) * dist, sinf(a) * cosf(camPitch) * dist));
            float gh = fmaxf(terrain_height(want.x, want.z), AP_SEA_Y) + 1.0f;
            if (want.y < gh) want.y = gh;
            camPos = RV(want);
            cam.position = camPos;
            cam.target = RV(tgt);
            cam.up = (Vector3){0, 1, 0};
        } else if (camMode == 0) {
            float a = camHeading + PI + camYawOff;
            V3 want = vadd(tgt, v3(cosf(a) * cosf(camPitch) * camDist, sinf(camPitch) * camDist + 0.8f, sinf(a) * cosf(camPitch) * camDist));
            float gh = terrain_height(want.x, want.z) + 0.5f;
            if (want.y < gh) want.y = gh;
            // Don't let buildings get between the camera and the truck.
            V3 dir = vsub(want, tgt);
            float L = vlen(dir), k = fminf(1.0f, dt * 10.0f);
            for (float d = 0.8f; d < L; d += 0.4f)
                if (world_point_solid(&world, vadd(tgt, vscale(dir, d / L)))) {
                    want = vadd(tgt, vscale(dir, fmaxf(d - 0.7f, 1.5f) / L));
                    k = fminf(1.0f, dt * 25.0f);
                    break;
                }
            camPos = (Vector3){camPos.x + (want.x - camPos.x) * k, camPos.y + (want.y - camPos.y) * k, camPos.z + (want.z - camPos.z) * k};
            cam.position = camPos;
            cam.target = RV(tgt);
            cam.up = (Vector3){0, 1, 0};
        } else {                                         // dash cam: the driver's eye point, fixed to the body
            V3 eye = car_local_to_world(pc, pc->def->seat);
            V3 look = vrot(pc->fwd, pc->up, camYawOff);       // mouse drag looks around the cabin
            look = vrot(look, vnorm(vcross(look, pc->up)), -(camPitch - 0.28f) * 0.8f - 0.05f);
            cam.position = RV(eye);
            cam.target = RV(vadd(eye, vscale(look, 10.0f)));
            cam.up = RV(pc->up);
            camPos = cam.position;
        }
        cam.fovy = camMode ? 72.0f : 62.0f;

        // ---- sun shadow map: the casters, seen from the sun
        post_set_quality(fancy, fancy);
        if (post_shadow_begin(flying ? RV(ap->pos) : RV(pc->pos), ENV, RV(AMB), RV(SUNC))) {
            depthOnly = 1;
            DrawModel(city, (Vector3){0, 0, 0}, 1.0f, WHITE);
            draw_trees(&world, cam.position);
            V3 eyeS = v3(cam.position.x, cam.position.y, cam.position.z);
            for (int i = 0; i < world.nplanes; i++) draw_airframe(&world.air[i], 0, eyeS);
            for (int i = 0; i < world.ncars; i++) if (world.cars[i].active) draw_car(&world.cars[i], 0, eyeS, 0);
            depthOnly = 0;
            post_shadow_end();
        }

        // ---- draw the world into the HDR buffer
        post_begin_scene();
        BeginMode3D(cam);
        post_capture_camera(cam);
        for (int i = 0; i < CH_N * CH_N; i++) DrawModel(chunks[i], (Vector3){0, 0, 0}, 1.0f, WHITE);
        for (int i = 0; i < nExt; i++) DrawModel(extChunk[i], (Vector3){0, 0, 0}, 1.0f, WHITE);
        if (hasFar) DrawModel(farTerrain, (Vector3){0, 0, 0}, 1.0f, WHITE);
        if (hasMarkings) DrawModel(markings, (Vector3){0, 0, 0}, 1.0f, WHITE);
        rlDisableBackfaceCulling();
        post_begin_objects();
        if (hasFar) {                                    // the bay
            post_water_begin(ENV, RV(AMB), RV(SUNC), (float)GetTime());
            const float E = AP_FAR * 1.5f;
            rlBegin(RL_TRIANGLES);
            rlColor4ub(255, 255, 255, 255); rlNormal3f(0, 1, 0);
            rlVertex3f(-E, AP_SEA_Y, -E); rlVertex3f(-E, AP_SEA_Y, E); rlVertex3f(E, AP_SEA_Y, E);
            rlVertex3f(-E, AP_SEA_Y, -E); rlVertex3f(E, AP_SEA_Y, E); rlVertex3f(E, AP_SEA_Y, -E);
            rlEnd();
            post_water_end();
        }
        DrawModel(city, (Vector3){0, 0, 0}, 1.0f, WHITE);
        if (hasScreens) { post_set_billboards((float)GetTime(), 1.4f + 1.6f * ENV->lights); DrawModel(screens, (Vector3){0, 0, 0}, 1.0f, WHITE); }
        draw_trees(&world, cam.position);
        post_end_objects();
        rlDrawRenderBatchActive();
        rlDisableDepthMask();                      // see-through decals: no depth writes, so overlaps just add up
        draw_skids();
        draw_tree_shadows();
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        post_begin_objects();
        if (!beamView) { post_paint_begin(ENV, RV(AMB), RV(SUNC), (float)GetTime()); pbrOn = 1; }
        rlTexCoord2f(0, 0);
        for (int i = 0; i < world.nplanes; i++) draw_airframe(&world.air[i], beamView, v3(cam.position.x, cam.position.y, cam.position.z));
        for (int i = 0; i < world.ncars; i++) if (world.cars[i].active) draw_car(&world.cars[i], beamView, v3(cam.position.x, cam.position.y, cam.position.z), camMode && i == 0);
        if (pbrOn) { pbrOn = 0; post_paint_end(); }
        rlNormal3f(0, 0, 1);
        post_end_objects();
        draw_shards();
        rlDrawRenderBatchActive();
        rlDisableDepthMask();
        draw_spray(cam);
        draw_fx(cam, 0);
        BeginBlendMode(BLEND_ADDITIVE);
        draw_fx(cam, 1); draw_fx(cam, 1);                 // twice over: bright enough to bloom
        EndBlendMode();
        if (weather == 2) draw_rain(v3(cam.position.x, cam.position.y, cam.position.z));
        if (weather == 3) draw_snow(cam);
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        // Things that glow, then additive light pools (no depth writes so they don't occlude).
        int lightsOn = ENV->lights > 0.01f;
        if (lightsOn) { post_set_emissive(0.6f + 2.2f * ENV->lights); DrawModel(cityGlow, (Vector3){0, 0, 0}, 1.0f, WHITE); }
        if (!beamView) for (int i = 0; i < world.ncars; i++) if (world.cars[i].active) draw_car_lights(&world.cars[i], lightsOn && (i == 0 || traffic_is_ai(i)));
        if (camMode && !beamView) { post_emissive_begin(0.6f + 1.4f * ENV->lights); draw_gauges(pc); post_emissive_end(); }
        post_emissive_begin(3.5f); draw_signal_lamps(1); post_emissive_end();
        if (world.flyable >= 0 && !beamView) draw_airliner_lights(&world.air[world.flyable], (float)GetTime(), lightsOn || flying);
        post_emissive_begin(0.5f); draw_signal_lamps(0); post_emissive_end();
        if (lightsOn) {
            rlDrawRenderBatchActive();
            rlDisableDepthMask();
            BeginBlendMode(BLEND_ADDITIVE);
            post_set_emissive(ENV->lights);
            DrawModel(cityPool, (Vector3){0, 0, 0}, 1.0f, WHITE);
            post_emissive_begin(ENV->lights);
            draw_headlights(pc);
            post_emissive_end();
            EndBlendMode();
            rlEnableDepthMask();
        }
        rlEnableBackfaceCulling();
        EndMode3D();
        post_end_scene();

        // ---- grade it onto the screen, then the HUD
        BeginDrawing();
        Env env = *ENV;
        if (weather == 3) {                              // overcast: flatter, greyer sky and thicker haze
            float day = 1.0f - ENV->lights;
            Vector3 grey = {0.16f + 0.48f * day, 0.18f + 0.50f * day, 0.24f + 0.50f * day};
            env.zenith = Vector3Lerp(env.zenith, Vector3Scale(grey, 0.75f), 0.7f);
            env.horizon = Vector3Lerp(env.horizon, grey, 0.75f);
            env.horizonAway = Vector3Lerp(env.horizonAway, grey, 0.75f);
            env.fogCol = Vector3Lerp(env.fogCol, Vector3Scale(grey, 0.9f), 0.8f);
            env.sunCol = Vector3Scale(env.sunCol, 0.35f);
            env.fogDensity *= 2.2f;
        }
        post_present(&env, (float)GetTime(), weather_wet(), weather_rain());
        if (flying) draw_flight_hud(ap, &world, slowmo, paused, help);
        else draw_hud(pc, &world, vF, pickerOpen ? (Controls){0} : ctl, slowmo, paused, help);
        if (pickerOpen) draw_picker();
        EndDrawing();
    }

    for (int i = 0; i < CH_N * CH_N; i++) UnloadModel(chunks[i]);
    UnloadModel(city); UnloadModel(cityGlow); UnloadModel(cityPool);
    UnloadModel(treeModel[0]); UnloadModel(treeModel[1]); UnloadModel(stumpModel);
    post_shutdown();
    CloseWindow();
    return 0;
}
