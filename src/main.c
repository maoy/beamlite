// BeamLite — a tiny soft-body driving sandbox in the spirit of BeamNG.drive.
// Rendering/input via raylib; physics in physics.c.

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"
#include "physics.h"
#include "render.h"
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
    else if (fabsf(x) < 8 && z > 40 && z < 140) { r = 0.55f; g = 0.42f; b = 0.28f; cover = 0.8f; } // dirt
    else if (road_at(x, z) == 1) { r = g = b = 0.30f + 0.04f * noise; cover = 0.18f + 0.22f * noise; }   // highway (ploughed)
    else if (road_at(x, z) == 2) { r = g = b = 0.27f + 0.03f * noise; cover = 0.25f + 0.25f * noise; }   // town street
    else if (road_at(x + 2.2f, z) == 2 || road_at(x - 2.2f, z) == 2 || road_at(x, z + 2.2f) == 2 || road_at(x, z - 2.2f) == 2) {
        r = 0.50f; g = 0.49f; b = 0.47f; cover = 0.85f;                                          // pavement
    }
    else {                                                                                       // grass / hills
        r = 0.26f + 0.10f * noise; g = 0.47f + 0.12f * noise; b = 0.20f;
        float rock = fminf(fmaxf((1.0f - n.y) * 4.0f, 0), 1);
        r += (0.50f - r) * rock; g += (0.47f - g) * rock; b += (0.42f - b) * rock;
        cover = 1.0f - 0.8f * rock;                                                              // steep rock sheds snow
        (void)y;
    }
    int gx = (int)lroundf(x / CELL), gz = (int)lroundf(z / CELL);
    if (gx % 5 == 0 || gz % 5 == 0) { r *= 0.93f; g *= 0.93f; b *= 0.93f; }  // 10 m grid for speed sense
    Color base = {(unsigned char)fminf(255, r * 255), (unsigned char)fminf(255, g * 255), (unsigned char)fminf(255, b * 255), 255};
    base = snow_over(base, snowCover * cover * (0.92f + 0.08f * noise));
    return lit(base, n, 1.0f);
}

static Model build_chunk(int cx, int cz) {
    const int V = CH_CELLS + 1;
    float x0 = -CH_N * CH_CELLS * CELL * 0.5f + cx * CH_CELLS * CELL;
    float z0 = -CH_N * CH_CELLS * CELL * 0.5f + cz * CH_CELLS * CELL;
    Mesh m = {0};
    m.vertexCount = V * V;
    m.triangleCount = CH_CELLS * CH_CELLS * 2;
    m.vertices = MemAlloc(m.vertexCount * 3 * sizeof(float));
    m.normals = MemAlloc(m.vertexCount * 3 * sizeof(float));
    m.colors = MemAlloc(m.vertexCount * 4);
    m.indices = MemAlloc(m.triangleCount * 3 * sizeof(unsigned short));
    for (int j = 0; j < V; j++)
    for (int i = 0; i < V; i++) {
        int v = j * V + i;
        float x = x0 + i * CELL, z = z0 + j * CELL, y = terrain_height(x, z);
        V3 n = terrain_normal(x, z);
        Color c = terrain_color(x, z, y, n);
        m.vertices[v * 3] = x; m.vertices[v * 3 + 1] = y; m.vertices[v * 3 + 2] = z;
        m.normals[v * 3] = n.x; m.normals[v * 3 + 1] = n.y; m.normals[v * 3 + 2] = n.z;
        m.colors[v * 4] = c.r; m.colors[v * 4 + 1] = c.g; m.colors[v * 4 + 2] = c.b; m.colors[v * 4 + 3] = 255;
    }
    int t = 0;
    for (int j = 0; j < CH_CELLS; j++)
    for (int i = 0; i < CH_CELLS; i++) {
        unsigned short a = j * V + i, b = a + 1, c = a + V, d = c + 1;
        m.indices[t++] = a; m.indices[t++] = c; m.indices[t++] = b;   // CCW seen from above
        m.indices[t++] = b; m.indices[t++] = c; m.indices[t++] = d;
    }
    UploadMesh(&m, false);
    return LoadModelFromMesh(m);
}

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

// A window: glass, plus a warm glowing pane if someone is home.
static float glowK = 1.0f;                  // brightness of lit panes (shopfronts are big, so dimmer)
static void window(MeshB *m, const Building *B, int s, float u, float y, float w, float h, float litP, Color glass) {
    facade_rect(m, B, s, u, y, w, h, 0.04f, glass);
    if (brnd() < litP) { mbFlat = 1; facade_rect(&GLOW, B, s, u + 0.06f, y + 0.06f, w - 0.12f, h - 0.12f, 0.05f, scale_col(WARM, glowK * (0.75f + 0.25f * brnd()))); mbFlat = 0; }
}

static void build_building(MeshB *m, const Building *B) {
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
    place_street_lamps(&m, w);
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
    for (int i = 0; i < TCH * TCH; i++) if (patchHas[i]) DrawModel(shadowPatch[i], (Vector3){0, 0, 0}, 1.0f, WHITE);
}

// ---------------------------------------------------------- skid marks ----

#define MAX_SKID 6000
typedef struct { Vector3 a, b, c, d; } Skid;
static Skid skids[MAX_SKID];
static int skidCount, skidHead;
static V3 skidLast[MAX_CARS][4];
static int skidHas[MAX_CARS][4];

static int snowTracks;                          // set while snowing: all rolling wheels leave tracks

static void skids_forget(int ci) { for (int i = 0; i < 4; i++) skidHas[ci][i] = 0; }

static void update_skids(World *w) {
    for (int ci = 0; ci < w->ncars; ci++) {
        Car *c = &w->cars[ci];
        for (int i = 0; i < 4; i++) {
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

// Paint shading for one point: baked-style sun/ambient, a sky reflection that grows toward
// grazing angles (clear coat), and a sun highlight.
static Color shade_paint(Color base, V3 n, V3 p, V3 eye, float gloss) {
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
static void draw_obox(V3 c, V3 ax, V3 ay, V3 az, Color col, V3 eye) {
    V3 axes[3] = {ax, ay, az};
    for (int a = 0; a < 3; a++)
        for (int sgn = -1; sgn <= 1; sgn += 2) {
            V3 n = vscale(axes[a], (float)sgn), u = axes[(a + 1) % 3], w = axes[(a + 2) % 3];
            V3 fc = vadd(c, n);
            V3 q0 = vsub(vsub(fc, u), w), q1 = vsub(vadd(fc, u), w), q2 = vadd(vadd(fc, u), w), q3 = vadd(vsub(fc, u), w);
            Color k = shade_paint(col, vnorm(n), fc, eye, 0.6f);
            DrawTriangle3D(RV(q0), RV(q1), RV(q2), k); DrawTriangle3D(RV(q0), RV(q2), RV(q3), k);
        }
}

static V3 nodeN[MAX_NODES];

static void draw_car(const Car *c, int beamView, V3 eye) {
    Color body = {(unsigned char)(c->color.x * 255), (unsigned char)(c->color.y * 255), (unsigned char)(c->color.z * 255), 255};
    Color glass = {34, 46, 60, 255}, under = {35, 35, 38, 255}, tonneau = {52, 54, 58, 255}, trim = {30, 31, 33, 255};

    // Blob shadow: project the underside onto the ground.
    Color sh = {0, 0, 0, 90};
    rlDrawRenderBatchActive();
    rlDisableDepthMask();
    for (int i = 0; i < c->nt; i++) {
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
            for (int k = 0; k < 3; k++) {
                V3 p = c->n[t->v[k]].p;
                Color col = shade_paint(base, nodeN[t->v[k]], p, eye, gloss);
                rlColor4ub(col.r, col.g, col.b, 255);
                rlVertex3f(p.x, p.y, p.z);
            }
        }
        rlEnd();
    }
    if (!beamView) {
        for (int i = 0; i < c->nt; i++) {
            const Tri *t = &c->t[i];
            if (t->panel >= 0 && c->glassBroken[t->panel]) continue;
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
            Color base = t->kind == SURF_GLASS ? glass : t->kind == SURF_UNDER ? under :
                         t->kind == SURF_TONNEAU ? tonneau : t->kind == SURF_TRIM ? trim : body;
            Color col = lit(base, nrm, 1.0f);
            if (t->kind == SURF_BODY) {                            // brushed steel picks up the sky
                float f = 0.18f + 0.22f * fmaxf(nrm.y, 0.0f);
                Color sk = {(unsigned char)(255 * powf(ENV->horizonAway.x, 0.4545f)), (unsigned char)(255 * powf(ENV->horizonAway.y, 0.4545f)), (unsigned char)(255 * powf(ENV->horizonAway.z, 0.4545f)), 255};
                col = (Color){(unsigned char)(col.r + (sk.r - col.r) * f), (unsigned char)(col.g + (sk.g - col.g) * f), (unsigned char)(col.b + (sk.b - col.b) * f), 255};
            }
            if (t->kind == SURF_BODY || t->kind == SURF_GLASS) {   // flat-faceted stainless / glass highlight
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

    // Wheels.
    for (int i = 0; i < 4; i++) {
        const Wheel *W = &c->w[i];
        V3 axis = vrot(c->right, c->up, W->steerAngle);
        float out = (i % 2 == 0) ? -1.0f : 1.0f;              // left wheels sit at -right
        V3 ctr = W->hub;
        V3 a = vsub(ctr, vscale(axis, c->def->wheelW * 0.5f)), b = vadd(ctr, vscale(axis, c->def->wheelW * 0.5f));
        DrawCylinderEx(RV(a), RV(b), c->def->wheelR, c->def->wheelR, 24, (Color){26, 26, 28, 255});
        V3 face = vadd(ctr, vscale(axis, out * (c->def->wheelW * 0.5f + 0.004f)));
        if (c->def->wheelStyle == 1) {                 // dark alloy laid over the tyre's outer face: barrel, ten spokes, lip, cap
            float R = c->def->wheelR;
            V3 fw = vcross(c->up, axis), n = vscale(axis, out);
            V3 f1 = vadd(face, vscale(n, 0.002f)), f2 = vadd(face, vscale(n, 0.005f)), f3 = vadd(face, vscale(n, 0.008f));
            DrawCylinderEx(RV(face), RV(f1), R * 0.74f, R * 0.74f, 28, (Color){70, 72, 78, 255});      // rim lip
            DrawCylinderEx(RV(f1), RV(vadd(f1, vscale(n, 0.001f))), R * 0.69f, R * 0.69f, 28, (Color){12, 12, 14, 255});   // barrel shadow
            Color sp = brighten(lit((Color){128, 130, 138, 255}, n, 1.0f), 0.06f);
            for (int k = 0; k < 10; k++) {
                float ang = W->spin + (k >> 1) * 1.25664f + (k & 1 ? 0.16f : -0.16f);
                V3 d = vrot(fw, axis, ang), sd = vcross(axis, d);
                float w0 = R * 0.055f, w1 = R * 0.030f;
                V3 r0 = vadd(f2, vscale(d, R * 0.13f)), r1 = vadd(f2, vscale(d, R * 0.69f));
                V3 q0 = vsub(r0, vscale(sd, w0)), q1 = vadd(r0, vscale(sd, w0)), q2 = vadd(r1, vscale(sd, w1)), q3 = vsub(r1, vscale(sd, w1));
                DrawTriangle3D(RV(q0), RV(q1), RV(q2), sp); DrawTriangle3D(RV(q0), RV(q2), RV(q3), sp);
            }
            DrawCylinderEx(RV(f2), RV(f3), R * 0.17f, R * 0.14f, 14, (Color){40, 40, 46, 255});         // centre cap
            continue;
        }
        DrawCylinderEx(RV(vsub(face, vscale(axis, out * 0.01f))), RV(face), c->def->wheelR * 0.66f, c->def->wheelR * 0.66f, 6, (Color){150, 152, 156, 255});
        V3 fw = vcross(c->up, axis);
        for (int s = 0; s < 6; s++) {   // cover segments show rotation
            V3 dir = vrot(fw, axis, W->spin + s * 1.0472f);
            V3 f0 = vadd(face, vscale(axis, out * 0.004f));
            DrawLine3D(RV(f0), RV(vadd(f0, vscale(dir, c->def->wheelR * 0.66f))), (Color){70, 72, 76, 255});
        }
    }
}

// Light bars, drawn in the emissive pass so they bloom.
static void draw_car_lights(const Car *c, int on) {
    float front = on ? 3.2f : 0.9f, rear = (on ? 2.2f : 1.4f) * (c->ctl.brake > 0.1f ? 2.0f : 1.0f);
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
    V3 o = vadd(c->pos, vscale(f, 2.6f));
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
        for (int i = 0; i < 4; i++) {
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
    city = build_city(w, &cityGlow, &cityPool);
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
static const char *zone_name(V3 p) {
    if (p.x > 28 && p.x < 80 && p.z < -12 && p.z > -42) return "WAREHOUSE DISTRICT";
    if (p.z > 12 && p.z < 140 && fabsf(p.x) < 116) return fabsf(p.x) < 9 ? "CROSS ROAD" : "TOWN";
    if (p.x * p.x + p.z * p.z > 100 * 100) return "HILLS";
    return "PROVING GROUND";
}

static void draw_hud(const Car *pc, const World *w, float vF, Controls ctl, int slowmo, int paused, int help) {
    float sw = (float)GetScreenWidth(), sh = (float)GetScreenHeight();
    const char *zn = zone_name(pc->pos);
    hud_panel(16, 16, 74 + txtw(zn, 20), 40);
    slash(26, 26, 5, 20, HUD_ORANGE); slash(35, 26, 5, 20, HUD_ORANGE);
    txt(zn, 58, 26, 20, RAYWHITE);
    const char *tr = TextFormat("%s [L]   %s [O]     %d FPS", ENV->name, WEATHER_NAME[weather], GetFPS());
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
            "C  hood cam      B  beam view      V  choose vehicle", "N  spawn truck ahead      X  clear trucks", "Q  recover in place      R  reset to start",
            "T  slow-mo      P  pause      K  clear skids      H  help", "G  drop tailgate      L  lighting      O  weather",
            "", "Ramp ahead, crash wall behind you. Town up the", "cross road to the left, warehouses to the right,",
            "forest on the hills. Small trees snap; big ones don't."};
        int n = sizeof lines / sizeof lines[0];
        hud_panel(16, 66, 420, 20 + n * 21);
        for (int i = 0; i < n; i++) txt(lines[i], 30, 76 + i * 21, 16, i < 8 ? RAYWHITE : HUD_DIM);
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
    for (int w = 0; w < 2; w++) {
        float x = d->st[w ? d->axleR : d->axleF].x;
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

static void spawn_player(World *w) {
    skids_forget(0);
    car_build_vehicle(&w->cars[0], playerType, v3(0, terrain_height(0, 0) + vehicle_def(playerType)->spawnH, 0), 0, player_paint());
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
    world_init(&world);
    build_world_meshes(&world);
    {
        float rr[16][4]; int nh;
        int n = road_rects(rr, 16, &nh);
        post_set_roads((const float (*)[4])rr, n, nh);
    }
    world.ncars = 3;
    spawn_player(&world);
    car_build_vehicle(&world.cars[1], VEH_HATCH, v3(-2.2f, vehicle_def(VEH_HATCH)->spawnH, 26), PI / 2, PALETTE[1]);
    car_build_vehicle(&world.cars[2], VEH_PICKUP, v3(2.2f, CAR_SPAWN_H, 30), -PI / 2, PALETTE[2]);
    for (int i = 1; i < world.ncars; i++) world.cars[i].ctl = (Controls){0, 0.6f, 0, 1};
    int nextSlot = 3;

    Camera3D cam = {0};
    cam.up = (Vector3){0, 1, 0};
    cam.fovy = 62;
    cam.projection = CAMERA_PERSPECTIVE;
    float camHeading = 0, camYawOff = 0, camPitch = 0.28f, camDist = 10.5f, idleOrbit = 0;
    int camMode = 0, beamView = 0, paused = 0, slowmo = 0, help = 1;
    Vector3 camPos = {-10, 4, 0};

    const float H = 1.0f / SIM_HZ;
    float acc = 0;

    while (!WindowShouldClose()) {
        float dt = fminf(GetFrameTime(), 1.0f / 20.0f);
        Car *pc = &world.cars[0];

        // ---- input
        if (IsKeyPressed(KEY_R)) spawn_player(&world);
        if (IsKeyPressed(KEY_Q)) {       // recover: upright where you are, keep heading, fresh car
            float hdg = atan2f(pc->fwd.z, pc->fwd.x);
            V3 p = pc->pos;
            skids_forget(0);
            car_build_vehicle(pc, playerType, v3(p.x, terrain_height(p.x, p.z) + vehicle_def(playerType)->spawnH + 0.2f, p.z), -hdg, player_paint());
        }
        if (IsKeyPressed(KEY_N)) {       // drop a parked truck ahead, side-on (unless that's inside a building)
            float hdg = atan2f(pc->fwd.z, pc->fwd.x);
            V3 p = vadd(pc->pos, vscale(vnorm(v3(pc->fwd.x, 0, pc->fwd.z)), 20.0f));
            p.y = terrain_height(p.x, p.z);
            if (!world_point_solid(&world, vadd(p, v3(0, 1.0f, 0)))) {
                int slot = world.ncars < MAX_CARS ? world.ncars++ : nextSlot;
                if (slot == nextSlot) nextSlot = nextSlot + 1 >= MAX_CARS ? 1 : nextSlot + 1;
                int ty = slot % VEH_COUNT;
                skids_forget(slot);
                car_build_vehicle(&world.cars[slot], ty, v3(p.x, p.y + vehicle_def(ty)->spawnH + 0.1f, p.z), -hdg + PI / 2, PALETTE[slot % 8]);
                world.cars[slot].ctl = (Controls){0, 0.6f, 0, 1};
            }
        }
        if (IsKeyPressed(KEY_X)) { world.ncars = 1; nextSlot = 1; }
        if (IsKeyPressed(KEY_B)) beamView = !beamView;
        if (IsKeyPressed(KEY_C)) camMode = !camMode;
        if (IsKeyPressed(KEY_P)) paused = !paused;
        if (IsKeyPressed(KEY_T)) slowmo = !slowmo;
        if (IsKeyPressed(KEY_H) || IsKeyPressed(KEY_F1)) help = !help;
        if (IsKeyPressed(KEY_K)) { skidCount = 0; skidHead = 0; }
        if (IsKeyPressed(KEY_G)) car_pop_tailgate(pc);
        if (IsKeyPressed(KEY_O)) {
            int was = weather == 3;
            weather = (weather + 1) % WEATHER_COUNT;
            dropsInit = 0;                               // reseed precipitation for the new weather
            if ((weather == 3) != was) { snowCover = weather == 3 ? 1.0f : 0.0f; build_world_meshes(&world); }
        }
        if (IsKeyPressed(KEY_V)) { pickerOpen = !pickerOpen; pickerSel = playerType; }
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
        pc->ctl = ctl;

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
        camHeading += wrap_pi(hdg - camHeading) * hRate;
        if (camMode == 0) {
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
        } else {
            V3 eye = vadd(vadd(pc->pos, vscale(pc->up, pc->def->eyeH)), vscale(pc->fwd, 0.1f));
            cam.position = RV(eye);
            cam.target = RV(vadd(eye, vscale(pc->fwd, 10.0f)));
            cam.up = RV(pc->up);
            camPos = cam.position;
        }

        // ---- draw the world into the HDR buffer
        post_begin_scene();
        BeginMode3D(cam);
        post_capture_camera(cam);
        for (int i = 0; i < CH_N * CH_N; i++) DrawModel(chunks[i], (Vector3){0, 0, 0}, 1.0f, WHITE);
        rlDisableBackfaceCulling();
        post_begin_objects();
        DrawModel(city, (Vector3){0, 0, 0}, 1.0f, WHITE);
        draw_trees(&world, cam.position);
        post_end_objects();
        rlDrawRenderBatchActive();
        rlDisableDepthMask();                      // see-through decals: no depth writes, so overlaps just add up
        draw_skids();
        draw_tree_shadows();
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        post_begin_objects();
        for (int i = 0; i < world.ncars; i++) if (world.cars[i].active) draw_car(&world.cars[i], beamView, v3(cam.position.x, cam.position.y, cam.position.z));
        post_end_objects();
        draw_shards();
        rlDrawRenderBatchActive();
        rlDisableDepthMask();
        draw_spray(cam);
        if (weather == 2) draw_rain(v3(cam.position.x, cam.position.y, cam.position.z));
        if (weather == 3) draw_snow(cam);
        rlDrawRenderBatchActive();
        rlEnableDepthMask();
        // Things that glow, then additive light pools (no depth writes so they don't occlude).
        int lightsOn = ENV->lights > 0.01f;
        if (lightsOn) { post_set_emissive(0.6f + 2.2f * ENV->lights); DrawModel(cityGlow, (Vector3){0, 0, 0}, 1.0f, WHITE); }
        if (!beamView) for (int i = 0; i < world.ncars; i++) if (world.cars[i].active) draw_car_lights(&world.cars[i], lightsOn && i == 0);
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
        draw_hud(pc, &world, vF, pickerOpen ? (Controls){0} : ctl, slowmo, paused, help);
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
