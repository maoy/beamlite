// BeamLite — a tiny soft-body driving sandbox in the spirit of BeamNG.drive.
// Rendering/input via raylib; physics in physics.c.

#include "raylib.h"
#include "rlgl.h"
#include "physics.h"
#include <stdio.h>
#include <math.h>

static Vector3 RV(V3 a) { return (Vector3){a.x, a.y, a.z}; }
static V3 SUN;

// ------------------------------------------------------------- terrain ----

#define CH_N 6          // chunks per side
#define CH_CELLS 50     // cells per chunk side
#define CELL 2.0f       // metres per cell  -> 600 m square world

static Model chunks[CH_N * CH_N];

static int on_road(float x, float z) { return (fabsf(z) < 7.0f && fabsf(x) < 260) || (fabsf(x) < 7.0f && fabsf(z) < 260); }

static Color terrain_color(float x, float z, float y, V3 n) {
    float r, g, b;
    float noise = 0.5f + 0.25f * sinf(x * 0.37f + 1.7f) * sinf(z * 0.41f) + 0.25f * sinf(x * 0.13f - z * 0.17f);
    if (x > 40 && x < 58 && fabsf(z) < 6.5f) { r = 0.72f; g = 0.66f; b = 0.55f; }               // ramp
    else if (fabsf(x) < 8 && z > 40 && z < 140) { r = 0.55f; g = 0.42f; b = 0.28f; }             // dirt
    else if (on_road(x, z)) { r = g = b = 0.30f + 0.04f * noise; }                               // asphalt
    else {                                                                                       // grass / hills
        r = 0.26f + 0.10f * noise; g = 0.47f + 0.12f * noise; b = 0.20f;
        float rock = fminf(fmaxf((1.0f - n.y) * 4.0f, 0), 1);
        r += (0.50f - r) * rock; g += (0.47f - g) * rock; b += (0.42f - b) * rock;
        (void)y;
    }
    int gx = (int)lroundf(x / CELL), gz = (int)lroundf(z / CELL);
    if (gx % 5 == 0 || gz % 5 == 0) { r *= 0.93f; g *= 0.93f; b *= 0.93f; }  // 10 m grid for speed sense
    float l = 0.42f + 0.62f * fmaxf(vdot(n, SUN), 0.0f);
    return (Color){(unsigned char)fminf(255, r * l * 255), (unsigned char)fminf(255, g * l * 255), (unsigned char)fminf(255, b * l * 255), 255};
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

// ---------------------------------------------------------- skid marks ----

#define MAX_SKID 6000
typedef struct { Vector3 a, b, c, d; } Skid;
static Skid skids[MAX_SKID];
static int skidCount, skidHead;
static V3 skidLast[MAX_CARS][4];
static int skidHas[MAX_CARS][4];

static void update_skids(World *w) {
    for (int ci = 0; ci < w->ncars; ci++) {
        Car *c = &w->cars[ci];
        for (int i = 0; i < 4; i++) {
            Wheel *W = &c->w[i];
            if (!c->active || !W->contact || W->slip < 0.12f) { skidHas[ci][i] = 0; continue; }
            V3 n = terrain_normal(W->cp.x, W->cp.z);
            V3 cur = vadd(W->cp, vscale(n, 0.03f));
            if (skidHas[ci][i]) {
                if (vlen(vsub(cur, skidLast[ci][i])) < 0.3f) continue;
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
    Color col = {20, 20, 20, 150};
    for (int i = 0; i < skidCount; i++) {
        Skid *s = &skids[i];
        DrawTriangle3D(s->a, s->c, s->b, col);
        DrawTriangle3D(s->a, s->d, s->c, col);
    }
}

// ----------------------------------------------------------------- car ----

static Color shade(Color c, float l) {
    return (Color){(unsigned char)fminf(255, c.r * l), (unsigned char)fminf(255, c.g * l), (unsigned char)fminf(255, c.b * l), c.a};
}
static Color brighten(Color c, float s) {
    return (Color){(unsigned char)fminf(255, c.r + s * 255), (unsigned char)fminf(255, c.g + s * 255), (unsigned char)fminf(255, c.b + s * 255), c.a};
}

// Emissive strip along an edge: top L/R nodes q[0..1], fading toward q[2..3] by `t`.
static void draw_lightbar(const Car *c, const int q[4], float t, float outSign, Color col) {
    V3 a = c->n[q[0]].p, b = c->n[q[1]].p;
    V3 al = vlerp(a, c->n[q[2]].p, t), bl = vlerp(b, c->n[q[3]].p, t);
    V3 off = vscale(vnorm(vcross(vsub(b, a), vsub(al, a))), outSign * 0.008f);
    a = vadd(a, off); b = vadd(b, off); al = vadd(al, off); bl = vadd(bl, off);
    DrawTriangle3D(RV(a), RV(al), RV(bl), col);
    DrawTriangle3D(RV(a), RV(bl), RV(b), col);
}

static void draw_car(const Car *c, int beamView, V3 eye) {
    Color body = {(unsigned char)(c->color.x * 255), (unsigned char)(c->color.y * 255), (unsigned char)(c->color.z * 255), 255};
    Color glass = {34, 46, 60, 255}, under = {35, 35, 38, 255}, tonneau = {52, 54, 58, 255}, trim = {30, 31, 33, 255};

    // Blob shadow: project the underside onto the ground.
    Color sh = {0, 0, 0, 70};
    for (int i = 0; i < c->nt; i++) {
        const Tri *t = &c->t[i];
        if (t->kind != SURF_UNDER) continue;
        Vector3 p[3];
        for (int k = 0; k < 3; k++) { V3 v = c->n[t->v[k]].p; p[k] = (Vector3){v.x, terrain_height(v.x, v.z) + 0.04f, v.z}; }
        DrawTriangle3D(p[0], p[2], p[1], sh); DrawTriangle3D(p[0], p[1], p[2], sh);
    }

    if (!beamView) {
        for (int i = 0; i < c->nt; i++) {
            const Tri *t = &c->t[i];
            if (t->panel >= 0 && c->glassBroken[t->panel]) continue;
            V3 p[3] = {c->n[t->v[0]].p, c->n[t->v[1]].p, c->n[t->v[2]].p};
            V3 nrm = vnorm(vcross(vsub(p[1], p[0]), vsub(p[2], p[0])));
            Color base = t->kind == SURF_GLASS ? glass : t->kind == SURF_UNDER ? under :
                         t->kind == SURF_TONNEAU ? tonneau : t->kind == SURF_TRIM ? trim : body;
            float l = 0.42f + 0.68f * fmaxf(vdot(nrm, SUN), 0.0f);
            Color col = shade(base, l);
            if (t->kind == SURF_BODY || t->kind == SURF_GLASS) {   // flat-faceted stainless / glass highlight
                V3 hv = vnorm(vadd(SUN, vnorm(vsub(eye, p[0]))));
                float s = powf(fmaxf(vdot(nrm, hv), 0.0f), t->kind == SURF_GLASS ? 40.0f : 14.0f);
                col = brighten(col, s * (t->kind == SURF_GLASS ? 0.55f : 0.30f));
            }
            DrawTriangle3D(RV(p[0]), RV(p[1]), RV(p[2]), col);
            if (t->edges) {
                Color edge = shade(base, 0.40f);
                for (int e = 0; e < 3; e++)
                    if (t->edges >> e & 1) DrawLine3D(RV(p[e]), RV(p[(e + 1) % 3]), edge);
            }
        }
        draw_lightbar(c, c->lightF, 0.45f, 1.0f, (Color){245, 248, 255, 255});
        draw_lightbar(c, c->lightR, 0.09f, -1.0f, (Color){230, 30, 30, 255});
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

    // Wheels: chunky all-terrain tyres with flat aero covers.
    for (int i = 0; i < 4; i++) {
        const Wheel *W = &c->w[i];
        V3 axis = vrot(c->right, c->up, W->steerAngle);
        float out = (i % 2 == 0) ? -1.0f : 1.0f;              // left wheels sit at -right
        V3 ctr = W->hub;
        V3 a = vsub(ctr, vscale(axis, WHEEL_W * 0.5f)), b = vadd(ctr, vscale(axis, WHEEL_W * 0.5f));
        DrawCylinderEx(RV(a), RV(b), WHEEL_R, WHEEL_R, 24, (Color){26, 26, 28, 255});
        V3 face = vadd(ctr, vscale(axis, out * (WHEEL_W * 0.5f + 0.004f)));
        DrawCylinderEx(RV(vsub(face, vscale(axis, out * 0.01f))), RV(face), WHEEL_R * 0.66f, WHEEL_R * 0.66f, 6, (Color){150, 152, 156, 255});
        V3 fw = vcross(c->up, axis);
        for (int s = 0; s < 6; s++) {   // cover segments show rotation
            V3 dir = vrot(fw, axis, W->spin + s * 1.0472f);
            V3 f0 = vadd(face, vscale(axis, out * 0.004f));
            DrawLine3D(RV(f0), RV(vadd(f0, vscale(dir, WHEEL_R * 0.66f))), (Color){70, 72, 76, 255});
        }
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

// ---------------------------------------------------------------- main ----

static const V3 PALETTE[] = {{0.71f, 0.72f, 0.74f},   // bare stainless for the player
                             {0.10f, 0.35f, 0.85f}, {0.95f, 0.75f, 0.10f}, {0.15f, 0.65f, 0.30f},
                             {0.92f, 0.92f, 0.92f}, {0.55f, 0.20f, 0.75f}, {0.95f, 0.45f, 0.10f}, {0.20f, 0.20f, 0.22f}};

static void spawn_player(World *w) {
    car_build(&w->cars[0], v3(0, terrain_height(0, 0) + CAR_SPAWN_H, 0), 0, PALETTE[0]);
}

static float wrap_pi(float a) { while (a > PI) a -= 2 * PI; while (a < -PI) a += 2 * PI; return a; }

int main(void) {
    SUN = vnorm(v3(0.45f, 0.85f, 0.30f));
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
    InitWindow(1440, 900, "BeamLite — soft-body driving");
    SetTargetFPS(120);

    for (int z = 0; z < CH_N; z++) for (int x = 0; x < CH_N; x++) chunks[z * CH_N + x] = build_chunk(x, z);

    static World world;
    world_init(&world);
    world.ncars = 3;
    spawn_player(&world);
    car_build(&world.cars[1], v3(-2.2f, CAR_SPAWN_H, 26), PI / 2, PALETTE[1]);
    car_build(&world.cars[2], v3(2.2f, CAR_SPAWN_H, 30), -PI / 2, PALETTE[2]);
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
            car_build(pc, v3(p.x, terrain_height(p.x, p.z) + CAR_SPAWN_H + 0.2f, p.z), -hdg, PALETTE[0]);
        }
        if (IsKeyPressed(KEY_N)) {       // drop a parked car ahead, side-on
            float hdg = atan2f(pc->fwd.z, pc->fwd.x);
            V3 p = vadd(pc->pos, vscale(vnorm(v3(pc->fwd.x, 0, pc->fwd.z)), 20.0f));
            int slot = world.ncars < MAX_CARS ? world.ncars++ : nextSlot;
            if (slot == nextSlot) nextSlot = nextSlot + 1 >= MAX_CARS ? 1 : nextSlot + 1;
            car_build(&world.cars[slot], v3(p.x, terrain_height(p.x, p.z) + CAR_SPAWN_H + 0.1f, p.z), -hdg + PI / 2, PALETTE[slot % 8]);
            world.cars[slot].ctl = (Controls){0, 0.6f, 0, 1};
        }
        if (IsKeyPressed(KEY_X)) { world.ncars = 1; nextSlot = 1; }
        if (IsKeyPressed(KEY_B)) beamView = !beamView;
        if (IsKeyPressed(KEY_C)) camMode = !camMode;
        if (IsKeyPressed(KEY_P)) paused = !paused;
        if (IsKeyPressed(KEY_T)) slowmo = !slowmo;
        if (IsKeyPressed(KEY_H) || IsKeyPressed(KEY_F1)) help = !help;
        if (IsKeyPressed(KEY_K)) { skidCount = 0; skidHead = 0; }
        if (IsKeyPressed(KEY_G)) car_pop_tailgate(pc);

        int up = IsKeyDown(KEY_W) || IsKeyDown(KEY_UP);
        int down = IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN);
        int left = IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT);
        int right = IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT);
        float vF = vdot(pc->vel, pc->fwd);
        Controls ctl = {0, 0, (float)(left - right), IsKeyDown(KEY_SPACE)};
        if (up) { if (vF < -1.0f) ctl.brake = 1; else ctl.throttle = 1; }
        if (down) { if (vF > 1.0f) ctl.brake = 1; else ctl.throttle = -1; }
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
            float k = fminf(1.0f, dt * 10.0f);
            camPos = (Vector3){camPos.x + (want.x - camPos.x) * k, camPos.y + (want.y - camPos.y) * k, camPos.z + (want.z - camPos.z) * k};
            cam.position = camPos;
            cam.target = RV(tgt);
            cam.up = (Vector3){0, 1, 0};
        } else {
            V3 eye = vadd(vadd(pc->pos, vscale(pc->up, 0.72f)), vscale(pc->fwd, 0.1f));
            cam.position = RV(eye);
            cam.target = RV(vadd(eye, vscale(pc->fwd, 10.0f)));
            cam.up = RV(pc->up);
            camPos = cam.position;
        }

        // ---- draw
        BeginDrawing();
        ClearBackground((Color){150, 190, 230, 255});
        BeginMode3D(cam);
        for (int i = 0; i < CH_N * CH_N; i++) DrawModel(chunks[i], (Vector3){0, 0, 0}, 1.0f, WHITE);
        rlDisableBackfaceCulling();
        draw_skids();
        for (int b = 0; b < world.nbox; b++) {
            Box *B = &world.box[b];
            Vector3 ctr = {(B->mn.x + B->mx.x) / 2, (B->mn.y + B->mx.y) / 2, (B->mn.z + B->mx.z) / 2};
            Vector3 sz = {B->mx.x - B->mn.x, B->mx.y - B->mn.y, B->mx.z - B->mn.z};
            DrawCubeV(ctr, sz, (Color){175, 172, 165, 255});
            DrawCubeWiresV(ctr, sz, (Color){95, 92, 88, 255});
        }
        for (int i = 0; i < world.ncars; i++) if (world.cars[i].active) draw_car(&world.cars[i], beamView, v3(cam.position.x, cam.position.y, cam.position.z));
        draw_shards();
        rlEnableBackfaceCulling();
        EndMode3D();

        // ---- HUD
        int sw = GetScreenWidth(), shh = GetScreenHeight();
        DrawRectangle(16, shh - 140, 330, 124, (Color){0, 0, 0, 120});
        DrawText(TextFormat("%3.0f km/h", fabsf(vF) * 3.6f), 30, shh - 132, 44, RAYWHITE);
        Color dc = pc->damage < 15 ? (Color){120, 230, 120, 255} : (pc->damage < 50 ? (Color){250, 210, 80, 255} : (Color){250, 90, 80, 255});
        DrawText(TextFormat("damage %3.0f%%   torn beams %d", pc->damage, pc->brokenBeams), 30, shh - 80, 18, dc);
        static const char *TG[] = {"latched", "open", "hanging", "gone"};
        DrawText(TextFormat("glass %d/%d broken   tailgate %s", pc->glassBrokenCount, pc->npanels, TG[pc->tailgate]), 30, shh - 58, 16, RAYWHITE);
        DrawText(TextFormat("%s%s%s", slowmo ? "SLOW-MO  " : "", paused ? "PAUSED  " : "", ctl.handbrake ? "HANDBRAKE" : ""), 30, shh - 34, 16, (Color){255, 220, 120, 255});
        DrawFPS(sw - 90, 12);
        if (help) {
            const char *lines[] = {
                "W/S or arrows  throttle / brake / reverse", "A/D            steer", "Space          handbrake",
                "Mouse drag     orbit camera   wheel: zoom", "C  hood cam    B  beam view (stress)",
                "N  spawn truck ahead   X  clear trucks", "Q  recover in place   R  reset to start",
                "T  slow-mo   P  pause   K  clear skids", "G  drop the tailgate   H  hide this help",
                "", "Ramp ahead, crash wall behind you,", "parked cars + dirt washboard to the left (+z)."};
            int n = sizeof lines / sizeof lines[0];
            DrawRectangle(16, 16, 390, 22 + n * 20, (Color){0, 0, 0, 120});
            for (int i = 0; i < n; i++) DrawText(lines[i], 28, 26 + i * 20, 16, RAYWHITE);
        }
        EndDrawing();
    }

    for (int i = 0; i < CH_N * CH_N; i++) UnloadModel(chunks[i]);
    CloseWindow();
    return 0;
}
