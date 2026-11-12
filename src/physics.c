// BeamLite — soft-body vehicle physics.
//
// The vehicle is an angular stainless-steel pickup (a Cybertruck-style wedge) built
// from 18 cross-section "rings" along its length. Each ring has 9 nodes: floor
// centre, floor edges, sills, beltline crease and roof/sail edge. Rings are joined
// into a monocoque shell (longitudinal members + shear-panel diagonals), with
// bulkheads at the nose, axles, dash, B-pillar, cabin rear and tail, long frame
// rails, a heavy structural battery pack in the floor, soft crumple zones at both
// ends and a stiff cabin safety cell.
//
// Beams yield plastically past a strain threshold (permanent deformation) and
// tear when stretched too far. Joints snap at a strain limit, cables only pull.
// Glass panels shatter when their frame is distorted. The tailgate is a separate
// part on hinges, latches and check straps, so it can pop open or come off.
//
// Wheels are ray-cast struts that push on the chassis nodes and generate tyre
// forces from a friction-circle model. Dual-motor AWD with rear-wheel steering.
// Everything is integrated with semi-implicit Euler at SIM_HZ.

#include "physics.h"
#include <string.h>

#define GRAV 9.81f
#define CAR_MASS 3100.0f          // kg, whole vehicle
#define TG_MASS 45.0f             // kg, tailgate

#define SUSP_REST 0.12f           // strut length at full extension (mount -> hub)
#define SUSP_TRAVEL 0.24f         // compression before hitting the bump stop
#define KS 70000.0f               // spring rate per corner (N/m)  ~1.5 Hz ride
#define CS 6500.0f                // damper rate per corner (N s/m)
#define KBUMP 600000.0f
#define MU_TIRE 1.15f
#define LAT_STIFF 3.0f            // lateral force per unit load per m/s of slip
#define BRAKE_MAX 8000.0f         // per wheel (N)
#define ENGINE_FMAX 20000.0f      // total, at the contact patches
#define ENGINE_POWER 440000.0f    // W (~600 hp dual motor)
#define REAR_STEER 0.30f          // rear lock as a fraction of front lock (~10 deg)
#define MU_GROUND 0.55f           // body scraping on the ground
#define NODE_R 0.28f              // node collision radius (car vs car)
#define KC 600000.0f
#define CC 5000.0f
#define CD_A 0.62f                // 0.5 * rho * Cd * A
#define GLASS_STRAIN 0.03f        // frame distortion that shatters a glass panel
#define TRACK_HALF 0.86f          // half track width (contact patch lateral position)

static float clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }
static float lerpf(float a, float b, float t) { return a + (b - a) * t; }
static float smooth01(float e0, float e1, float x) {
    float t = clampf((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// ---------------------------------------------------------------- terrain --

float terrain_height(float x, float z) {
    float r = sqrtf(x * x + z * z);
    float hills = smooth01(90.0f, 170.0f, r);
    float h = hills * (7.0f * sinf(x * 0.019f) * cosf(z * 0.016f) +
                       3.0f * sinf(x * 0.047f + z * 0.031f + 1.3f) +
                       1.2f * sinf(z * 0.11f - x * 0.05f));
    // Launch ramp straight ahead of spawn.
    if (x > 40.0f && x < 58.0f) {
        float wmask = smooth01(6.5f, 5.0f, fabsf(z));
        float t = (x - 40.0f) / 18.0f;
        h += wmask * 3.6f * (0.45f * t + 0.55f * t * t);
    }
    // Washboard dirt section on the cross road.
    if (fabsf(x) < 8.0f && z > 40.0f && z < 140.0f) {
        float m = smooth01(8.0f, 6.0f, fabsf(x)) * smooth01(40.0f, 46.0f, z) * smooth01(140.0f, 134.0f, z);
        h += m * 0.28f * (0.5f + 0.5f * sinf(z * 0.66f));
    }
    return h;
}

V3 terrain_normal(float x, float z) {
    const float e = 0.5f;
    float hx = terrain_height(x - e, z) - terrain_height(x + e, z);
    float hz = terrain_height(x, z - e) - terrain_height(x, z + e);
    return vnorm(v3(hx, 2.0f * e, hz));
}

// -------------------------------------------------------------- truck build --
//
// Local frame: +x forward, +y up (0 = underside of the floor), +z right.
// Overall 5.68 m long, 2.02 m wide, 1.79 m tall at the roof apex; wheelbase 3.81 m.

#define NR 18                     // rings along the body
#define NP 9                      // nodes per ring, in perimeter order:
enum { P_C, P_FL, P_RKL, P_BLL, P_TPL, P_TPR, P_BLR, P_RKR, P_FR };
#define ARCH_Y 0.54f              // sill height over the wheel arches
#define X_NOSE 2.84f
#define X_APEX 0.05f
#define X_TAIL -2.84f

typedef struct { float x, floorY, rkY; unsigned char arch, bulk; } Station;
static const Station ST[NR] = {
    { 2.84f, 0.14f, 0.24f, 0, 1},  //  0 nose (raised for approach angle)
    { 2.62f, 0.06f, 0.16f, 0, 0},  //  1
    { 2.40f, 0.00f, 0.14f, 0, 0},  //  2 front arch, lower corner
    { 2.22f, 0.00f, ARCH_Y, 1, 0}, //  3 front arch, top
    { 1.84f, 0.00f, ARCH_Y, 1, 1}, //  4 front axle
    { 1.46f, 0.00f, ARCH_Y, 1, 0}, //  5
    { 1.28f, 0.00f, 0.14f, 0, 0},  //  6
    { 0.85f, 0.00f, 0.14f, 0, 1},  //  7 dash / A-pillar base
    { 0.40f, 0.00f, 0.14f, 0, 0},  //  8
    { 0.05f, 0.00f, 0.14f, 0, 0},  //  9 roof apex
    {-0.45f, 0.00f, 0.14f, 0, 1},  // 10 B-pillar
    {-0.95f, 0.00f, 0.14f, 0, 1},  // 11 cabin rear bulkhead / bed front
    {-1.38f, 0.00f, 0.14f, 0, 0},  // 12 rear arch, lower corner
    {-1.60f, 0.00f, ARCH_Y, 1, 0}, // 13
    {-1.97f, 0.00f, ARCH_Y, 1, 1}, // 14 rear axle
    {-2.34f, 0.00f, ARCH_Y, 1, 0}, // 15
    {-2.56f, 0.00f, 0.14f, 0, 0},  // 16
    {-2.84f, 0.10f, 0.20f, 0, 1},  // 17 tail
};

// The signature profile: one straight line from the nose up to the roof apex,
// another straight line from the apex down to the tail.
static float prof_top(float x) {
    if (x >= X_APEX) return lerpf(1.32f, 0.58f, (x - X_APEX) / (X_NOSE - X_APEX));
    return lerpf(1.32f, 0.78f, (X_APEX - x) / (X_APEX - X_TAIL));
}
// Beltline crease: level along the flank, dropping under the hood line at the nose.
static float prof_belt(float x) { return fminf(0.70f, prof_top(x) - 0.08f); }
// Half-width of the top edge: full-width hood, greenhouse tumblehome, wide sails over the bed.
static float prof_tw(float x) {
    if (x >= 1.28f) return 0.95f;
    if (x >= X_APEX) return lerpf(0.74f, 0.95f, (x - X_APEX) / (1.28f - X_APEX));
    if (x >= -0.95f) return lerpf(0.74f, 0.80f, (X_APEX - x) / 1.0f);
    if (x >= -1.60f) return lerpf(0.80f, 0.97f, (-0.95f - x) / 0.65f);
    return 0.97f;
}

static int is_floor(int p) { return p == P_C || p == P_FL || p == P_FR; }
static int is_door(int p) { return p == P_RKL || p == P_BLL || p == P_RKR || p == P_BLR; }

// Structural zones, by position along the body.
static void material(float x, int floorBeam, float *K, float *Y) {
    int crumple = x > 2.05f || x < -2.40f;
    int cabin = x < 0.95f && x > -1.00f;
    // Yield force of a beam is ~0.7 * K * Y, independent of its length.
    if (floorBeam == 2) { *K = 300000.0f; *Y = 0.020f; }                                       // door skins
    else if (floorBeam) { *K = crumple ? 520000.0f : 900000.0f; *Y = crumple ? 0.030f : 0.050f; }   // frame / battery floor
    else if (crumple) { *K = 300000.0f; *Y = 0.025f; }                                          // crush cans, bumpers
    else if (cabin) { *K = 650000.0f; *Y = 0.045f; }                                            // stainless safety cell
    else { *K = 420000.0f; *Y = 0.035f; }                                                       // fenders, bed sides
}

// Builder scratch (single-threaded).
static V3 L[MAX_NODES];
static unsigned char ADJ[MAX_NODES][MAX_NODES];

static int add_node(Car *c, V3 l, float m) {
    int i = c->nn++;
    L[i] = l;
    c->n[i].m = m;
    return i;
}

static int add_beam(Car *c, int a, int b, float K, float yield, float brk, int type) {
    if (a == b || ADJ[a][b] || c->nb >= MAX_BEAMS) return -1;
    float len = vlen(vsub(L[a], L[b]));
    if (len < 1e-3f) return -1;
    ADJ[a][b] = ADJ[b][a] = 1;
    Beam *B = &c->b[c->nb];
    memset(B, 0, sizeof *B);
    B->a = a; B->b = b;
    B->rest = B->rest0 = len;
    // Axial stiffness ~ EA/L, then clamped so the explicit integrator stays stable
    // for the two masses it connects (short beams between light nodes would otherwise blow up).
    const float h = 1.0f / SIM_HZ;
    float ma = c->n[a].m, mb = c->n[b].m, mu = ma * mb / (ma + mb);
    B->k = fminf(K * (0.7f / len), 0.10f * mu / (h * h));
    B->c = fminf(B->k * 0.0028f, 0.15f * mu / h);
    B->yield = yield; B->brk = brk; B->type = (unsigned char)type;
    return c->nb++;
}

static void add_tri(Car *c, int a, int b, int d, int kind, int panel, int edges, V3 ref) {
    if (c->nt >= MAX_TRIS) return;
    V3 pa = L[a], pb = L[b], pd = L[d];
    V3 n = vcross(vsub(pb, pa), vsub(pd, pa));
    V3 ctr = vscale(vadd(vadd(pa, pb), pd), 1.0f / 3.0f);
    if (vdot(n, vsub(ctr, ref)) < 0) {        // wind so the normal points away from ref
        int t = b; b = d; d = t;
        edges = (edges & 2) | ((edges & 1) << 2) | ((edges >> 2) & 1);
    }
    Tri *T = &c->t[c->nt++];
    T->v[0] = a; T->v[1] = b; T->v[2] = d;
    T->kind = (unsigned char)kind; T->panel = (signed char)panel; T->edges = (unsigned char)edges;
    for (int e = 0; e < 3; e++) T->e0[e] = vlen(vsub(L[T->v[e]], L[T->v[(e + 1) % 3]]));
}

// Quad a-b-c-d; edge bits: 1 ab, 2 bc, 4 cd, 8 da.
static void add_quad(Car *c, int a, int b, int cc, int d, int kind, int panel, int e4, V3 ref) {
    add_tri(c, a, b, cc, kind, panel, (e4 & 1) | (e4 & 2), ref);
    add_tri(c, a, cc, d, kind, panel, ((e4 >> 2) & 1) << 1 | ((e4 >> 3) & 1) << 2, ref);
}

void car_build(Car *c, V3 pos, float yaw, V3 color) {
    memset(c, 0, sizeof *c);
    memset(ADJ, 0, sizeof ADJ);
    c->color = color;
    c->active = 1;

    // ---- nodes, with mass concentrated low: battery pack in the floor, drive units at the axles
    float wt[NR][NP], wsum = 0;
    for (int r = 0; r < NR; r++)
    for (int p = 0; p < NP; p++) {
        float w = is_floor(p) ? 1.6f : ((p == P_TPL || p == P_TPR) ? 1.0f : 1.2f);
        if (p == P_C && ST[r].x < 1.30f && ST[r].x > -1.00f) w += 3.0f;
        if ((p == P_FL || p == P_FR) && (r == 4 || r == 14)) w += 1.0f;
        wt[r][p] = w; wsum += w;
    }
    float unit = (CAR_MASS - TG_MASS) / wsum;

    int R[NR][NP];
    for (int r = 0; r < NR; r++) {
        float x = ST[r].x, fy = ST[r].floorY, rk = ST[r].rkY;
        float top = prof_top(x), belt = prof_belt(x), tw = prof_tw(x);
        V3 l[NP] = {
            {x, fy, 0}, {x, fy, -0.62f}, {x, rk, -0.97f}, {x, belt, -1.01f}, {x, top, -tw},
            {x, top, tw}, {x, belt, 1.01f}, {x, rk, 0.97f}, {x, fy, 0.62f},
        };
        for (int p = 0; p < NP; p++) R[r][p] = add_node(c, l[p], wt[r][p] * unit);
    }
    c->nBody = c->nn;

    // ---- body structure
    for (int r = 0; r < NR; r++)                      // ring frames / bulkheads
    for (int a = 0; a < NP; a++)
    for (int b = a + 1; b < NP; b++) {
        int perim = (b - a == 1) || (a == 0 && b == NP - 1);
        float K, Y;
        material(ST[r].x, is_floor(a) && is_floor(b), &K, &Y);
        // Cross-bracing: full bulkheads at the ends, axles and cabin rear; the dash and B-pillar
        // rings are open across the cabin (people sit there), so their webs are lighter.
        if (!perim) K *= !ST[r].bulk ? 0.35f : (r == 7 || r == 10) ? 0.25f : 1.0f;
        add_beam(c, R[r][a], R[r][b], K, Y, 0, BEAM_NORMAL);
    }
    for (int r = 0; r + 1 < NR; r++) {                // shell between rings
        float xm = 0.5f * (ST[r].x + ST[r + 1].x);
        for (int p = 0; p < NP; p++) {
            int q = (p + 1) % NP;
            float K, Y;
            int cab = r >= 7 && r <= 10, door = cab && is_door(p) && is_door(q);
            material(xm, cab && is_door(p) ? 2 : is_floor(p), &K, &Y);
            add_beam(c, R[r][p], R[r + 1][p], K, Y, 0, BEAM_NORMAL);             // longitudinal
            material(xm, door ? 2 : is_floor(p) && is_floor(q), &K, &Y);
            add_beam(c, R[r][p], R[r + 1][q], 0.8f * K, Y, 0, BEAM_NORMAL);      // shear panel
            add_beam(c, R[r][q], R[r + 1][p], 0.8f * K, Y, 0, BEAM_NORMAL);
        }
    }
    const int rails[] = {P_C, P_FL, P_FR, P_BLL, P_BLR, P_TPL, P_TPR};   // frame rails, belt creases, roof rails
    for (int r = 0; r + 2 < NR; r++)
    for (int i = 0; i < 7; i++) {
        int p = rails[i];
        float K, Y;
        material(ST[r + 1].x, is_floor(p), &K, &Y);
        add_beam(c, R[r][p], R[r + 2][p], 0.6f * K, Y, 0, BEAM_NORMAL);
    }

    // ---- suspension towers: everything attached to a strut node is a stiffer, stronger subframe
    {
        static unsigned char strut[MAX_NODES];
        memset(strut, 0, sizeof strut);
        const int sp[4] = {P_FL, P_FR, P_RKL, P_RKR};
        for (int i = 0; i < 4; i++) { strut[R[4][sp[i]]] = 1; strut[R[14][sp[i]]] = 1; }
        const float h = 1.0f / SIM_HZ;
        for (int i = 0; i < c->nb; i++) {
            Beam *B = &c->b[i];
            if (!strut[B->a] && !strut[B->b]) continue;
            float ma = c->n[B->a].m, mb = c->n[B->b].m, mu = ma * mb / (ma + mb);
            B->k = fminf(B->k * 1.5f, 0.10f * mu / (h * h));
            B->c = fminf(B->k * 0.0028f, 0.15f * mu / h);
            B->yield = fmaxf(B->yield, 0.07f);
        }
    }

    // ---- skin
    for (int r = 0; r + 1 < NR; r++) {
        float xm = 0.5f * (ST[r].x + ST[r + 1].x);
        int arch = ST[r].arch || ST[r + 1].arch;
        int seam = (r == 7 || r == 10 || r == 11);    // door shut lines
        V3 ref = v3(clampf(xm, -2.4f, 2.4f), 0.40f, 0);
        for (int s = 0; s < NP; s++) {
            int p = s, q = (s + 1) % NP;
            int kind = SURF_BODY, panel = -1, e = 1 | 4;   // longitudinal edges are the creases
            if (s == 0 || s == 8) kind = SURF_UNDER;
            else if (s == 1 || s == 7) kind = SURF_TRIM;   // sill chamfer / wheel-well liner
            else if (s == 3 || s == 5) {                   // greenhouse side / sails
                if (r == 7 || r == 8) panel = s == 3 ? 3 : 4;
                else if (r == 9) panel = s == 3 ? 5 : 6;
            } else if (s == 4) {                           // hood, windshield, roof, tonneau
                if (r == 7 || r == 8) panel = 0;
                else if (r == 9) panel = 1;
                else if (r == 10) panel = 2;
                else if (r >= 11) kind = SURF_TONNEAU;
            }
            if (panel >= 0) { kind = SURF_GLASS; e = 15; }
            if (seam && s >= 2 && s <= 6) e |= 8;
            (void)arch;
            add_quad(c, R[r][p], R[r + 1][p], R[r + 1][q], R[r][q], kind, panel, e, ref);
        }
    }
    c->npanels = 7;
    {   // nose: flat front face, fan from the floor centre
        V3 ref = v3(2.0f, 0.4f, 0);
        for (int i = 1; i < NP - 1; i++) {
            int kind = (i == 1 || i == NP - 2) ? SURF_TRIM : SURF_BODY;
            int e = 2 | (i == 1 ? 1 : 0) | (i == NP - 2 ? 4 : 0);
            add_tri(c, R[0][P_C], R[0][i], R[0][i + 1], kind, -1, e, ref);
        }
    }
    {   // tail: bumper below the tailgate opening, sail ends above it
        int *T = R[NR - 1];
        V3 ref = v3(-2.0f, 0.4f, 0);
        add_tri(c, T[P_C], T[P_FL], T[P_RKL], SURF_TRIM, -1, 3, ref);
        add_tri(c, T[P_C], T[P_RKL], T[P_RKR], SURF_TRIM, -1, 2, ref);
        add_tri(c, T[P_C], T[P_RKR], T[P_FR], SURF_TRIM, -1, 6, ref);
        add_tri(c, T[P_BLL], T[P_TPL], T[P_TPR], SURF_BODY, -1, 3, ref);
        add_tri(c, T[P_BLL], T[P_TPR], T[P_BLR], SURF_BODY, -1, 6, ref);
    }

    // ---- tailgate: a separate rigid part (6 nodes, 15 beams)
    {
        const float tx = -2.86f, tz = 0.93f, m = TG_MASS / 6.0f;
        int BL = add_node(c, v3(tx, 0.24f, -tz), m), BR = add_node(c, v3(tx, 0.24f, tz), m);
        int TL = add_node(c, v3(tx - 0.06f, 0.66f, -tz), m), TR = add_node(c, v3(tx - 0.06f, 0.66f, tz), m);  // raked out
        int ML = add_node(c, v3(-2.80f, 0.45f, -0.90f), m), MR = add_node(c, v3(-2.80f, 0.45f, 0.90f), m);
        int tg[6] = {BL, BR, TL, TR, ML, MR};
        for (int i = 0; i < 6; i++)
            for (int j = i + 1; j < 6; j++) add_beam(c, tg[i], tg[j], 420000.0f, 0.02f, 0, BEAM_NORMAL);
        int *T = R[NR - 1], *U = R[NR - 2];
        // Hinges: each bottom corner pinned by three joints.
        c->tgHingeL[0] = add_beam(c, BL, T[P_RKL], 500000.0f, 1e9f, 0.35f, BEAM_JOINT);
        c->tgHingeL[1] = add_beam(c, BL, T[P_FL], 500000.0f, 1e9f, 0.35f, BEAM_JOINT);
        c->tgHingeL[2] = add_beam(c, BL, U[P_RKL], 500000.0f, 1e9f, 0.35f, BEAM_JOINT);
        c->tgHingeR[0] = add_beam(c, BR, T[P_RKR], 500000.0f, 1e9f, 0.35f, BEAM_JOINT);
        c->tgHingeR[1] = add_beam(c, BR, T[P_FR], 500000.0f, 1e9f, 0.35f, BEAM_JOINT);
        c->tgHingeR[2] = add_beam(c, BR, U[P_RKR], 500000.0f, 1e9f, 0.35f, BEAM_JOINT);
        // Latches at the top corners: weaker, they let go first.
        c->tgLatch[0] = add_beam(c, TL, T[P_BLL], 400000.0f, 1e9f, 0.10f, BEAM_JOINT);
        c->tgLatch[1] = add_beam(c, TL, T[P_TPL], 400000.0f, 1e9f, 0.10f, BEAM_JOINT);
        c->tgLatch[2] = add_beam(c, TL, U[P_BLL], 400000.0f, 1e9f, 0.10f, BEAM_JOINT);
        c->tgLatch[3] = add_beam(c, TR, T[P_BLR], 400000.0f, 1e9f, 0.10f, BEAM_JOINT);
        c->tgLatch[4] = add_beam(c, TR, T[P_TPR], 400000.0f, 1e9f, 0.10f, BEAM_JOINT);
        c->tgLatch[5] = add_beam(c, TR, U[P_BLR], 400000.0f, 1e9f, 0.10f, BEAM_JOINT);
        // Rubber stops on the bed sides: the gate can't swing in past closed.
        add_beam(c, TL, U[P_RKL], 300000.0f, 1e9f, 0, BEAM_STOP);
        add_beam(c, TR, U[P_RKR], 300000.0f, 1e9f, 0, BEAM_STOP);
        // Check straps: cables that go taut when the gate is folded down level.
        for (int s = 0; s < 2; s++) {
            int top = s ? TR : TL, anchor = s ? U[P_TPR] : U[P_TPL];
            V3 hinge = L[s ? BR : BL], rel = vsub(L[top], hinge);
            V3 open = vadd(hinge, v3(-sqrtf(rel.x * rel.x + rel.y * rel.y), 0, rel.z));
            int bi = add_beam(c, top, anchor, 300000.0f, 1e9f, 0.15f, BEAM_CABLE);
            if (bi >= 0) c->b[bi].rest = c->b[bi].rest0 = vlen(vsub(open, L[anchor]));
            c->tgStrap[s] = bi;
        }
        V3 ref = v3(-2.84f, 0.45f, 0);
        add_quad(c, BL, BR, TR, TL, SURF_BODY, -1, 15, ref);
        add_quad(c, TL, TR, MR, ML, SURF_TRIM, -1, 0, ref);
        add_quad(c, BL, ML, MR, BR, SURF_TRIM, -1, 0, ref);
        add_tri(c, BL, TL, ML, SURF_TRIM, -1, 0, ref);
        add_tri(c, BR, MR, TR, SURF_TRIM, -1, 0, ref);
        c->lightR[0] = TL; c->lightR[1] = TR; c->lightR[2] = BL; c->lightR[3] = BR;
    }
    c->lightF[0] = R[0][P_TPL]; c->lightF[1] = R[0][P_TPR]; c->lightF[2] = R[0][P_BLL]; c->lightF[3] = R[0][P_BLR];

    // ---- place in the world
    float cy = cosf(yaw), sy = sinf(yaw);
    for (int n = 0; n < c->nn; n++) {
        V3 l = L[n];
        c->n[n].p = v3(pos.x + l.x * cy + l.z * sy, pos.y + l.y, pos.z - l.x * sy + l.z * cy);
        c->n[n].v = v3(0, 0, 0);
    }

    // ---- wheels: FL, FR, RL, RR on the floor-edge nodes at the axle bulkheads. Dual-motor AWD.
    const int wr[4] = {4, 4, 14, 14};
    for (int w = 0; w < 4; w++) {
        int left = (w % 2 == 0);
        Wheel *W = &c->w[w];
        W->mount = R[wr[w]][left ? P_FL : P_FR];
        W->upper = R[wr[w]][left ? P_RKL : P_RKR];
        W->off = (left ? -1.0f : 1.0f) * (TRACK_HALF - 0.62f);
        W->front = w < 2;
        W->driveShare = W->front ? 0.225f : 0.275f;
    }
    c->tailgate = TG_LATCHED;
    car_update_frame(c);
    for (int w = 0; w < 4; w++) {
        Wheel *W = &c->w[w];
        W->hub = vsub(vadd(c->n[W->mount].p, vscale(c->right, W->off)), vscale(c->up, SUSP_REST));
    }
}

void car_update_frame(Car *c) {
    V3 fl = c->n[c->w[0].mount].p, fr = c->n[c->w[1].mount].p;
    V3 rl = c->n[c->w[2].mount].p, rr = c->n[c->w[3].mount].p;
    V3 f = vnorm(vsub(vadd(fl, fr), vadd(rl, rr)));
    V3 r = vsub(vadd(fr, rr), vadd(fl, rl));
    c->up = vnorm(vcross(r, f));
    c->right = vnorm(vcross(f, c->up));
    c->fwd = f;
    // Centre of mass and velocity of the body only (a lost tailgate shouldn't drag the camera).
    V3 p = v3(0, 0, 0), v = v3(0, 0, 0);
    float M = 0;
    for (int i = 0; i < c->nBody; i++) {
        float m = c->n[i].m;
        p = vadd(p, vscale(c->n[i].p, m)); v = vadd(v, vscale(c->n[i].v, m)); M += m;
    }
    c->pos = vscale(p, 1.0f / M);
    c->vel = vscale(v, 1.0f / M);
    V3 mn = c->n[0].p, mx = c->n[0].p;
    for (int i = 0; i < c->nn; i++) {
        V3 q = c->n[i].p;
        mn = v3(fminf(mn.x, q.x), fminf(mn.y, q.y), fminf(mn.z, q.z));
        mx = v3(fmaxf(mx.x, q.x), fmaxf(mx.y, q.y), fmaxf(mx.z, q.z));
    }
    c->bmin = mn; c->bmax = mx;
}

static int intact(const Car *c, int bi) { return bi >= 0 && !c->b[bi].broken; }

void car_update_damage(Car *c) {
    float sum = 0; int broken = 0, n = 0;
    for (int i = 0; i < c->nb; i++) {
        const Beam *B = &c->b[i];
        if (B->type != BEAM_NORMAL) continue;
        n++;
        if (B->broken) { broken++; sum += 0.5f; continue; }
        sum += fabsf(B->rest - B->rest0) / B->rest0;
    }
    c->brokenBeams = broken;
    c->damage = clampf(sum / (float)(n ? n : 1) * 1000.0f, 0.0f, 100.0f);

    // Glass: a panel shatters when any edge of its frame has been distorted too far.
    for (int i = 0; i < c->nt; i++) {
        const Tri *T = &c->t[i];
        if (T->panel < 0 || c->glassBroken[T->panel]) continue;
        for (int e = 0; e < 3; e++) {
            float len = vlen(vsub(c->n[T->v[e]].p, c->n[T->v[(e + 1) % 3]].p));
            if (fabsf(len - T->e0[e]) > GLASS_STRAIN * T->e0[e]) {
                c->glassBroken[T->panel] = 1;
                c->glassNew |= 1u << T->panel;
                c->glassBrokenCount++;
                break;
            }
        }
    }

    // Tailgate state.
    int latched = 0, hl = 0, hr = 0;
    for (int i = 0; i < 6; i++) latched |= intact(c, c->tgLatch[i]);
    for (int i = 0; i < 3; i++) { hl |= intact(c, c->tgHingeL[i]); hr |= intact(c, c->tgHingeR[i]); }
    c->tailgate = (hl || hr) ? (latched ? TG_LATCHED : (hl && hr ? TG_OPEN : TG_HANGING)) : TG_GONE;
}

void car_pop_tailgate(Car *c) {
    if (c->tailgate != TG_LATCHED) return;
    for (int i = 0; i < 6; i++) if (c->tgLatch[i] >= 0) c->b[c->tgLatch[i]].broken = 1;
    // A small shove outward, like the latch spring.
    for (int i = 0; i < 2; i++) {
        Node *n = &c->n[c->lightR[i]];
        n->v = vsub(n->v, vscale(c->fwd, 0.6f));
    }
}

// ------------------------------------------------------------- dynamics ----

static void add_force(Node *n, V3 f) { n->f = vadd(n->f, f); }

static void beam_forces(Car *c) {
    for (int i = 0; i < c->nb; i++) {
        Beam *B = &c->b[i];
        if (B->broken) continue;
        Node *a = &c->n[B->a], *b = &c->n[B->b];
        V3 d = vsub(b->p, a->p);
        float L = vlen(d);
        if (L < 1e-6f) continue;
        V3 dir = vscale(d, 1.0f / L);
        float s = (L - B->rest) / B->rest;
        float vrel = vdot(vsub(b->v, a->v), dir);

        if (B->type == BEAM_CABLE) {               // pulls only, snaps when overloaded
            if (s <= 0) { B->stress = 0; continue; }
            if (s > B->brk) { B->broken = 1; continue; }
            B->stress = s / B->brk;
            float F = fmaxf(B->k * (L - B->rest) + B->c * vrel, 0.0f);
            add_force(a, vscale(dir, F));
            add_force(b, vscale(dir, -F));
            continue;
        }
        if (B->type == BEAM_STOP) {                // pushes only: a rubber bump stop
            B->stress = 0;
            if (s >= 0) continue;
            float F = fminf(B->k * (L - B->rest) + B->c * vrel, 0.0f);
            add_force(a, vscale(dir, F));
            add_force(b, vscale(dir, -F));
            continue;
        }
        if (B->type == BEAM_JOINT) {               // elastic until it snaps
            if (fabsf(s) > B->brk) { B->broken = 1; continue; }
            B->stress = s / B->brk;
        } else {
            // Plasticity: past the yield strain the rest length follows the deformation.
            if (s > B->yield) B->rest = L / (1.0f + B->yield);
            else if (s < -B->yield) B->rest = L / (1.0f - B->yield);
            B->rest = clampf(B->rest, 0.3f * B->rest0, 1.3f * B->rest0);
            if (B->rest >= 1.3f * B->rest0) { B->broken = 1; continue; }   // torn
            B->stress = clampf(((L - B->rest) / B->rest) / B->yield, -1.5f, 1.5f);
        }

        float F = B->k * (L - B->rest) + B->c * vrel;
        float minL = 0.45f * B->rest0;             // keep crushed structure from inverting
        if (L < minL) F -= B->k * 4.0f * (minL - L);
        add_force(a, vscale(dir, F));
        add_force(b, vscale(dir, -F));
    }
}

static void wheel_forces(Car *c, float h) {
    const Controls *ctl = &c->ctl;
    float vF = vdot(c->vel, c->fwd);

    // Steering: speed-sensitive lock, rate-limited. Rear wheels counter-steer at low speed.
    float maxLock = 0.58f / (1.0f + fabsf(vF) / 22.0f);
    float target = clampf(ctl->steer, -1, 1) * maxLock;
    c->steer += clampf(target - c->steer, -2.2f * h, 2.2f * h);
    float rearSteer = -c->steer * REAR_STEER * clampf(1.0f - fabsf(vF) / 18.0f, 0.0f, 1.0f);

    float drive = 0;
    if (ctl->throttle > 0) drive = ctl->throttle * fminf(ENGINE_FMAX, ENGINE_POWER / fmaxf(fabsf(vF), 2.0f));
    else if (ctl->throttle < 0 && vF > -12.0f) drive = ctl->throttle * 9000.0f;

    for (int i = 0; i < 4; i++) {
        Wheel *W = &c->w[i];
        Node *m = &c->n[W->mount], *u = &c->n[W->upper];
        V3 up = c->up;
        V3 org = vadd(m->p, vscale(c->right, W->off));
        W->steerAngle = W->front ? c->steer : rearSteer;
        W->contact = 0; W->load = 0; W->slip = 0;

        float gh = terrain_height(org.x, org.z);
        float dist = up.y > 0.25f ? (org.y - gh) / up.y : 1e9f;   // along -up to ground
        float comp = SUSP_REST + WHEEL_R - dist;
        V3 vel = vscale(vadd(m->v, u->v), 0.5f);

        V3 wf = vrot(c->fwd, up, W->steerAngle);

        if (comp <= 0) {
            W->compress = 0;
            W->hub = vsub(org, vscale(up, SUSP_REST));
            W->side = vnorm(vcross(wf, up));
            W->spin += (ctl->throttle > 0.1f ? 30.0f : 0.0f) * h;
            continue;
        }

        W->contact = 1;
        W->compress = comp;
        float vcomp = -vdot(vel, up);
        float Fs = KS * comp + CS * vcomp;
        if (comp > SUSP_TRAVEL) Fs += KBUMP * (comp - SUSP_TRAVEL);
        if (Fs < 0) Fs = 0;

        V3 n = terrain_normal(org.x, org.z);
        float N = Fs * fmaxf(vdot(up, n), 0.0f);
        V3 fs = vscale(up, Fs * 0.5f);
        add_force(m, fs); add_force(u, fs);

        // Tyre forces in the contact plane.
        wf = vnorm(vsub(wf, vscale(n, vdot(wf, n))));
        V3 ws = vnorm(vcross(wf, n));
        float vl = vdot(vel, wf), vs = vdot(vel, ws);
        int hb = ctl->handbrake && !W->front;
        float mu = MU_TIRE * (1.0f - 0.2f * c->wetness) * (hb ? 0.55f : 1.0f);

        float Fy = -vs * N * LAT_STIFF;
        float Fx = drive * W->driveShare;
        float bf = BRAKE_MAX * clampf(ctl->brake, 0, 1);
        if (hb) { bf = fmaxf(bf, mu * N); Fx = 0; }
        Fx -= clampf(vl * 9000.0f, -bf, bf);
        Fx -= vl * 8.0f;                              // rolling resistance

        float lim = mu * N, mag = sqrtf(Fx * Fx + Fy * Fy);
        if (mag > lim && mag > 1e-6f) {
            W->slip = (mag - lim) / (lim + 1.0f);
            Fx *= lim / mag; Fy *= lim / mag;
        }
        if (hb && fabsf(vl) > 1.0f) W->slip = fmaxf(W->slip, 0.5f);
        V3 ft = vadd(vscale(wf, Fx * 0.5f), vscale(ws, Fy * 0.5f));
        add_force(m, ft); add_force(u, ft);

        W->load = N;
        W->side = ws;
        W->hub = vsub(org, vscale(up, dist - WHEEL_R));
        W->cp = vsub(W->hub, vscale(n, WHEEL_R));
        if (!hb) W->spin += (vl / WHEEL_R) * h;
        if (W->slip > 0.05f && ctl->throttle > 0.5f) W->spin += 25.0f * h;
    }
}

static void contact_resolve(Node *N, V3 n, float pen, float mu) {
    N->p = vadd(N->p, vscale(n, pen));
    float vn = vdot(N->v, n);
    if (vn >= 0) return;
    N->v = vsub(N->v, vscale(n, vn));
    V3 vt = vsub(N->v, vscale(n, vdot(N->v, n)));
    float vtl = vlen(vt), dv = -vn * mu;
    N->v = vtl <= dv ? vsub(N->v, vt) : vsub(N->v, vscale(vt, dv / vtl));
}

static int grid_cell(float x, float z) {
    int gx = (int)floorf((x + GRID_N * GRID_CELL * 0.5f) / GRID_CELL);
    int gz = (int)floorf((z + GRID_N * GRID_CELL * 0.5f) / GRID_CELL);
    if (gx < 0 || gz < 0 || gx >= GRID_N || gz >= GRID_N) return -1;
    return gz * GRID_N + gx;
}

static void break_tree(World *w, Tree *t, V3 v, V3 n) {
    t->broken = 1;
    w->treesDown++;
    V3 dir = v3(v.x, 0, v.z);
    if (vlen(dir) < 0.5f) dir = v3(-n.x, 0, -n.z);
    t->fall = vnorm(dir);
    t->tilt = 0.03f;
    t->tiltVel = fminf(vlen(v3(v.x, 0, v.z)) / (0.5f * t->h), 2.5f);
}

static void collide_static(World *w, Car *c) {
    for (int i = 0; i < c->nn; i++) {
        Node *N = &c->n[i];
        float gh = terrain_height(N->p.x, N->p.z);
        if (N->p.y < gh) {
            V3 n = terrain_normal(N->p.x, N->p.z);
            contact_resolve(N, n, (gh - N->p.y) * n.y, MU_GROUND);
        }
        int cell = grid_cell(N->p.x, N->p.z);
        if (cell < 0) continue;
        for (int k = w->gridStart[cell]; k < w->gridStart[cell + 1]; k++) {
            int it = w->gridItem[k];
            V3 p = N->p;
            if (it < MAX_BOXES) {
                Box *B = &w->box[it];
                if (p.x < B->mn.x || p.x > B->mx.x || p.y < B->mn.y || p.y > B->mx.y || p.z < B->mn.z || p.z > B->mx.z) continue;
                float d[6] = {p.x - B->mn.x, B->mx.x - p.x, p.y - B->mn.y, B->mx.y - p.y, p.z - B->mn.z, B->mx.z - p.z};
                int best = 0;
                for (int q = 1; q < 6; q++) if (d[q] < d[best]) best = q;
                V3 n = v3(0, 0, 0);
                if (best == 0) n.x = -1; else if (best == 1) n.x = 1;
                else if (best == 2) n.y = -1; else if (best == 3) n.y = 1;
                else if (best == 4) n.z = -1; else n.z = 1;
                contact_resolve(N, n, d[best], 0.4f);
            } else {
                Tree *t = &w->tree[it - MAX_BOXES];
                if (t->broken || p.y < t->y - 0.5f || p.y > t->y + t->h) continue;
                float dx = p.x - t->x, dz = p.z - t->z, d2 = dx * dx + dz * dz;
                if (d2 >= t->r * t->r) continue;
                float d = sqrtf(d2);
                V3 n = d > 1e-4f ? v3(dx / d, 0, dz / d) : v3(1, 0, 0);
                V3 vin = N->v;
                float vn = vdot(vin, n);
                contact_resolve(N, n, t->r - d, 0.3f);
                if (vn < 0) t->imp += N->m * -vn;
                if (t->breakImp > 0 && t->imp > t->breakImp) break_tree(w, t, vin, n);
            }
        }
    }
}

static int in_box(V3 p, V3 lo, V3 hi) {
    return p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y && p.z >= lo.z && p.z <= hi.z;
}

static void collide_cars(Car *A, Car *B) {
    float D = 2.0f * NODE_R;
    if (A->bmax.x + D < B->bmin.x || B->bmax.x + D < A->bmin.x ||
        A->bmax.y + D < B->bmin.y || B->bmax.y + D < A->bmin.y ||
        A->bmax.z + D < B->bmin.z || B->bmax.z + D < A->bmin.z) return;
    // Only nodes inside the overlap of the two bounding boxes can touch.
    V3 lo = v3(fmaxf(A->bmin.x, B->bmin.x) - D, fmaxf(A->bmin.y, B->bmin.y) - D, fmaxf(A->bmin.z, B->bmin.z) - D);
    V3 hi = v3(fminf(A->bmax.x, B->bmax.x) + D, fminf(A->bmax.y, B->bmax.y) + D, fminf(A->bmax.z, B->bmax.z) + D);
    int ia[MAX_NODES], ib[MAX_NODES], na = 0, nb = 0;
    for (int i = 0; i < A->nn; i++) if (in_box(A->n[i].p, lo, hi)) ia[na++] = i;
    if (!na) return;
    for (int j = 0; j < B->nn; j++) if (in_box(B->n[j].p, lo, hi)) ib[nb++] = j;
    float D2 = D * D;
    for (int i = 0; i < na; i++) {
        Node *a = &A->n[ia[i]];
        for (int j = 0; j < nb; j++) {
            Node *b = &B->n[ib[j]];
            V3 d = vsub(b->p, a->p);
            float d2 = vdot(d, d);
            if (d2 >= D2 || d2 < 1e-10f) continue;
            float dist = sqrtf(d2);
            V3 n = vscale(d, 1.0f / dist);
            V3 rv = vsub(b->v, a->v);
            float vrel = vdot(rv, n);
            float F = KC * (D - dist) - CC * vrel;
            if (F <= 0) continue;
            add_force(a, vscale(n, -F));
            add_force(b, vscale(n, F));
            V3 vt = vsub(rv, vscale(n, vrel));
            float vtl = vlen(vt);
            if (vtl > 1e-4f) {
                V3 ff = vscale(vt, 0.35f * F * fminf(1.0f, vtl / 0.5f) / vtl);
                add_force(a, ff);
                add_force(b, vscale(ff, -1));
            }
        }
    }
}

static void update_trees(World *w, float h) {
    for (int i = 0; i < w->ntree; i++) {
        Tree *t = &w->tree[i];
        if (t->imp > 0) t->imp = fmaxf(0.0f, t->imp - t->imp * 6.0f * h - 1.0f);   // forget old knocks
        if (!t->broken || t->tilt >= 1.45f) continue;
        t->tiltVel += 1.5f * GRAV / t->h * sinf(t->tilt) * h;                       // falling rod
        t->tilt += t->tiltVel * h;
        if (t->tilt >= 1.45f) { t->tilt = 1.45f; t->tiltVel = 0; }
    }
}

void world_substep(World *w, float h) {
    update_trees(w, h);
    for (int ci = 0; ci < w->ncars; ci++) {
        Car *c = &w->cars[ci];
        if (!c->active) continue;
        car_update_frame(c);
        float sp = vlen(c->vel);
        for (int i = 0; i < c->nn; i++) {
            Node *N = &c->n[i];
            N->f = v3(0, -GRAV * N->m, 0);
            N->f = vadd(N->f, vscale(N->v, -CD_A * sp * N->m / CAR_MASS));   // aero drag, spread by mass
        }
        beam_forces(c);
        wheel_forces(c, h);
    }
    for (int a = 0; a < w->ncars; a++)
        for (int b = a + 1; b < w->ncars; b++)
            if (w->cars[a].active && w->cars[b].active) collide_cars(&w->cars[a], &w->cars[b]);
    for (int ci = 0; ci < w->ncars; ci++) {
        Car *c = &w->cars[ci];
        if (!c->active) continue;
        for (int i = 0; i < c->nn; i++) {
            Node *N = &c->n[i];
            N->v = vadd(N->v, vscale(N->f, h / N->m));
            N->p = vadd(N->p, vscale(N->v, h));
            if (!(fabsf(N->v.x) + fabsf(N->v.y) + fabsf(N->v.z) < 400.0f)) c->exploded = 1;  // also catches NaN
        }
        collide_static(w, c);
    }
}

// ------------------------------------------------------------- the map ----
//
// Highway along x, a cross road (dirt washboard section) along z, and two small
// towns either side of the cross road on the north (+z) side, each with an avenue
// and side streets. Forest on the hills. The south side and the test corridors
// stay open.

typedef struct { float x0, x1, z0, z1; } Rect;

static const Rect ROADS[] = {
    {-260, 260, -7, 7},          // highway
    {-7, 7, -260, 260},          // cross road
};
static const Rect STREETS[] = {
    {-112, 112, 44, 52},         // avenue
    {-80, 80, 94, 100},          // upper avenue
    {36, 44, 7, 136}, {76, 84, 7, 94}, {-44, -36, 7, 136}, {-84, -76, 7, 94},   // side streets
    {30, 78, -13, -7},           // service road into the warehouse district
};
// No scenery here: ramp + landing run-out, crash wall, spawn, and the headless test routes.
static const Rect KEEP_CLEAR[] = {
    {30, 225, -12, 12}, {-82, -64, -18, 18}, {-16, 16, -16, 16},
    {-82, 72, -72, -38}, {-82, -14, -72, 8},
};
#define NELEM(a) ((int)(sizeof(a) / sizeof((a)[0])))

static int in_rect(const Rect *r, float x, float z, float m) {
    return x > r->x0 - m && x < r->x1 + m && z > r->z0 - m && z < r->z1 + m;
}
static int rect_overlap(const Rect *a, const Rect *b, float m) {
    return a->x0 < b->x1 + m && b->x0 < a->x1 + m && a->z0 < b->z1 + m && b->z0 < a->z1 + m;
}

int road_at(float x, float z) {
    for (int i = 0; i < NELEM(ROADS); i++) if (in_rect(&ROADS[i], x, z, 0)) return 1;
    for (int i = 0; i < NELEM(STREETS); i++) if (in_rect(&STREETS[i], x, z, 0)) return 2;
    return 0;
}

int road_rects(float (*out)[4], int max, int *nHighway) {
    int n = 0;
    for (int i = 0; i < NELEM(ROADS) && n < max; i++, n++) { out[n][0] = ROADS[i].x0; out[n][1] = ROADS[i].x1; out[n][2] = ROADS[i].z0; out[n][3] = ROADS[i].z1; }
    *nHighway = n;
    for (int i = 0; i < NELEM(STREETS) && n < max; i++, n++) { out[n][0] = STREETS[i].x0; out[n][1] = STREETS[i].x1; out[n][2] = STREETS[i].z0; out[n][3] = STREETS[i].z1; }
    return n;
}

static unsigned rng_state;
static float frnd(void) { rng_state = rng_state * 1664525u + 1013904223u; return (rng_state >> 8) / 16777216.0f; }
static float frange(float a, float b) { return a + (b - a) * frnd(); }

static int add_box(World *w, V3 mn, V3 mx) {
    if (w->nbox >= MAX_BOXES) return -1;
    w->box[w->nbox].mn = mn; w->box[w->nbox].mx = mx;
    return w->nbox++;
}

// Is this footprint free of roads, keep-clear zones, boxes and trees?
static int area_free(const World *w, Rect r, float margin) {
    float rm = fminf(margin, 1.0f);             // kerb clearance is smaller than the spacing between things
    for (int i = 0; i < NELEM(ROADS); i++) if (rect_overlap(&r, &ROADS[i], rm)) return 0;
    for (int i = 0; i < NELEM(STREETS); i++) if (rect_overlap(&r, &STREETS[i], rm)) return 0;
    for (int i = 0; i < NELEM(KEEP_CLEAR); i++) if (rect_overlap(&r, &KEEP_CLEAR[i], 0)) return 0;
    if (fabsf(r.x0) > 290 || fabsf(r.x1) > 290 || fabsf(r.z0) > 290 || fabsf(r.z1) > 290) return 0;
    for (int i = 0; i < w->nbox; i++) {
        Rect b = {w->box[i].mn.x, w->box[i].mx.x, w->box[i].mn.z, w->box[i].mx.z};
        if (rect_overlap(&r, &b, margin)) return 0;
    }
    for (int i = 0; i < w->ntree; i++)
        if (in_rect(&r, w->tree[i].x, w->tree[i].z, margin)) return 0;
    return 1;
}

int world_add_tree(World *w, float x, float z, float h, int kind) {
    if (w->ntree >= MAX_TREES) return -1;
    Tree *t = &w->tree[w->ntree];
    memset(t, 0, sizeof *t);
    t->x = x; t->z = z; t->y = terrain_height(x, z);
    t->h = h; t->kind = (unsigned char)kind;
    t->r = (kind == TREE_CONIFER ? 0.025f : 0.0275f) * h;
    // Saplings and young trees snap; mature trunks stop a truck dead.
    if (t->r < 0.17f) t->breakImp = 3500.0f;
    else if (t->r < 0.25f) t->breakImp = 14000.0f;
    return w->ntree++;
}

static void try_tree(World *w, float x, float z, float h, int kind, float spacing) {
    Rect r = {x, x, z, z};
    if (area_free(w, r, spacing)) world_add_tree(w, x, z, h, kind);
}

static void add_building(World *w, Rect r, int kind, float height, int front) {
    if (w->nbld >= MAX_BLD) return;
    float lo = 1e9f, hi = -1e9f;
    for (int i = 0; i <= 2; i++)
        for (int j = 0; j <= 2; j++) {
            float g = terrain_height(lerpf(r.x0, r.x1, i * 0.5f), lerpf(r.z0, r.z1, j * 0.5f));
            lo = fminf(lo, g); hi = fmaxf(hi, g);
        }
    Building *B = &w->bld[w->nbld];
    B->mn = v3(r.x0, lo - 0.6f, r.z0);
    B->mx = v3(r.x1, hi + height, r.z1);
    B->ground = hi;
    B->kind = kind; B->front = front;
    B->roof = kind == BLD_HOUSE ? frange(2.4f, 3.4f) : (kind == BLD_WAREHOUSE ? 1.4f : 0.0f);
    B->seed = (unsigned)(frnd() * 4294967040.0f);
    if (add_box(w, B->mn, B->mx) >= 0) w->nbld++;
}

typedef struct { Rect r; int zone; } Block;   // zone 0 downtown, 1 mixed, 2 suburb

static int pick_kind(int zone, int first) {
    float u = frnd();
    if (zone == 0) return first ? BLD_TOWER : (u < 0.12f ? BLD_TOWER : u < 0.6f ? BLD_APARTMENT : BLD_SHOP);
    if (zone == 1) return u < 0.3f ? BLD_APARTMENT : u < 0.65f ? BLD_SHOP : BLD_HOUSE;
    if (zone == 2) return u < 0.75f ? BLD_HOUSE : u < 0.87f ? BLD_SHOP : BLD_WAREHOUSE;
    return u < 0.75f ? BLD_WAREHOUSE : BLD_SHOP;                                  // industrial
}

// Frontage width, depth and wall height for a kind.
static void pick_size(int k, float *wdt, float *dep, float *ht) {
    switch (k) {
        case BLD_TOWER: *wdt = frange(13, 17); *dep = frange(13, 17); *ht = frange(30, 52); break;
        case BLD_APARTMENT: *wdt = frange(14, 22); *dep = frange(10, 13); *ht = 3.2f * (int)frange(4, 7) + 0.8f; break;
        case BLD_SHOP: *wdt = frange(10, 16); *dep = frange(8, 11); *ht = frange(4.5f, 5.6f); break;
        case BLD_WAREHOUSE: *wdt = frange(18, 26); *dep = frange(12, 16); *ht = frange(7, 9); break;
        default: *wdt = frange(8.5f, 11); *dep = frange(7.5f, 9.5f); *ht = frange(5.6f, 6.4f); break;
    }
}

static void fill_block(World *w, const Block *bk) {
    const Rect *R = &bk->r;
    const float SET = 1.5f;                     // setback from the kerb
    int first = 1;
    // Lots along each street edge: south, north, west, east.
    for (int e = 0; e < 4; e++) {
        int alongX = e < 2;
        float len = alongX ? R->x1 - R->x0 : R->z1 - R->z0;
        float cur = frange(0.5f, 2.5f);
        while (cur < len - 6) {
            int k = pick_kind(bk->zone, first);
            float wdt, dep, ht;
            pick_size(k, &wdt, &dep, &ht);
            if (k == BLD_TOWER) { if (cur + wdt > len - 1) { cur += 2; continue; } }
            if (cur + wdt > len - 1) wdt = len - 1 - cur;
            if (wdt < 7.5f) break;
            Rect fp; int front;
            if (e == 0) { fp = (Rect){R->x0 + cur, R->x0 + cur + wdt, R->z0 + SET, R->z0 + SET + dep}; front = 3; }
            else if (e == 1) { fp = (Rect){R->x0 + cur, R->x0 + cur + wdt, R->z1 - SET - dep, R->z1 - SET}; front = 2; }
            else if (e == 2) { fp = (Rect){R->x0 + SET, R->x0 + SET + dep, R->z0 + cur, R->z0 + cur + wdt}; front = 1; }
            else { fp = (Rect){R->x1 - SET - dep, R->x1 - SET, R->z0 + cur, R->z0 + cur + wdt}; front = 0; }
            if (fp.x1 <= R->x1 && fp.z1 <= R->z1 && area_free(w, fp, k == BLD_HOUSE ? 2.2f : 1.6f)) {
                add_building(w, fp, k, ht, front);
                first = 0;
                cur += wdt + (k == BLD_HOUSE ? frange(2.5f, 5.0f) : frange(1.8f, 3.5f));
            } else cur += 2.0f;
        }
    }
    // Yard and park trees in what's left.
    for (int t = 0; t < 30; t++) {
        float x = frange(R->x0 + 1, R->x1 - 1), z = frange(R->z0 + 1, R->z1 - 1);
        try_tree(w, x, z, frange(5.0f, 8.5f), frnd() < 0.8f ? TREE_BROADLEAF : TREE_CONIFER, 2.5f);
    }
}

void world_build_grid(World *w) {
    static int count[GRID_N * GRID_N];
    memset(count, 0, sizeof count);
    const float M = 0.6f;                       // margin so nodes near a cell edge still see the obstacle
    for (int pass = 0; pass < 2; pass++) {
        int total = 0;
        if (pass == 1) {
            for (int c = 0; c < GRID_N * GRID_N; c++) { w->gridStart[c] = total; total += count[c]; count[c] = 0; }
            w->gridStart[GRID_N * GRID_N] = total;
        }
        int nobj = w->nbox + w->ntree;
        for (int o = 0; o < nobj; o++) {
            float x0, x1, z0, z1; int item;
            if (o < w->nbox) { x0 = w->box[o].mn.x; x1 = w->box[o].mx.x; z0 = w->box[o].mn.z; z1 = w->box[o].mx.z; item = o; }
            else { Tree *t = &w->tree[o - w->nbox]; x0 = t->x - t->r; x1 = t->x + t->r; z0 = t->z - t->r; z1 = t->z + t->r; item = MAX_BOXES + o - w->nbox; }
            const float half = GRID_N * GRID_CELL * 0.5f;
            int gx0 = (int)floorf((x0 - M + half) / GRID_CELL), gx1 = (int)floorf((x1 + M + half) / GRID_CELL);
            int gz0 = (int)floorf((z0 - M + half) / GRID_CELL), gz1 = (int)floorf((z1 + M + half) / GRID_CELL);
            for (int gz = gz0 < 0 ? 0 : gz0; gz <= gz1 && gz < GRID_N; gz++)
                for (int gx = gx0 < 0 ? 0 : gx0; gx <= gx1 && gx < GRID_N; gx++) {
                    int c = gz * GRID_N + gx;
                    if (pass == 1) {
                        int at = w->gridStart[c] + count[c];
                        if (at < MAX_GRID_ITEMS) w->gridItem[at] = item;
                    }
                    count[c]++;
                }
        }
        if (pass == 1 && w->gridStart[GRID_N * GRID_N] > MAX_GRID_ITEMS)   // overflow: drop the tail safely
            for (int c = 0; c <= GRID_N * GRID_N; c++) if (w->gridStart[c] > MAX_GRID_ITEMS) w->gridStart[c] = MAX_GRID_ITEMS;
    }
}

int world_point_solid(const World *w, V3 p) {
    int cell = grid_cell(p.x, p.z);
    if (cell < 0) return 0;
    for (int k = w->gridStart[cell]; k < w->gridStart[cell + 1]; k++) {
        int it = w->gridItem[k];
        if (it < MAX_BOXES) {
            const Box *B = &w->box[it];
            if (p.x > B->mn.x && p.x < B->mx.x && p.y > B->mn.y && p.y < B->mx.y && p.z > B->mn.z && p.z < B->mx.z) return 1;
        } else {
            const Tree *t = &w->tree[it - MAX_BOXES];
            float dx = p.x - t->x, dz = p.z - t->z;
            if (!t->broken && p.y < t->y + t->h && dx * dx + dz * dz < t->r * t->r) return 1;
        }
    }
    return 0;
}

void world_init(World *w) {
    memset(w, 0, sizeof *w);
    rng_state = 20240611u;
    // Crash-test wall behind the spawn point.
    add_box(w, v3(-74, -1, -14), v3(-71, 3.0f, 14));
    // A row of concrete pillars beside the main road.
    const float px[6] = {-60, -46, -32, -18, 18, 32};
    for (int i = 0; i < 6; i++) {
        float x = px[i];
        add_box(w, v3(x - 0.6f, -1, 11.0f), v3(x + 0.6f, 4.0f, 12.2f));
    }
    // Low barrier blocks scattered on the plaza.
    add_box(w, v3(18, -1, -26), v3(30, 0.8f, -24.5f));
    add_box(w, v3(-30, -1, -30), v3(-28.5f, 1.2f, -18));

    // Towns: blocks between the highway, the avenue and the side streets, mirrored across the cross road.
    const Block east[] = {
        {{9, 36, 15, 44}, 0}, {{44, 76, 15, 44}, 1}, {{84, 112, 15, 44}, 2},
        {{9, 36, 52, 92}, 1}, {{44, 76, 52, 92}, 2}, {{84, 112, 52, 92}, 2},
        {{9, 36, 100, 134}, 2}, {{44, 76, 100, 132}, 2},
    };
    for (int side = -1; side <= 1; side += 2)
        for (int i = 0; i < NELEM(east); i++) {
            Block b = east[i];
            if (side < 0) { float t = b.r.x0; b.r.x0 = -b.r.x1; b.r.x1 = -t; }
            fill_block(w, &b);
        }
    {   // Warehouse district south of the highway (east side only; the west is a test route).
        Block ind = {{34, 74, -38, -14}, 3};
        fill_block(w, &ind);
    }
    // Saplings along the avenue kerbs, in the setback strip in front of the buildings.
    for (int side = -1; side <= 1; side += 2)
        for (float x = 12; x < 110; x += 9) {
            if (fabsf(x - 40) < 6 || fabsf(x - 80) < 6) continue;
            for (int s = 0; s < 2; s++) {
                float tx = side * x, tz = s ? 52.7f : 43.3f;
                Rect r = {tx, tx, tz, tz};
                int ok = 1;
                for (int i = 0; i < w->nbox && ok; i++) {
                    Rect b = {w->box[i].mn.x, w->box[i].mx.x, w->box[i].mn.z, w->box[i].mx.z};
                    ok = !rect_overlap(&r, &b, 0.5f);
                }
                if (ok) world_add_tree(w, tx, tz, frange(4.2f, 5.4f), TREE_BROADLEAF);
            }
        }

    // Roadside trees along the highway and the cross road, out in the country.
    for (float x = 66; x < 250; x += 11) { try_tree(w, x, 14.5f + frange(-1, 1), frange(6, 9), TREE_BROADLEAF, 2.0f); try_tree(w, x + 5, -14.5f + frange(-1, 1), frange(6, 9), TREE_BROADLEAF, 2.0f); }
    for (float x = -86; x > -250; x -= 11) { try_tree(w, x, 14.5f + frange(-1, 1), frange(6, 9), TREE_BROADLEAF, 2.0f); try_tree(w, x - 5, -14.5f + frange(-1, 1), frange(6, 9), TREE_BROADLEAF, 2.0f); }
    for (float z = 100; z < 250; z += 12) { try_tree(w, 12, z, frange(6, 10), TREE_CONIFER, 2.0f); try_tree(w, -12, z + 6, frange(6, 10), TREE_CONIFER, 2.0f); }
    for (float z = -80; z > -250; z -= 12) { try_tree(w, 12, z, frange(6, 10), TREE_CONIFER, 2.0f); try_tree(w, -12, z - 6, frange(6, 10), TREE_CONIFER, 2.0f); }

    // Forest on the hills, thicker further out; a few loners on the flat.
    for (int i = 0; i < 2600 && w->ntree < MAX_TREES - 40; i++) {
        float r = frange(95, 290), a = frange(0, 6.2831853f);
        float x = r * cosf(a), z = r * sinf(a);
        if (frnd() > smooth01(90, 200, r) + 0.15f) continue;
        int conifer = frnd() < 0.35f + 0.5f * smooth01(110, 220, r);
        try_tree(w, x, z, conifer ? frange(8, 15) : frange(6, 10), conifer ? TREE_CONIFER : TREE_BROADLEAF, 3.2f);
    }
    for (int i = 0; i < 120; i++) {
        float x = frange(-88, 88), z = frange(-88, 88);
        if (x * x + z * z > 88 * 88) continue;
        try_tree(w, x, z, frange(5, 9), frnd() < 0.7f ? TREE_BROADLEAF : TREE_CONIFER, 6.0f);
    }

    world_build_grid(w);
    w->ncars = 0;
}
