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
        float mu = MU_TIRE * (hb ? 0.55f : 1.0f);

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

static void collide_static(World *w, Car *c) {
    for (int i = 0; i < c->nn; i++) {
        Node *N = &c->n[i];
        float gh = terrain_height(N->p.x, N->p.z);
        if (N->p.y < gh) {
            V3 n = terrain_normal(N->p.x, N->p.z);
            contact_resolve(N, n, (gh - N->p.y) * n.y, MU_GROUND);
        }
        for (int b = 0; b < w->nbox; b++) {
            Box *B = &w->box[b];
            V3 p = N->p;
            if (p.x < B->mn.x || p.x > B->mx.x || p.y < B->mn.y || p.y > B->mx.y || p.z < B->mn.z || p.z > B->mx.z) continue;
            float d[6] = {p.x - B->mn.x, B->mx.x - p.x, p.y - B->mn.y, B->mx.y - p.y, p.z - B->mn.z, B->mx.z - p.z};
            int best = 0;
            for (int k = 1; k < 6; k++) if (d[k] < d[best]) best = k;
            V3 n = v3(0, 0, 0);
            if (best == 0) n.x = -1; else if (best == 1) n.x = 1;
            else if (best == 2) n.y = -1; else if (best == 3) n.y = 1;
            else if (best == 4) n.z = -1; else n.z = 1;
            contact_resolve(N, n, d[best], 0.4f);
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

void world_substep(World *w, float h) {
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

static void add_box(World *w, V3 mn, V3 mx) {
    if (w->nbox < MAX_BOXES) { w->box[w->nbox].mn = mn; w->box[w->nbox].mx = mx; w->nbox++; }
}

void world_init(World *w) {
    memset(w, 0, sizeof *w);
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
    w->ncars = 0;
}
