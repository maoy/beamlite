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
#define MU_TIRE 1.15f
#define LAT_STIFF 3.0f            // lateral force per unit load per m/s of slip
#define MU_GROUND 0.55f           // body scraping on the ground
#define NODE_R 0.28f              // node collision radius (car vs car)
#define KC 600000.0f
#define CC 5000.0f
#define GLASS_STRAIN 0.03f        // frame distortion that shatters a glass panel

static float clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }
static float lerpf(float a, float b, float t) { return a + (b - a) * t; }
static float smooth01(float e0, float e1, float x) {
    float t = clampf((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// ---------------------------------------------------------------- terrain --

static int g_map;                                 // map the world was last built with (terrain depends on it)
static float wash_z0(void) { return g_map == MAP_NEWYORK ? -150.0f : 40.0f; }
static float wash_z1(void) { return g_map == MAP_NEWYORK ? -90.0f : 140.0f; }
int dirt_at(float x, float z) { return fabsf(x) < 8.0f && z > wash_z0() && z < wash_z1(); }

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
    // Washboard dirt section on the cross road (north of the highway on Default, south on New York).
    if (dirt_at(x, z)) {
        float z0 = wash_z0(), z1 = wash_z1();
        float m = smooth01(8.0f, 6.0f, fabsf(x)) * smooth01(z0, z0 + 6.0f, z) * smooth01(z1, z1 - 6.0f, z);
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

#define NP 9                      // nodes per ring, in perimeter order:
enum { P_C, P_FL, P_RKL, P_BLL, P_TPL, P_TPR, P_BLR, P_RKR, P_FR };

static VehicleDef DEFS[VEH_COUNT];
static int defsReady;

// ---- Stainless pickup: 5.68 m long, 2.02 m wide, 1.79 m tall at the roof apex, 3.81 m wheelbase.
// The signature profile: one straight line from the nose up to the roof apex, another down to the tail.
#define PU_ARCH 0.54f
static float pu_top(float x) {
    if (x >= 0.05f) return lerpf(1.32f, 0.58f, (x - 0.05f) / (2.84f - 0.05f));
    return lerpf(1.32f, 0.78f, (0.05f - x) / (0.05f + 2.84f));
}
static float pu_tw(float x) {
    if (x >= 1.28f) return 0.95f;
    if (x >= 0.05f) return lerpf(0.74f, 0.95f, (x - 0.05f) / (1.28f - 0.05f));
    if (x >= -0.95f) return lerpf(0.74f, 0.80f, (0.05f - x) / 1.0f);
    if (x >= -1.60f) return lerpf(0.80f, 0.97f, (-0.95f - x) / 0.65f);
    return 0.97f;
}

static void define_pickup(VehicleDef *d) {
    static const float X[18] = {2.84f, 2.62f, 2.40f, 2.22f, 1.84f, 1.46f, 1.28f, 0.85f, 0.40f, 0.05f, -0.45f, -0.95f, -1.38f, -1.60f, -1.97f, -2.34f, -2.56f, -2.84f};
    static const float FY[18] = {0.14f, 0.06f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.10f};
    static const float RK[18] = {0.24f, 0.16f, 0.14f, PU_ARCH, PU_ARCH, PU_ARCH, 0.14f, 0.14f, 0.14f, 0.14f, 0.14f, 0.14f, 0.14f, PU_ARCH, PU_ARCH, PU_ARCH, 0.14f, 0.20f};
    static const unsigned char BULK[18] = {1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1};
    d->nr = 18;
    for (int r = 0; r < 18; r++) {
        Station *s = &d->st[r];
        float x = X[r], top = pu_top(x);
        *s = (Station){x, FY[r], RK[r], fminf(0.70f, top - 0.08f), top, pu_tw(x), 1.0f, RK[r] > 0.3f, BULK[r], 0, -1, -1};
        s->seam = (r == 7 || r == 10 || r == 11);
        if (r == 6) s->topPanel = 0;                                    // long windshield runs down the hood line
        if (r == 7 || r == 8) { s->topPanel = 0; s->sidePanel = 3; }   // windshield, front side glass
        else if (r == 9) { s->topPanel = 1; s->sidePanel = 5; }        // glass roof, rear side glass
        else if (r == 10) s->topPanel = 2;                              // rear glass
        else if (r >= 11) s->topPanel = -2;                             // tonneau over the bed
    }
    d->name = "STAINLESS PICKUP";
    d->blurb = "3.1 t  |  dual-motor AWD  |  rear steer";
    d->floorHalf = 0.62f; d->sillHalf = 0.97f; d->beltHalf = 1.01f;
    d->axleF = 4; d->axleR = 14; d->doorR0 = 7; d->doorR1 = 10; d->dashRing = 7; d->bRing = 10;
    d->mass = 3100.0f;
    d->batW = 3.0f; d->batX0 = -1.00f; d->batX1 = 1.30f;      // structural battery pack
    d->engW = 0.0f; d->engX0 = d->engX1 = 0;
    d->crumpleF = 2.05f; d->crumpleR = -2.40f; d->cabinX0 = -1.00f; d->cabinX1 = 0.95f;
    d->kScale = 1.0f; d->yScale = 1.0f;
    d->wheelR = 0.44f; d->wheelW = 0.30f; d->trackHalf = 0.86f;
    d->suspRest = 0.12f; d->suspTravel = 0.24f; d->ks = 70000.0f; d->cs = 6500.0f; d->kbump = 600000.0f;
    d->brakeMax = 8000.0f; d->engineF = 20000.0f; d->enginePower = 440000.0f; d->reverseF = 9000.0f; d->driveFront = 0.45f;
    d->rearSteer = 0.30f; d->maxLock = 0.58f; d->cdA = 0.62f;
    d->spawnH = CAR_SPAWN_H;
    d->seat = v3(-0.35f, 1.12f, -0.42f); d->dashX = 1.05f; d->dashY = 0.88f; d->rearSeatX = -0.72f;
    d->npanels = 7; d->hasTailgate = 1; d->lightStyle = 0;
    d->smooth = 0; d->wheelStyle = 0; d->mirrorRing = -1; d->spoilerRing = -1;
    d->paint = v3(0.71f, 0.72f, 0.74f);
}

// ---- Sparrow: a modern compact five-door hatchback, proportioned after a current C-segment
// hatch. 4.37 m long, 1.79 m wide, 1.45 m tall, 2.64 m wheelbase, ~1.35 t, 2.0 L front-wheel drive.
// Long sloping hood into a steeply raked windshield; the roof peaks over the front seats and
// falls away to a spoiler over a raked hatch; the window line kicks up into a pointed rear
// quarter window; round wheel arches and flared rear haunches. Heights are above the floor
// underside, which sits ~0.15 m off the ground.
static void define_hatch(VehicleDef *d) {
    // x, floor, sill, belt, top, roof-edge half-width, width scale, arch, bulkhead, seam, top glass, side glass
    static const Station S[] = {
        { 2.185f, 0.25f, 0.30f, 0.42f, 0.50f, 0.58f, 0.80f, 0, 1, 0, -1, -1},   //  0 nose (front fascia)
        { 2.080f, 0.14f, 0.18f, 0.50f, 0.58f, 0.66f, 0.90f, 0, 0, 0, -1, -1},   //  1 hood leading edge
        { 1.900f, 0.05f, 0.10f, 0.56f, 0.64f, 0.70f, 0.96f, 0, 0, 0, -1, -1},   //  2
        { 1.685f, 0.00f, 0.10f, 0.62f, 0.68f, 0.72f, 1.00f, 0, 0, 0, -1, -1},   //  3 front arch, lower front
        { 1.545f, 0.00f, 0.48f, 0.65f, 0.71f, 0.72f, 1.00f, 1, 0, 0, -1, -1},   //  4
        { 1.285f, 0.00f, 0.58f, 0.70f, 0.75f, 0.72f, 1.00f, 1, 1, 0, -1, -1},   //  5 front axle
        { 1.025f, 0.00f, 0.48f, 0.74f, 0.79f, 0.72f, 1.00f, 1, 0, 0, -1, -1},   //  6
        { 0.885f, 0.00f, 0.10f, 0.77f, 0.81f, 0.71f, 1.00f, 0, 0, 0, -1, -1},   //  7 front arch, lower rear
        { 0.750f, 0.00f, 0.08f, 0.80f, 0.84f, 0.70f, 1.00f, 0, 1, 1,  0,  2},   //  8 cowl / A-pillar base / dash
        { 0.400f, 0.00f, 0.08f, 0.82f, 1.05f, 0.64f, 1.00f, 0, 0, 0,  0,  2},   //  9
        { 0.050f, 0.00f, 0.08f, 0.84f, 1.22f, 0.60f, 1.00f, 0, 0, 0,  0,  2},   // 10 windshield top
        {-0.200f, 0.00f, 0.08f, 0.86f, 1.29f, 0.59f, 1.00f, 0, 0, 0, -1,  2},   // 11 roof header
        {-0.550f, 0.00f, 0.08f, 0.88f, 1.30f, 0.58f, 1.00f, 0, 1, 1, -1,  4},   // 12 B-pillar (roof peak)
        {-0.955f, 0.00f, 0.10f, 0.91f, 1.28f, 0.58f, 1.02f, 0, 0, 1, -1,  6},   // 13 rear door shut / arch lower front
        {-1.095f, 0.00f, 0.48f, 0.95f, 1.26f, 0.57f, 1.03f, 1, 0, 0, -1,  6},   // 14
        {-1.355f, 0.00f, 0.58f, 1.00f, 1.23f, 0.56f, 1.04f, 1, 1, 0, -1, -1},   // 15 rear axle; C-pillar behind
        {-1.615f, 0.00f, 0.48f, 1.04f, 1.21f, 0.55f, 1.03f, 1, 0, 0,  1, -1},   // 16 roof end / spoiler root
        {-1.755f, 0.00f, 0.10f, 0.92f, 1.06f, 0.58f, 0.99f, 0, 0, 0,  1, -1},   // 17 rear arch lower rear; hatch glass
        {-1.950f, 0.03f, 0.12f, 0.80f, 0.88f, 0.62f, 0.94f, 0, 0, 0, -1, -1},   // 18 hatch glass bottom
        {-2.120f, 0.15f, 0.22f, 0.70f, 0.80f, 0.64f, 0.88f, 0, 0, 0, -1, -1},   // 19 lower hatch
        {-2.185f, 0.26f, 0.32f, 0.60f, 0.72f, 0.62f, 0.82f, 0, 1, 0, -1, -1},   // 20 tail (rear fascia)
    };
    d->nr = (int)(sizeof S / sizeof S[0]);
    for (int r = 0; r < d->nr; r++) d->st[r] = S[r];
    d->name = "SPARROW HATCHBACK";
    d->blurb = "1.35 t  |  2.0 L front-wheel drive  |  sport hatch";
    d->floorHalf = 0.58f; d->sillHalf = 0.84f; d->beltHalf = 0.86f;
    d->axleF = 5; d->axleR = 15; d->doorR0 = 8; d->doorR1 = 12; d->dashRing = 8; d->bRing = 12;
    d->mass = 1350.0f;
    d->batW = 0.8f; d->batX0 = -1.30f; d->batX1 = -0.60f;     // fuel tank under the rear seat
    d->engW = 2.0f; d->engX0 = 0.80f; d->engX1 = 2.10f;       // transverse engine and gearbox up front
    d->crumpleF = 1.75f; d->crumpleR = -1.85f; d->cabinX0 = -0.96f; d->cabinX1 = 0.76f;
    d->kScale = 0.42f; d->yScale = 0.85f;
    d->wheelR = 0.32f; d->wheelW = 0.225f; d->trackHalf = 0.77f;      // 225/40 R18
    d->suspRest = -0.06f; d->suspTravel = 0.16f; d->ks = 30000.0f; d->cs = 2400.0f; d->kbump = 300000.0f;
    d->brakeMax = 3900.0f; d->engineF = 6800.0f; d->enginePower = 95000.0f; d->reverseF = 3500.0f; d->driveFront = 1.0f;
    d->rearSteer = 0.0f; d->maxLock = 0.62f; d->cdA = 0.41f;
    d->spawnH = 0.22f;
    d->seat = v3(-0.52f, 1.00f, -0.38f); d->dashX = 0.52f; d->dashY = 0.80f; d->rearSeatX = -1.12f;
    d->npanels = 8; d->hasTailgate = 0; d->lightStyle = 1;
    d->smooth = 1; d->wheelStyle = 1; d->mirrorRing = 8; d->spoilerRing = 16;
    d->paint = v3(0.93f, 0.33f, 0.09f);                       // bright orange
}

const VehicleDef *vehicle_def(int type) {
    if (!defsReady) { define_pickup(&DEFS[VEH_PICKUP]); define_hatch(&DEFS[VEH_HATCH]); defsReady = 1; }
    return &DEFS[type < 0 || type >= VEH_COUNT ? 0 : type];
}

static int is_floor(int p) { return p == P_C || p == P_FL || p == P_FR; }
static int is_door(int p) { return p == P_RKL || p == P_BLL || p == P_RKR || p == P_BLR; }

// Structural zones, by position along the body. kind: 0 shell, 1 floor/frame, 2 door skin.
static void material(const VehicleDef *d, float x, int kind, float *K, float *Y) {
    int crumple = x > d->crumpleF || x < d->crumpleR;
    int cabin = x < d->cabinX1 && x > d->cabinX0;
    // Yield force of a beam is ~0.7 * K * Y, independent of its length.
    if (kind == 2) { *K = 300000.0f; *Y = 0.020f; }                                            // door skins
    else if (kind) { *K = crumple ? 520000.0f : 900000.0f; *Y = crumple ? 0.030f : 0.050f; }   // frame / floor
    else if (crumple) { *K = 300000.0f; *Y = 0.025f; }                                          // crush zones, bumpers
    else if (cabin) { *K = 650000.0f; *Y = 0.045f; }                                            // safety cell
    else { *K = 420000.0f; *Y = 0.035f; }                                                       // fenders, bed sides
    *K *= d->kScale; *Y *= d->yScale;
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

static void add_light(Car *c, int q0, int q1, int q2, int q3, float t, float u0, float u1, float out, int rear) {
    if (c->nlights >= 4) return;
    c->lights[c->nlights++] = (LightStrip){{q0, q1, q2, q3}, t, u0, u1, out, (unsigned char)rear};
}

void car_build(Car *c, V3 pos, float yaw, V3 color) { car_build_vehicle(c, VEH_PICKUP, pos, yaw, color); }

void car_build_vehicle(Car *c, int type, V3 pos, float yaw, V3 color) {
    const VehicleDef *D = vehicle_def(type);
    const Station *ST = D->st;
    const int nr = D->nr;
    memset(c, 0, sizeof *c);
    memset(ADJ, 0, sizeof ADJ);
    c->def = D; c->type = type;
    c->grip = 1.0f;
    c->color = color;
    c->active = 1;
    for (int i = 0; i < 6; i++) c->tgLatch[i] = -1;
    for (int i = 0; i < 3; i++) c->tgHingeL[i] = c->tgHingeR[i] = -1;
    c->tgStrap[0] = c->tgStrap[1] = -1;
    const float TG_MASS = D->hasTailgate ? 45.0f : 0.0f;

    // ---- nodes, with mass concentrated low: battery/tank and engine on the floor, drive units at the axles
    float wt[NR_MAX][NP], wsum = 0;
    for (int r = 0; r < nr; r++)
    for (int p = 0; p < NP; p++) {
        float w = is_floor(p) ? 1.6f : ((p == P_TPL || p == P_TPR) ? 1.0f : 1.2f);
        float x = ST[r].x;
        if (p == P_C && x < D->batX1 && x > D->batX0) w += D->batW;
        if (is_floor(p) && x < D->engX1 && x > D->engX0) w += D->engW;
        if ((p == P_FL || p == P_FR) && (r == D->axleF || r == D->axleR)) w += 1.0f;
        wt[r][p] = w; wsum += w;
    }
    float unit = (D->mass - TG_MASS) / wsum;

    int R[NR_MAX][NP];
    for (int r = 0; r < nr; r++) {
        const Station *s = &ST[r];
        float x = s->x, fy = s->floorY, rk = s->rkY, sw = D->sillHalf * s->ws, bw = D->beltHalf * s->ws, fw = D->floorHalf;
        V3 l[NP] = {
            {x, fy, 0}, {x, fy, -fw}, {x, rk, -sw}, {x, s->belt, -bw}, {x, s->top, -s->tw},
            {x, s->top, s->tw}, {x, s->belt, bw}, {x, rk, sw}, {x, fy, fw},
        };
        for (int p = 0; p < NP; p++) R[r][p] = add_node(c, l[p], wt[r][p] * unit);
    }
    c->nBody = c->nn;
    {   // body centre of mass, for car_local_to_world()
        V3 m = v3(0, 0, 0); float M = 0;
        for (int i = 0; i < c->nBody; i++) { m = vadd(m, vscale(L[i], c->n[i].m)); M += c->n[i].m; }
        c->comLocal = vscale(m, 1.0f / M);
    }

    // ---- body structure
    for (int r = 0; r < nr; r++)                      // ring frames / bulkheads
    for (int a = 0; a < NP; a++)
    for (int b = a + 1; b < NP; b++) {
        int perim = (b - a == 1) || (a == 0 && b == NP - 1);
        float K, Y;
        material(D, ST[r].x, is_floor(a) && is_floor(b), &K, &Y);
        // Cross-bracing: full bulkheads at the ends, axles and cabin rear; the dash and B-pillar
        // rings are open across the cabin (people sit there), so their webs are lighter.
        if (!perim) K *= !ST[r].bulk ? 0.35f : (r == D->dashRing || r == D->bRing) ? 0.25f : 1.0f;
        add_beam(c, R[r][a], R[r][b], K, Y, 0, BEAM_NORMAL);
    }
    for (int r = 0; r + 1 < nr; r++) {                // shell between rings
        float xm = 0.5f * (ST[r].x + ST[r + 1].x);
        for (int p = 0; p < NP; p++) {
            int q = (p + 1) % NP;
            float K, Y;
            int cab = r >= D->doorR0 && r <= D->doorR1, door = cab && is_door(p) && is_door(q);
            material(D, xm, cab && is_door(p) ? 2 : is_floor(p), &K, &Y);
            add_beam(c, R[r][p], R[r + 1][p], K, Y, 0, BEAM_NORMAL);             // longitudinal
            material(D, xm, door ? 2 : is_floor(p) && is_floor(q), &K, &Y);
            add_beam(c, R[r][p], R[r + 1][q], 0.8f * K, Y, 0, BEAM_NORMAL);      // shear panel
            add_beam(c, R[r][q], R[r + 1][p], 0.8f * K, Y, 0, BEAM_NORMAL);
        }
    }
    const int rails[] = {P_C, P_FL, P_FR, P_BLL, P_BLR, P_TPL, P_TPR};   // frame rails, belt creases, roof rails
    for (int r = 0; r + 2 < nr; r++)
    for (int i = 0; i < 7; i++) {
        int p = rails[i];
        float K, Y;
        material(D, ST[r + 1].x, is_floor(p), &K, &Y);
        add_beam(c, R[r][p], R[r + 2][p], 0.6f * K, Y, 0, BEAM_NORMAL);
    }

    // ---- suspension towers: everything attached to a strut node is a stiffer, stronger subframe
    {
        static unsigned char strut[MAX_NODES];
        memset(strut, 0, sizeof strut);
        const int sp[4] = {P_FL, P_FR, P_RKL, P_RKR};
        for (int i = 0; i < 4; i++) { strut[R[D->axleF][sp[i]]] = 1; strut[R[D->axleR][sp[i]]] = 1; }
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
    float xN = ST[0].x, xT = ST[nr - 1].x;
    for (int r = 0; r + 1 < nr; r++) {
        float xm = 0.5f * (ST[r].x + ST[r + 1].x);
        V3 ref = v3(clampf(xm, xT + 0.45f, xN - 0.45f), 0.40f, 0);
        for (int s = 0; s < NP; s++) {
            int p = s, q = (s + 1) % NP;
            int kind = SURF_BODY, panel = -1, e = 1 | 4;   // longitudinal edges are the creases
            if (s == 0 || s == 8) kind = SURF_UNDER;
            else if (s == 1 || s == 7) kind = SURF_TRIM;   // sill chamfer / wheel-well liner
            else if (s == 3 || s == 5) {                   // greenhouse side
                if (ST[r].sidePanel >= 0) panel = ST[r].sidePanel + (s == 5);
            } else if (s == 4) {                           // hood, windshield, roof, tonneau
                if (ST[r].topPanel >= 0) panel = ST[r].topPanel;
                else if (ST[r].topPanel == -2) kind = SURF_TONNEAU;
            }
            if (D->smooth) e = 0;
            if (panel >= 0) { kind = SURF_GLASS; e = 15; }
            if (ST[r].seam && s >= 2 && s <= 6) e |= 8;
            add_quad(c, R[r][p], R[r + 1][p], R[r + 1][q], R[r][q], kind, panel, e, ref);
        }
    }
    c->npanels = D->npanels;
    {   // nose: front face, fan from the floor centre
        V3 ref = v3(xN - 0.8f, 0.4f, 0);
        for (int i = 1; i < NP - 1; i++) {
            int kind = (i == 1 || i == NP - 2) ? SURF_TRIM : SURF_BODY;
            int e = D->smooth ? 0 : 2 | (i == 1 ? 1 : 0) | (i == NP - 2 ? 4 : 0);
            add_tri(c, R[0][P_C], R[0][i], R[0][i + 1], kind, -1, e, ref);
        }
    }
    int *T = R[nr - 1], *U = R[nr - 2];
    V3 tref = v3(xT + 0.8f, 0.4f, 0);
    if (D->hasTailgate) {   // tail: bumper below the tailgate opening, sail ends above it
        add_tri(c, T[P_C], T[P_FL], T[P_RKL], SURF_TRIM, -1, 3, tref);
        add_tri(c, T[P_C], T[P_RKL], T[P_RKR], SURF_TRIM, -1, 2, tref);
        add_tri(c, T[P_C], T[P_RKR], T[P_FR], SURF_TRIM, -1, 6, tref);
        add_tri(c, T[P_BLL], T[P_TPL], T[P_TPR], SURF_BODY, -1, 3, tref);
        add_tri(c, T[P_BLL], T[P_TPR], T[P_BLR], SURF_BODY, -1, 6, tref);
    } else {                // closed tail, like the nose
        for (int i = 1; i < NP - 1; i++) {
            int kind = (i == 1 || i == NP - 2) ? SURF_TRIM : SURF_BODY;
            int e = D->smooth ? 0 : 2 | (i == 1 ? 1 : 0) | (i == NP - 2 ? 4 : 0);
            add_tri(c, T[P_C], T[i], T[i + 1], kind, -1, e, tref);
        }
    }

    // ---- tailgate (pickup): a separate rigid part (6 nodes, 15 beams)
    if (D->hasTailgate) {
        const float tx = xT - 0.02f, tz = 0.93f, m = TG_MASS / 6.0f;
        int BL = add_node(c, v3(tx, 0.24f, -tz), m), BR = add_node(c, v3(tx, 0.24f, tz), m);
        int TL = add_node(c, v3(tx - 0.06f, 0.66f, -tz), m), TR = add_node(c, v3(tx - 0.06f, 0.66f, tz), m);  // raked out
        int ML = add_node(c, v3(tx + 0.06f, 0.45f, -0.90f), m), MR = add_node(c, v3(tx + 0.06f, 0.45f, 0.90f), m);
        int tg[6] = {BL, BR, TL, TR, ML, MR};
        for (int i = 0; i < 6; i++)
            for (int j = i + 1; j < 6; j++) add_beam(c, tg[i], tg[j], 420000.0f, 0.02f, 0, BEAM_NORMAL);
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
        V3 ref = v3(xT, 0.45f, 0);
        add_quad(c, BL, BR, TR, TL, SURF_BODY, -1, 15, ref);
        add_quad(c, TL, TR, MR, ML, SURF_TRIM, -1, 0, ref);
        add_quad(c, BL, ML, MR, BR, SURF_TRIM, -1, 0, ref);
        add_tri(c, BL, TL, ML, SURF_TRIM, -1, 0, ref);
        add_tri(c, BR, MR, TR, SURF_TRIM, -1, 0, ref);
        c->tgNode[0] = TL; c->tgNode[1] = TR; c->tgNode[2] = BL; c->tgNode[3] = BR;
        c->tailgate = TG_LATCHED;
    } else c->tailgate = TG_NONE;

    // ---- lights
    if (D->lightStyle == 0) {                                   // full-width bars
        add_light(c, R[0][P_TPL], R[0][P_TPR], R[0][P_BLL], R[0][P_BLR], 0.45f, 0, 1, 1.0f, 0);
        add_light(c, c->tgNode[0], c->tgNode[1], c->tgNode[2], c->tgNode[3], 0.09f, 0, 1, -1.0f, 1);
    } else {                                                    // headlamps and tail-light clusters at the corners
        // Slim headlamps along the top of the front fascia, tail lamps across the top of the rear.
        add_light(c, R[0][P_TPL], R[0][P_TPR], R[0][P_BLL], R[0][P_BLR], 0.85f, 0.00f, 0.30f, 1.0f, 0);
        add_light(c, R[0][P_TPL], R[0][P_TPR], R[0][P_BLL], R[0][P_BLR], 0.85f, 0.70f, 1.00f, 1.0f, 0);
        add_light(c, T[P_TPL], T[P_TPR], T[P_BLL], T[P_BLR], 0.70f, 0.00f, 0.30f, -1.0f, 1);
        add_light(c, T[P_TPL], T[P_TPR], T[P_BLL], T[P_BLR], 0.70f, 0.70f, 1.00f, -1.0f, 1);
    }

    // ---- attachment points for render-only details
    for (int i = 0; i < 4; i++) c->mirrorNode[i] = c->spoilerNode[i] = -1;
    if (D->mirrorRing >= 0) {
        int m = D->mirrorRing;
        c->mirrorNode[0] = R[m][P_BLL]; c->mirrorNode[1] = R[m][P_BLR];
        c->mirrorNode[2] = R[m + 1][P_BLL]; c->mirrorNode[3] = R[m + 1][P_BLR];
    }
    if (D->spoilerRing >= 1) {
        int sr = D->spoilerRing;
        c->spoilerNode[0] = R[sr][P_TPL]; c->spoilerNode[1] = R[sr][P_TPR];
        c->spoilerNode[2] = R[sr - 1][P_TPL]; c->spoilerNode[3] = R[sr - 1][P_TPR];
    }

    // Pillars: the windshield's side edges (roof-edge nodes over its glazed gaps) and the B-pillar ring.
    c->naPillar = 0;
    for (int r = 0; r + 1 < nr && c->naPillar < 7; r++)
        if (ST[r].topPanel == 0) {
            if (!c->naPillar) { c->aPillar[0][0] = R[r][P_TPL]; c->aPillar[1][0] = R[r][P_TPR]; c->naPillar = 1; }
            c->aPillar[0][c->naPillar] = R[r + 1][P_TPL]; c->aPillar[1][c->naPillar] = R[r + 1][P_TPR]; c->naPillar++;
        }
    c->bPillar[0] = R[D->bRing][P_BLL]; c->bPillar[1] = R[D->bRing][P_TPL];
    c->bPillar[2] = R[D->bRing][P_BLR]; c->bPillar[3] = R[D->bRing][P_TPR];

    // ---- place in the world
    float cy = cosf(yaw), sy = sinf(yaw);
    for (int n = 0; n < c->nn; n++) {
        V3 l = L[n];
        c->n[n].p = v3(pos.x + l.x * cy + l.z * sy, pos.y + l.y, pos.z - l.x * sy + l.z * cy);
        c->n[n].v = v3(0, 0, 0);
    }

    // ---- wheels: FL, FR, RL, RR on the floor-edge nodes at the axle bulkheads
    for (int w = 0; w < 4; w++) {
        int left = (w % 2 == 0), ring = w < 2 ? D->axleF : D->axleR;
        Wheel *W = &c->w[w];
        W->mount = R[ring][left ? P_FL : P_FR];
        W->upper = R[ring][left ? P_RKL : P_RKR];
        W->off = (left ? -1.0f : 1.0f) * (D->trackHalf - D->floorHalf);
        W->front = w < 2;
        W->driveShare = 0.5f * (W->front ? D->driveFront : 1.0f - D->driveFront);
    }
    car_update_frame(c);
    for (int w = 0; w < 4; w++) {
        Wheel *W = &c->w[w];
        W->hub = vsub(vadd(c->n[W->mount].p, vscale(c->right, W->off)), vscale(c->up, D->suspRest));
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
    if (c->def && c->def->hasTailgate)
        c->tailgate = (hl || hr) ? (latched ? TG_LATCHED : (hl && hr ? TG_OPEN : TG_HANGING)) : TG_GONE;
}

void car_pop_tailgate(Car *c) {
    if (c->tailgate != TG_LATCHED) return;
    for (int i = 0; i < 6; i++) if (c->tgLatch[i] >= 0) c->b[c->tgLatch[i]].broken = 1;
    // A small shove outward, like the latch spring.
    for (int i = 0; i < 2; i++) {
        Node *n = &c->n[c->tgNode[i]];
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
    const VehicleDef *D = c->def;
    float vF = vdot(c->vel, c->fwd);

    // Steering: speed-sensitive lock, rate-limited. Rear wheels counter-steer at low speed.
    float maxLock = D->maxLock / (1.0f + fabsf(vF) / 22.0f);
    float target = clampf(ctl->steer, -1, 1) * maxLock;
    c->steer += clampf(target - c->steer, -2.2f * h, 2.2f * h);
    float rearSteer = -c->steer * D->rearSteer * clampf(1.0f - fabsf(vF) / 18.0f, 0.0f, 1.0f);

    float drive = 0;
    if (ctl->throttle > 0) drive = ctl->throttle * fminf(D->engineF, D->enginePower / fmaxf(fabsf(vF), 2.0f));
    else if (ctl->throttle < 0 && vF > -12.0f) drive = ctl->throttle * D->reverseF;

    for (int i = 0; i < 4; i++) {
        Wheel *W = &c->w[i];
        Node *m = &c->n[W->mount], *u = &c->n[W->upper];
        V3 up = c->up;
        V3 org = vadd(m->p, vscale(c->right, W->off));
        W->steerAngle = W->front ? c->steer : rearSteer;
        W->contact = 0; W->load = 0; W->slip = 0;

        float gh = terrain_height(org.x, org.z);
        float dist = up.y > 0.25f ? (org.y - gh) / up.y : 1e9f;   // along -up to ground
        float comp = D->suspRest + D->wheelR - dist;
        V3 vel = vscale(vadd(m->v, u->v), 0.5f);

        V3 wf = vrot(c->fwd, up, W->steerAngle);

        if (comp <= 0) {
            W->compress = 0;
            W->hub = vsub(org, vscale(up, D->suspRest));
            W->side = vnorm(vcross(wf, up));
            W->spin += (ctl->throttle > 0.1f ? 30.0f : 0.0f) * h;
            continue;
        }

        W->contact = 1;
        W->compress = comp;
        float vcomp = -vdot(vel, up);
        float Fs = D->ks * comp + D->cs * vcomp;
        if (comp > D->suspTravel) Fs += D->kbump * (comp - D->suspTravel);
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
        float mu = MU_TIRE * c->grip * (hb ? 0.55f : 1.0f);

        float Fy = -vs * N * LAT_STIFF;
        float Fx = drive * W->driveShare;
        float bf = D->brakeMax * clampf(ctl->brake, 0, 1);
        if (hb) { bf = fmaxf(bf, mu * N); Fx = 0; }
        Fx -= clampf(vl * D->mass * 3.0f, -bf, bf);
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
        W->hub = vsub(org, vscale(up, dist - D->wheelR));
        W->cp = vsub(W->hub, vscale(n, D->wheelR));
        if (!hb) W->spin += (vl / D->wheelR) * h;
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
            N->f = vadd(N->f, vscale(N->v, -c->def->cdA * sp * N->m / c->def->mass));   // aero drag, spread by mass
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

// ------------------------------------------------------------- the maps ---
//
// Both maps share the proving ground: highway along x, cross road along z, launch ramp, crash wall,
// pillars, plaza barriers, a warehouse district south of the highway, and forest on the hills. The
// headless test routes stay open on both.
//
//  Default:  two small towns north of the highway, either side of the cross road (dirt washboard),
//            with an avenue and side streets.
//  New York: Midtown. Avenues along z (7th in the middle, 6th and 8th either side), cross streets
//            along x, and Broadway on a diagonal through the "bowtie" where it crosses 7th Avenue.
//            Tall towers fill the blocks; the facades around the bowtie carry LED billboards (drawn
//            by the renderer). The washboard moves south of the highway.

typedef struct { float x0, x1, z0, z1; } Rect;

static const Rect ROADS[] = {
    {-260, 260, -7, 7},          // highway
    {-7, 7, -260, 260},          // cross road (7th Avenue through Midtown)
};
static const Rect DEF_STREETS[] = {
    {-112, 112, 44, 52},         // avenue
    {-80, 80, 94, 100},          // upper avenue
    {36, 44, 7, 136}, {76, 84, 7, 94}, {-44, -36, 7, 136}, {-84, -76, 7, 94},   // side streets
    {30, 78, -13, -7},           // service road into the warehouse district
};
static const Rect NY_STREETS[] = {
    {-8, 8, CITY_Z0, CITY_Z1},                                   // 7th Avenue, full width
    {48, 60, CITY_Z0, CITY_Z1}, {-60, -48, CITY_Z0, CITY_Z1},    // 6th and 8th Avenues
    {-CITY_X, CITY_X, 44, 50}, {-CITY_X, CITY_X, 72, 78},        // cross streets
    {-CITY_X, CITY_X, 100, 106}, {-CITY_X, CITY_X, 124, CITY_Z1},
    {30, 78, -13, -7},                                           // service road into the warehouse district
};
// No scenery here: ramp + landing run-out, crash wall, spawn, and the headless test routes.
static const Rect KEEP_CLEAR[] = {
    {30, 225, -12, 12}, {-82, -64, -18, 18}, {-16, 16, -16, 16},
    {-82, 72, -72, -38}, {-82, -14, -72, 8},
};
#define NELEM(a) ((int)(sizeof(a) / sizeof((a)[0])))

static const char *MAP_NAMES[MAP_COUNT] = {"DEFAULT", "NEW YORK"};
const char *map_name(int map) { return MAP_NAMES[map < 0 || map >= MAP_COUNT ? 0 : map]; }
int map_has_broadway(void) { return g_map == MAP_NEWYORK; }
#define STREETS (g_map == MAP_NEWYORK ? NY_STREETS : DEF_STREETS)
#define NSTREETS (g_map == MAP_NEWYORK ? NELEM(NY_STREETS) : NELEM(DEF_STREETS))

float broadway_x(float z) { return TS_X + BW_DX * (z - TS_Z); }
float broadway_dist(float x, float z) { return fabsf(x - broadway_x(z)) / sqrtf(1.0f + BW_DX * BW_DX); }
static int on_broadway(float x, float z, float m) { return g_map == MAP_NEWYORK && z > CITY_Z0 && z < CITY_Z1 && broadway_dist(x, z) < BW_HALF + m; }
static int in_city(float x, float z) { return g_map == MAP_NEWYORK && fabsf(x) < CITY_X + 6 && z > CITY_Z0 - 2 && z < CITY_Z1 + 6; }

static int in_rect(const Rect *r, float x, float z, float m) {
    return x > r->x0 - m && x < r->x1 + m && z > r->z0 - m && z < r->z1 + m;
}
static int rect_overlap(const Rect *a, const Rect *b, float m) {
    return a->x0 < b->x1 + m && b->x0 < a->x1 + m && a->z0 < b->z1 + m && b->z0 < a->z1 + m;
}

int road_at(float x, float z) {
    for (int i = 0; i < NELEM(ROADS); i++) if (in_rect(&ROADS[i], x, z, 0)) return 1;
    for (int i = 0; i < NSTREETS; i++) if (in_rect(&STREETS[i], x, z, 0)) return 2;
    if (on_broadway(x, z, 0)) return fabsf(z - TS_Z) < PLAZA_HALF ? 3 : 2;
    return 0;
}

int road_rects(float (*out)[4], int max, int *nHighway) {
    int n = 0;
    for (int i = 0; i < NELEM(ROADS) && n < max; i++, n++) { out[n][0] = ROADS[i].x0; out[n][1] = ROADS[i].x1; out[n][2] = ROADS[i].z0; out[n][3] = ROADS[i].z1; }
    *nHighway = n;
    for (int i = 0; i < NSTREETS && n < max; i++, n++) { out[n][0] = STREETS[i].x0; out[n][1] = STREETS[i].x1; out[n][2] = STREETS[i].z0; out[n][3] = STREETS[i].z1; }
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
    for (int i = 0; i < NSTREETS; i++) if (rect_overlap(&r, &STREETS[i], rm)) return 0;
    for (int i = 0; i < NELEM(KEEP_CLEAR); i++) if (rect_overlap(&r, &KEEP_CLEAR[i], 0)) return 0;
    if (g_map == MAP_NEWYORK && r.z1 > CITY_Z0 && r.z0 < CITY_Z1) {   // Broadway: corners on both sides, or any sample point too close
        float s0 = r.x0 - broadway_x(r.z0), s1 = r.x1 - broadway_x(r.z0), s2 = r.x0 - broadway_x(r.z1), s3 = r.x1 - broadway_x(r.z1);
        if (!((s0 > 0 && s1 > 0 && s2 > 0 && s3 > 0) || (s0 < 0 && s1 < 0 && s2 < 0 && s3 < 0))) return 0;
        for (int i = 0; i <= 4; i++)
            for (int j = 0; j <= 4; j++)
                if (on_broadway(lerpf(r.x0, r.x1, i * 0.25f), lerpf(r.z0, r.z1, j * 0.25f), rm)) return 0;
    }
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
    if (zone == 4) return u < 0.62f ? BLD_TOWER : u < 0.92f ? BLD_APARTMENT : BLD_SHOP;          // Midtown
    return u < 0.75f ? BLD_WAREHOUSE : BLD_SHOP;                                  // industrial
}

// Frontage width, depth and wall height for a kind.
static void pick_size(int k, int zone, float *wdt, float *dep, float *ht) {
    if (zone == 4) {                                // Midtown: big floor plates, tall
        switch (k) {
            case BLD_TOWER: *wdt = frange(11, 20); *dep = frange(12, 20); *ht = frange(55, 140); return;
            case BLD_APARTMENT: *wdt = frange(12, 20); *dep = frange(12, 18); *ht = 3.2f * (int)frange(8, 15) + 0.8f; return;
            default: *wdt = frange(10, 16); *dep = frange(10, 14); *ht = frange(9, 16); return;
        }
    }
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
    const float SET = bk->zone == 4 ? 0.8f : 1.5f;     // setback from the kerb (Midtown builds right up to it)
    int first = 1;
    // Lots along each street edge: south, north, west, east.
    for (int e = 0; e < 4; e++) {
        int alongX = e < 2;
        float len = alongX ? R->x1 - R->x0 : R->z1 - R->z0;
        float cur = frange(0.5f, 2.5f);
        while (cur < len - 6) {
            int k = pick_kind(bk->zone, first);
            float wdt, dep, ht;
            pick_size(k, bk->zone, &wdt, &dep, &ht);
            if (bk->zone == 4) {                    // Midtown: lots on facing sides share the block depth
                float across = e < 2 ? R->z1 - R->z0 : R->x1 - R->x0;
                float half = across * 0.5f - SET - 0.3f;
                dep = across < 24.0f ? across - 2.0f * SET : fminf(dep, half);
                if (dep < 6.0f) break;
            }
            if (k == BLD_TOWER) { if (cur + wdt > len - 1) { cur += 2; continue; } }
            if (cur + wdt > len - 1) wdt = len - 1 - cur;
            if (wdt < 7.5f) break;
            Rect fp; int front;
            if (e == 0) { fp = (Rect){R->x0 + cur, R->x0 + cur + wdt, R->z0 + SET, R->z0 + SET + dep}; front = 3; }
            else if (e == 1) { fp = (Rect){R->x0 + cur, R->x0 + cur + wdt, R->z1 - SET - dep, R->z1 - SET}; front = 2; }
            else if (e == 2) { fp = (Rect){R->x0 + SET, R->x0 + SET + dep, R->z0 + cur, R->z0 + cur + wdt}; front = 1; }
            else { fp = (Rect){R->x1 - SET - dep, R->x1 - SET, R->z0 + cur, R->z0 + cur + wdt}; front = 0; }
            float gap = bk->zone == 4 ? 0.6f : (k == BLD_HOUSE ? 2.2f : 1.6f);
            if (fp.x1 <= R->x1 && fp.z1 <= R->z1 && area_free(w, fp, gap)) {
                add_building(w, fp, k, ht, front);
                first = 0;
                cur += wdt + (bk->zone == 4 ? frange(0.6f, 1.4f) : k == BLD_HOUSE ? frange(2.5f, 5.0f) : frange(1.8f, 3.5f));
            } else cur += 2.0f;
        }
    }
    if (bk->zone == 4) {                                // Midtown: second row of towers behind the frontage
        for (int t = 0; t < 40; t++) {
            int k = frnd() < 0.7f ? BLD_TOWER : BLD_APARTMENT;
            float wdt, dep, ht;
            pick_size(k, 4, &wdt, &dep, &ht);
            if (wdt > R->x1 - R->x0 - 2 || dep > R->z1 - R->z0 - 2) continue;
            float x0 = frange(R->x0 + 1, R->x1 - 1 - wdt), z0 = frange(R->z0 + 1, R->z1 - 1 - dep);
            Rect fp = {x0, x0 + wdt, z0, z0 + dep};
            if (area_free(w, fp, 0.6f)) add_building(w, fp, k, ht, z0 - R->z0 < R->z1 - (z0 + dep) ? 3 : 2);
        }
        return;
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

// Default map: two small towns, mirrored across the cross road, plus the warehouse district.
static void build_default(World *w) {
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
    Block ind = {{34, 74, -38, -14}, 3};          // warehouse district (east side only; the west is a test route)
    fill_block(w, &ind);
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
}

// New York map: Times Square landmarks, Midtown blocks, then the warehouse district.
static void build_newyork(World *w) {
    // Landmarks first, so the blocks pack around them: the screen-clad wedge tower in the south gore
    // of the bowtie (between Broadway and 7th Avenue), and the red grandstand steps on the Broadway
    // plaza north of the crossing.
    add_building(w, (Rect){-18.0f, -9.6f, 24.0f, 38.0f}, BLD_WEDGE, 112.0f, 2);
    float sx = broadway_x(92.0f);
    add_building(w, (Rect){sx - 3.2f, sx + 3.2f, 86.0f, 97.0f}, BLD_STEPS, 3.4f, 3);
    // Midtown blocks between the avenues and cross streets.
    const float xs[4][2] = {{-CITY_X + 1, -60}, {-48, -8}, {8, 48}, {60, CITY_X - 1}};
    const float zs[4][2] = {{14, 44}, {50, 72}, {78, 100}, {106, 124}};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            Block b = {{xs[i][0], xs[i][1], zs[j][0], zs[j][1]}, 4};
            fill_block(w, &b);
        }
    Block ind = {{34, 74, -38, -14}, 3};
    fill_block(w, &ind);
}

void world_init(World *w, int map) {
    memset(w, 0, sizeof *w);
    w->map = g_map = (map < 0 || map >= MAP_COUNT) ? MAP_DEFAULT : map;
    rng_state = 20240611u;
    // Crash-test wall behind the spawn point.
    add_box(w, v3(-74, -1, -14), v3(-71, 3.0f, 14));
    // A row of concrete pillars beside the main road (on New York, moved clear of the avenue mouths).
    const float pxD[6] = {-60, -46, -32, -18, 18, 32}, pxN[6] = {-66, -40, -30, -18, 18, 32};
    for (int i = 0; i < 6; i++) {
        float x = g_map == MAP_NEWYORK ? pxN[i] : pxD[i];
        add_box(w, v3(x - 0.6f, -1, 11.0f), v3(x + 0.6f, 4.0f, 12.2f));
    }
    // Low barrier blocks scattered on the plaza.
    add_box(w, v3(18, -1, -26), v3(30, 0.8f, -24.5f));
    add_box(w, v3(-30, -1, -30), v3(-28.5f, 1.2f, -18));

    if (g_map == MAP_NEWYORK) build_newyork(w); else build_default(w);

    // Roadside trees along the highway and the cross road, out in the country.
    float crossNorth = g_map == MAP_NEWYORK ? 142.0f : 100.0f;
    for (float x = 66; x < 250; x += 11) { try_tree(w, x, 14.5f + frange(-1, 1), frange(6, 9), TREE_BROADLEAF, 2.0f); try_tree(w, x + 5, -14.5f + frange(-1, 1), frange(6, 9), TREE_BROADLEAF, 2.0f); }
    for (float x = -86; x > -250; x -= 11) { try_tree(w, x, 14.5f + frange(-1, 1), frange(6, 9), TREE_BROADLEAF, 2.0f); try_tree(w, x - 5, -14.5f + frange(-1, 1), frange(6, 9), TREE_BROADLEAF, 2.0f); }
    for (float z = crossNorth; z < 250; z += 12) { try_tree(w, 12, z, frange(6, 10), TREE_CONIFER, 2.0f); try_tree(w, -12, z + 6, frange(6, 10), TREE_CONIFER, 2.0f); }
    for (float z = -80; z > -250; z -= 12) { try_tree(w, 12, z, frange(6, 10), TREE_CONIFER, 2.0f); try_tree(w, -12, z - 6, frange(6, 10), TREE_CONIFER, 2.0f); }

    // Forest on the hills, thicker further out; a few loners on the flat.
    for (int i = 0; i < 2600 && w->ntree < MAX_TREES - 40; i++) {
        float r = frange(95, 290), a = frange(0, 6.2831853f);
        float x = r * cosf(a), z = r * sinf(a);
        if (in_city(x, z)) continue;
        if (frnd() > smooth01(90, 200, r) + 0.15f) continue;
        int conifer = frnd() < 0.35f + 0.5f * smooth01(110, 220, r);
        try_tree(w, x, z, conifer ? frange(8, 15) : frange(6, 10), conifer ? TREE_CONIFER : TREE_BROADLEAF, 3.2f);
    }
    for (int i = 0; i < 120; i++) {
        float x = frange(-88, 88), z = frange(-88, 88);
        if (x * x + z * z > 88 * 88 || in_city(x, z)) continue;
        try_tree(w, x, z, frange(5, 9), frnd() < 0.7f ? TREE_BROADLEAF : TREE_CONIFER, 6.0f);
    }

    world_build_grid(w);
    w->ncars = 0;
}
