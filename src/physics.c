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
#include <stdlib.h>

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

#ifdef __APPLE__
#include <os/lock.h>
static os_unfair_lock treeLock = OS_UNFAIR_LOCK_INIT;
#define TREE_LOCK() os_unfair_lock_lock(&treeLock)
#define TREE_UNLOCK() os_unfair_lock_unlock(&treeLock)
#else
#define TREE_LOCK() ((void)0)
#define TREE_UNLOCK() ((void)0)
#endif
#define BLD_STRENGTH_K 420.0f     // N s of impact per m^2 of (volume^2/3): the one knob for how tough buildings are
#define BLD_HIT_SPEED 2.5f        // m/s into a wall before a contact counts as damage (resting contact does not)
static int g_map;                                 // map the world was last built with (terrain depends on it)
static float wash_z0(void) { return g_map == MAP_NEWYORK ? -150.0f : g_map == MAP_AIRPORT ? 1.0f : 40.0f; }    // (none on the airport)
static float wash_z1(void) { return g_map == MAP_NEWYORK ? -90.0f : g_map == MAP_AIRPORT ? 0.0f : 140.0f; }
int dirt_at(float x, float z) { return fabsf(x) < 8.0f && z > wash_z0() && z < wash_z1(); }

// Airport map: the bay. North of the field for its whole width; east of it the shore swings south,
// and the runway carries on out into the water on a causeway. Returns metres seaward of the nearest
// shore (negative on land).
float sea_dist(float x, float z) {
    if (g_map != MAP_AIRPORT) return -1e9f;
    float shore = 252.0f - 190.0f * smooth01(300.0f, 420.0f, x) + 14.0f * sinf(x * 0.013f) + 5.0f * sinf(x * 0.041f + 1.0f);
    float ds = z - shore;
    float dc = fmaxf(x - (AP_RWY_X1 + 75.0f), fmaxf(116.0f - z, z - 216.0f));   // outside the causeway
    return fminf(ds, dc);
}

// ---- rubble mounds: a collapsed building leaves a pile that is part of the ground (wheels drive up it,
// nodes rest on it). k rises 0..1 through the collapse.
typedef struct { float x0, x1, z0, z1, ramp, H, k, ph; } Mound;
static Mound g_mound[MAX_BLD];
static int g_moundOf[MAX_BLD];       // the building each mound belongs to
static int g_nmound;
static void make_mound(const Building *B, Mound *m) {
    const float skirt = 0.8f;
    m->x0 = B->mn.x - skirt; m->x1 = B->mx.x + skirt; m->z0 = B->mn.z - skirt; m->z1 = B->mx.z + skirt;
    float half = 0.5f * fminf(m->x1 - m->x0, m->z1 - m->z0);
    m->ramp = fminf(half, fmaxf(4.0f, 3.0f * B->moundH));
    m->H = B->moundH; m->k = 1.0f;
    m->ph = (float)(B->seed % 1000u) * 0.01f;
}
static float mound_at(const Mound *m, float x, float z) {
    float e = fminf(fminf(x - m->x0, m->x1 - x), fminf(z - m->z0, m->z1 - z));
    if (e <= 0.0f) return 0.0f;
    float t = fminf(e / m->ramp, 1.0f);
    float bumps = 0.88f + 0.12f * sinf(x * 0.9f + m->ph) * cosf(z * 0.8f - m->ph);
    return m->H * m->k * t * t * (3.0f - 2.0f * t) * bumps;
}
static float terrain_base(float x, float z);
float terrain_height(float x, float z) {
    float h = terrain_base(x, z);
    for (int i = 0; i < g_nmound; i++) {
        const Mound *m = &g_mound[i];
        if (x > m->x0 && x < m->x1 && z > m->z0 && z < m->z1) h += mound_at(m, x, z);
    }
    return h;
}
float building_mound(const World *w, int b, float x, float z) {
    if (b < 0 || b >= w->nbld) return 0.0f;
    Mound m; make_mound(&w->bld[b], &m);
    return mound_at(&m, x, z);
}

static float terrain_base(float x, float z) {
    float r = sqrtf(x * x + z * z);
    float hills = smooth01(90.0f, 170.0f, r);
    float far = 0.0f;
    if (g_map == MAP_AIRPORT) {
        float d = 1e9f;                                  // distance outside the graded airfield and causeway
        if (z > -20.0f) d = fmaxf(fmaxf(0.0f, fabsf(x) - 262.0f), fmaxf(0.0f, z - 218.0f));
        if (x > 200.0f) d = fminf(d, fmaxf(fmaxf(0.0f, x - (AP_RWY_X1 + 70.0f)), fmaxf(0.0f, fmaxf(118.0f - z, z - 214.0f))));
        if (d < 1e8f) hills *= smooth01(0.0f, 60.0f, d);
        // Coastal hills well away from the field, for something to fly around.
        if (r > 600.0f) far = smooth01(600.0f, 2000.0f, r) * smooth01(0.0f, 500.0f, fminf(d, 500.0f)) * smooth01(10.0f, 700.0f, -sea_dist(x, z)) *
                              (55.0f + 70.0f * sinf(x * 0.0017f + 0.6f) * cosf(z * 0.0013f) + 35.0f * sinf(x * 0.0041f - z * 0.0037f));
    }
    float h = hills * (7.0f * sinf(x * 0.019f) * cosf(z * 0.016f) +
                       3.0f * sinf(x * 0.047f + z * 0.031f + 1.3f) +
                       1.2f * sinf(z * 0.11f - x * 0.05f)) + far;
    if (g_map == MAP_AIRPORT) {
        float sd = sea_dist(x, z);
        if (sd > 0.0f) h = lerpf(h, -4.5f - 40.0f * smooth01(40.0f, 600.0f, sd), smooth01(0.0f, 16.0f, sd));   // beach, then the seabed
    }
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

// ---- Long-nose semi: a classic conventional tractor with a sleeper, bobtail. 8.25 m long, 2.45 m
// over the fenders, 3.0 m tall, 8.2 t, three axles (tandem drive, dual tyres). The body rings are the
// narrow butterfly hood, the cab and sleeper box, and the bare frame behind; bumper, fenders, stacks,
// tanks and the rest of the brightwork are drawn by the renderer on these nodes.
static void define_semi(VehicleDef *d) {
    static const Station S[] = {
        { 3.80f, 0.12f, 0.50f, 1.26f, 1.40f, 0.50f, 0.52f, 0, 1, 0, -1, -1},   //  0 grille
        { 3.60f, 0.04f, 0.46f, 1.28f, 1.43f, 0.50f, 0.52f, 0, 0, 0, -1, -1},   //  1
        { 3.25f, 0.00f, 0.45f, 1.30f, 1.45f, 0.50f, 0.52f, 0, 0, 0, -1, -1},   //  2
        { 2.95f, 0.00f, 0.45f, 1.31f, 1.47f, 0.50f, 0.52f, 0, 1, 0, -1, -1},   //  3 front axle
        { 2.55f, 0.00f, 0.45f, 1.32f, 1.49f, 0.50f, 0.53f, 0, 0, 0, -1, -1},   //  4
        { 2.10f, 0.00f, 0.45f, 1.33f, 1.51f, 0.51f, 0.54f, 0, 0, 0, -1, -1},   //  5
        { 1.70f, 0.00f, 0.45f, 1.34f, 1.53f, 0.52f, 0.56f, 0, 1, 0, -1, -1},   //  6 hood rear / cowl
        { 1.55f, 0.00f, 0.35f, 1.50f, 1.58f, 0.98f, 0.875f, 0, 1, 1,  0, -1},  //  7 firewall, dash, windshield base
        { 1.36f, 0.00f, 0.35f, 1.50f, 2.42f, 0.93f, 0.875f, 0, 0, 1, -1,  1},  //  8 windshield top / door front
        { 0.80f, 0.00f, 0.35f, 1.50f, 2.46f, 0.93f, 0.875f, 0, 0, 0, -1,  1},  //  9
        { 0.30f, 0.00f, 0.35f, 1.50f, 2.46f, 0.93f, 0.875f, 0, 1, 1, -1, -1},  // 10 cab rear / sleeper front
        {-0.60f, 0.00f, 0.35f, 1.50f, 2.46f, 0.93f, 0.875f, 0, 0, 0, -1, -1},  // 11 sleeper
        {-1.45f, 0.00f, 0.35f, 1.50f, 2.46f, 0.93f, 0.875f, 0, 1, 1, -1, -1},  // 12 sleeper rear
        {-1.56f, 0.00f, 0.20f, 0.40f, 0.46f, 0.45f, 0.40f, 0, 0, 0, -2, -1},   // 13 frame deck
        {-2.00f, 0.00f, 0.20f, 0.40f, 0.46f, 0.45f, 0.40f, 0, 0, 0, -2, -1},   // 14
        {-2.40f, 0.00f, 0.20f, 0.40f, 0.46f, 0.45f, 0.40f, 0, 0, 0, -2, -1},   // 15
        {-2.75f, 0.00f, 0.20f, 0.40f, 0.46f, 0.45f, 0.40f, 0, 1, 0, -2, -1},   // 16 first drive axle
        {-3.40f, 0.00f, 0.20f, 0.40f, 0.46f, 0.45f, 0.40f, 0, 0, 0, -2, -1},   // 17
        {-4.05f, 0.00f, 0.20f, 0.40f, 0.46f, 0.45f, 0.40f, 0, 1, 0, -2, -1},   // 18 second drive axle
        {-4.45f, 0.00f, 0.20f, 0.40f, 0.46f, 0.45f, 0.40f, 0, 1, 0, -1, -1},   // 19 frame end
    };
    d->nr = (int)(sizeof S / sizeof S[0]);
    for (int r = 0; r < d->nr; r++) d->st[r] = S[r];
    d->name = "LONG-NOSE SEMI";
    d->blurb = "8.2 t  |  600 hp tandem drive  |  flame livery";
    d->floorHalf = 0.55f; d->sillHalf = 1.20f; d->beltHalf = 1.20f;
    d->axleF = 3; d->axleR = 16; d->axleR2 = 18; d->doorR0 = 8; d->doorR1 = 10; d->dashRing = 7; d->bRing = 10;
    d->mass = 8200.0f;
    d->batW = 1.5f; d->batX0 = -1.40f; d->batX1 = 0.40f;      // fuel tanks under the cab
    d->engW = 3.0f; d->engX0 = 1.60f; d->engX1 = 3.40f;       // 15 L diesel and gearbox under the hood
    d->crumpleF = 3.10f; d->crumpleR = -4.20f; d->cabinX0 = -1.45f; d->cabinX1 = 1.56f;
    d->kScale = 2.4f; d->yScale = 1.0f;
    d->wheelR = 0.52f; d->wheelW = 0.30f; d->trackHalf = 1.02f; d->wheelWR = 0.62f; d->trackHalfR = 0.91f;
    d->suspRest = 0.05f; d->suspTravel = 0.20f; d->ks = 230000.0f; d->cs = 15000.0f; d->kbump = 1600000.0f;
    d->brakeMax = 9400.0f; d->engineF = 27000.0f; d->enginePower = 450000.0f; d->reverseF = 12000.0f; d->driveFront = 0.0f;
    d->rearSteer = 0.0f; d->maxLock = 0.66f; d->cdA = 3.6f;
    d->spawnH = 0.56f;
    d->seat = v3(0.72f, 2.02f, -0.52f); d->dashX = 1.36f; d->dashY = 1.60f; d->rearSeatX = -0.75f;
    d->npanels = 3; d->hasTailgate = 0; d->lightStyle = 2;
    d->smooth = 0; d->wheelStyle = 2; d->mirrorRing = -1; d->spoilerRing = -1;
    d->noEdges = 1; d->chromeNose = 1;
    d->paint = v3(0.05f, 0.09f, 0.42f);                       // deep blue under the flames
}

const VehicleDef *vehicle_def(int type) {
    if (!defsReady) {
        define_pickup(&DEFS[VEH_PICKUP]); define_hatch(&DEFS[VEH_HATCH]); define_semi(&DEFS[VEH_SEMI]);
        DEFS[VEH_PICKUP].axleR2 = DEFS[VEH_HATCH].axleR2 = -1;
        defsReady = 1;
    }
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
        if ((p == P_FL || p == P_FR) && (r == D->axleF || r == D->axleR || r == D->axleR2)) w += 1.0f;
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
        for (int i = 0; i < 4; i++) {
            strut[R[D->axleF][sp[i]]] = 1; strut[R[D->axleR][sp[i]]] = 1;
            if (D->axleR2 >= 0) strut[R[D->axleR2][sp[i]]] = 1;
        }
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
            if (D->smooth || D->noEdges) e = 0;
            if (panel >= 0) { kind = SURF_GLASS; e = 15; }
            if (ST[r].seam && s >= 2 && s <= 6) e |= 8;
            add_quad(c, R[r][p], R[r + 1][p], R[r + 1][q], R[r][q], kind, panel, e, ref);
        }
    }
    c->npanels = D->npanels;
    {   // nose: front face, fan from the floor centre
        V3 ref = v3(xN - 0.8f, 0.4f, 0);
        for (int i = 1; i < NP - 1; i++) {
            int kind = (i == 1 || i == NP - 2) ? SURF_TRIM : D->chromeNose ? SURF_CHROME : SURF_BODY;
            int e = (D->smooth || D->noEdges) ? 0 : 2 | (i == 1 ? 1 : 0) | (i == NP - 2 ? 4 : 0);
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
            int e = (D->smooth || D->noEdges) ? 0 : 2 | (i == 1 ? 1 : 0) | (i == NP - 2 ? 4 : 0);
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
    if (D->lightStyle == 2) {
        // lamps are render-only details
    } else if (D->lightStyle == 0) {                            // full-width bars
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
    for (int n = 0; n < c->nn; n++) c->rest[n] = L[n];
    float cy = cosf(yaw), sy = sinf(yaw);
    for (int n = 0; n < c->nn; n++) {
        V3 l = L[n];
        c->n[n].p = v3(pos.x + l.x * cy + l.z * sy, pos.y + l.y, pos.z - l.x * sy + l.z * cy);
        c->n[n].v = v3(0, 0, 0);
    }

    // ---- wheels: FL, FR, RL, RR on the floor-edge nodes at the axle bulkheads
    c->nw = D->axleR2 >= 0 ? 6 : 4;
    for (int w = 0; w < c->nw; w++) {
        int left = (w % 2 == 0), ring = w < 2 ? D->axleF : w < 4 ? D->axleR : D->axleR2;
        Wheel *W = &c->w[w];
        W->mount = R[ring][left ? P_FL : P_FR];
        W->upper = R[ring][left ? P_RKL : P_RKR];
        float track = (w >= 2 && D->trackHalfR > 0) ? D->trackHalfR : D->trackHalf;
        W->off = (left ? -1.0f : 1.0f) * (track - D->floorHalf);
        W->front = w < 2;
        W->driveShare = (W->front ? D->driveFront : 1.0f - D->driveFront) / (W->front ? 2.0f : (float)(c->nw - 2));
    }
    car_update_frame(c);
    for (int w = 0; w < c->nw; w++) {
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

    for (int i = 0; i < c->nw; i++) {
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
        // The ground pushes back along its own normal; any sideways part of the strut force is taken by
        // the suspension links, not the road. (Pushing along the body's up made a pitched car creep.)
        V3 fs = vscale(n, N * 0.5f);
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
                if (B->dead) continue;
                if (p.x < B->mn.x || p.x > B->mx.x || p.y < B->mn.y || p.y > B->mx.y || p.z < B->mn.z || p.z > B->mx.z) continue;
                float d[6] = {p.x - B->mn.x, B->mx.x - p.x, p.y - B->mn.y, B->mx.y - p.y, p.z - B->mn.z, B->mx.z - p.z};
                int best = 0;
                for (int q = 1; q < 6; q++) if (d[q] < d[best]) best = q;
                V3 n = v3(0, 0, 0);
                if (best == 0) n.x = -1; else if (best == 1) n.x = 1;
                else if (best == 2) n.y = -1; else if (best == 3) n.y = 1;
                else if (best == 4) n.z = -1; else n.z = 1;
                float vn = vdot(N->v, n);
                if (B->bld >= 0 && vn < -BLD_HIT_SPEED) {    // a hard hit on a building: damage, and material for a breach
                    float J = N->m * -vn;
                    TREE_LOCK();
                    Building *bd = &w->bld[B->bld];
                    bd->damage += J; bd->hitImp += J; bd->hitQuiet = 0;
                    bd->hitP = vadd(bd->hitP, vscale(p, J)); bd->hitN = vadd(bd->hitN, vscale(n, J));
                    TREE_UNLOCK();
                }
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
                TREE_LOCK();                                  // trees are shared between cars; collide_static can run on several threads
                if (vn < 0) t->imp += N->m * -vn;
                if (t->breakImp > 0 && t->imp > t->breakImp && !t->broken) break_tree(w, t, vin, n);
                TREE_UNLOCK();
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

// ------------------------------------------------------------ airliners ----
//
// Parked airliners are soft bodies too: a ~40 t node/beam airframe resting on its landing gear. The
// fuselage is a tube of 10-node frames joined by stringers and shear diagonals; wings and stabilisers
// are two-layer boxes; engines hang on pylons. Every beam sits at the integrator's stiffness limit
// for its node masses, so the structure carries its own weight but dents, buckles and tears where a
// car hits it. An airframe sleeps (costs nothing) until a car comes near.

static int af_node(Car *c, V3 p, float m) { int i = add_node(c, p, m); c->n[i].p = p; return i; }
#define AF_K 1.0e9f                                     // every beam clamps to the stability limit
static int af_beam(Car *c, int a, int b, float Y) { return add_beam(c, a, b, AF_K, Y, 0.0f, BEAM_NORMAL); }
static void af_tri(Airframe *A, int a, int b, int d, int paint, V3 ref) {
    int t = A->c.nt;
    add_tri(&A->c, a, b, d, SURF_BODY, -1, 0, ref);
    if (A->c.nt > t) A->paint[t] = (unsigned char)paint;
}
static void af_quad(Airframe *A, int a, int b, int cc, int d, int paint, V3 ref) { af_tri(A, a, b, cc, paint, ref); af_tri(A, a, cc, d, paint, ref); }
static void af_brace(Car *c, const int *ids, int n, float Y) {      // all pairs
    for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) af_beam(c, ids[i], ids[j], Y);
}

// Fuselage shape: station (m from the nose), radius, centre height.
static const float AF_ST[][3] = {{0.0f, 0.15f, 3.35f}, {0.9f, 1.0f, 3.40f}, {2.2f, 1.6f, 3.50f}, {4.0f, 1.95f, 3.55f}, {28.0f, 1.95f, 3.6f},
                                 {31.0f, 1.5f, 3.95f}, {34.0f, 0.8f, 4.5f}, {36.0f, 0.2f, 4.9f}};
static void af_shape(float s, float *r, float *y) {
    int n = (int)(sizeof AF_ST / sizeof AF_ST[0]), i = 0;
    while (i < n - 2 && s > AF_ST[i + 1][0]) i++;
    float t = clampf((s - AF_ST[i][0]) / (AF_ST[i + 1][0] - AF_ST[i][0]), 0, 1);
    *r = AF_ST[i][1] + (AF_ST[i + 1][1] - AF_ST[i][1]) * t;
    *y = AF_ST[i][2] + (AF_ST[i + 1][2] - AF_ST[i][2]) * t;
}

// ---- flight
//
// The airframe flies the way BeamNG's aircraft do: there is no rigid-body flight model, only forces
// on nodes. Each lifting surface (three panels per wing, the centre section, the two stabilisers and
// the fin) is a quad of nodes: every substep it reads its own velocity through the air and its own
// angle of attack, and pushes along its normal, at its quarter-chord. Control inputs change a
// panel's effective angle (ailerons on the outer wing, flaps inboard, elevator, rudder). Engines push
// on their own nodes, so thrust, lift and weight all reach the fuselage through the beams - and bend
// or break them if you overdo it. Wheels roll freely along the fuselage axis and grip sideways.
#define AIR_RHO 1.225f
#define ENGINE_THRUST 80000.0f                  // N per engine, static

static void af_surf(Airframe *A, int n0, int n1, int n2, int n3, int m0, int m1, int m2, int m3, V3 ref, float a0, float slope, int ctl, float gain, float flap) {
    if (A->nsurf >= 12) return;
    int i = A->nsurf++;
    A->surf[i].n[0] = n0; A->surf[i].n[1] = n1; A->surf[i].n[2] = n2; A->surf[i].n[3] = n3;
    A->surf[i].m[0] = m0; A->surf[i].m[1] = m1; A->surf[i].m[2] = m2; A->surf[i].m[3] = m3;
    A->surf[i].a0 = a0; A->surf[i].slope = slope; A->surf[i].ctl = (unsigned char)ctl; A->surf[i].gain = gain; A->surf[i].flap = flap;
    V3 n = vcross(vsub(L[n1], L[n0]), vsub(L[n3], L[n0]));
    A->surf[i].flip = vdot(n, ref) < 0 ? -1 : 1;
    // The panel's nodes are wherever the structure put them, so the quad itself may sit at an angle
    // to the fuselage axis. Measure that and fold it into a0, so a0 is the true rigging angle.
    V3 c1 = vcross(vsub(L[n1], L[n0]), vsub(L[n2], L[n0])), c2 = vcross(vsub(L[n2], L[n0]), vsub(L[n3], L[n0]));
    V3 nn = vscale(vnorm(vadd(c1, c2)), (float)A->surf[i].flip);
    V3 chord = vnorm(vsub(vadd(L[n0], L[n1]), vadd(L[n2], L[n3])));
    V3 fwd = vnorm(vsub(L[A->noseNode], L[A->tailNode]));
    fwd.y = 0; fwd = vnorm(fwd);
    A->surf[i].a0 = a0 - atan2f(-vdot(fwd, nn), vdot(fwd, chord));
}

static V3 af_centre(const Car *c, const int *ids, int n) {
    V3 s = v3(0, 0, 0);
    for (int i = 0; i < n; i++) s = vadd(s, c->n[ids[i]].p);
    return vscale(s, 1.0f / n);
}

void airframe_update_frame(Airframe *A) {
    Car *c = &A->c;
    V3 p = v3(0, 0, 0), v = v3(0, 0, 0); float M = 0;
    for (int i = 0; i < c->nn; i++) { float m = c->n[i].m; p = vadd(p, vscale(c->n[i].p, m)); v = vadd(v, vscale(c->n[i].v, m)); M += m; }
    A->pos = vscale(p, 1.0f / M); A->vel = vscale(v, 1.0f / M);
    c->pos = A->pos; c->vel = A->vel;
    V3 f, r;
    if (A->kind == AC_HELI) {                   // cabin: windscreen frame to back wall, left belt to right belt
        f = vnorm(vsub(af_centre(c, A->ring[1], 6), af_centre(c, A->ring[4], 6)));
        r = vnorm(vsub(vadd(c->n[A->ring[2][1]].p, c->n[A->ring[3][1]].p), vadd(c->n[A->ring[2][4]].p, c->n[A->ring[3][4]].p)));
    } else {
        f = vnorm(vsub(af_centre(c, A->ring[2], AF_RN), af_centre(c, A->ring[12], AF_RN)));
        r = vnorm(vsub(c->n[A->ring[8][0]].p, c->n[A->ring[8][AF_RN / 2]].p));
    }
    A->up = vnorm(vcross(r, f));
    A->right = vnorm(vcross(f, A->up));
    A->fwd = f;
    c->fwd = f; c->up = A->up; c->right = A->right;
    A->airspeed = vlen(A->vel);
    A->aoa = A->airspeed > 3.0f ? atan2f(-vdot(A->vel, A->up), vdot(A->vel, f)) : 0.0f;
    float agl = 1e9f;
    for (int g = 0; g < 3; g++) for (int k = 0; k < 2; k++) {
        V3 q = c->n[A->gearWheel[g][k]].p;
        agl = fminf(agl, q.y - fmaxf(terrain_height(q.x, q.z), g_map == MAP_AIRPORT ? AP_SEA_Y : -1e9f));
    }
    A->agl = agl;
}

static void airframe_aero(Airframe *A, float h) {
    Car *c = &A->c;
    airframe_update_frame(A);
    float V = A->airspeed;
    // Controls: rate-limited, and scaled back with dynamic pressure so full stick stays flyable at speed.
    float auth = clampf((70.0f / fmaxf(V, 35.0f)) * (70.0f / fmaxf(V, 35.0f)), 0.10f, 1.0f);
    if (A->pilot) {
        // Pitch trim: with the stick centred, wind in whatever elevator stops the nose moving.
        float len = vlen(vsub(c->n[A->noseNode].p, c->n[A->tailNode].p));
        float rate = vdot(vsub(c->n[A->noseNode].v, c->n[A->tailNode].v), A->up) / fmaxf(len, 1.0f);
        if (!A->onGround && fabsf(A->pitch) < 0.05f && V > 40.0f) A->trim = clampf(A->trim - rate * 1.6f * h, -0.7f, 0.7f);
        if (A->onGround) A->trim *= 1.0f - 0.5f * h;
    }
    if (A->wrecked) auth = 0;
    float te = clampf(A->pitch * auth + A->trim, -1, 1), ta = clampf(A->roll * auth, -1, 1), tr = clampf(A->yaw * auth + A->roll * auth * 0.25f, -1, 1);
    // Fly-by-wire wing leveller: with the roll keys released, ease the bank back toward level.
    if (A->pilot && !A->onGround && fabsf(A->roll) < 0.05f) ta = clampf(-1.2f * asinf(clampf(-A->right.y, -1, 1)), -0.4f, 0.4f) * fminf(1.0f, auth * 3.0f);
    A->elev += clampf(te - A->elev, -2.5f * h, 2.5f * h);
    A->ail += clampf(ta - A->ail, -3.0f * h, 3.0f * h);
    A->rud += clampf(tr - A->rud, -2.5f * h, 2.5f * h);
    A->stall = 0;
    for (int i = 0; i < A->nsurf; i++) {
        Node *n0 = &c->n[A->surf[i].n[0]], *n1 = &c->n[A->surf[i].n[1]], *n2 = &c->n[A->surf[i].n[2]], *n3 = &c->n[A->surf[i].n[3]];
        V3 v = vscale(vadd(vadd(n0->v, n1->v), vadd(n2->v, n3->v)), 0.25f);
        float sp = vlen(v);
        if (sp < 2.0f) continue;
        V3 c1 = vcross(vsub(n1->p, n0->p), vsub(n2->p, n0->p)), c2 = vcross(vsub(n2->p, n0->p), vsub(n3->p, n0->p));
        float area = 0.5f * (vlen(c1) + vlen(c2));
        V3 n = vnorm(vadd(c1, c2));
        n = vscale(n, (float)A->surf[i].flip);          // the side that was up (or right, for the fin) when built
        V3 chord = vnorm(vsub(vadd(n0->p, n1->p), vadd(n2->p, n3->p)));
        float alpha = atan2f(-vdot(v, n), vdot(v, chord)) + A->surf[i].a0 + A->surf[i].flap * A->flaps;
        float d = A->surf[i].ctl == 1 ? A->ail : A->surf[i].ctl == 2 ? A->elev : A->surf[i].ctl == 3 ? A->rud : 0.0f;
        alpha += A->surf[i].gain * d;
        while (alpha > 3.14159f) alpha -= 6.28318f;
        while (alpha < -3.14159f) alpha += 6.28318f;
        // Attached flow: lift grows with the angle; past ~15 degrees it breaks away and the panel is just a plate.
        float sa = sinf(alpha), st = smooth01(0.24f, 0.40f, fabsf(alpha));
        float cn = lerpf(A->surf[i].slope * sa, (sa >= 0 ? 1.0f : -1.0f) * (0.65f + 0.6f * fabsf(sa)), st);
        if (i < 7 && st > 0.3f && sp > 20.0f) A->stall = 1;
        if (A->wrecked) cn *= 0.2f;                     // torn, buckled panels don't fly
        float F = 0.5f * AIR_RHO * sp * sp * area * cn;
        V3 fl = vscale(n, F * 0.1875f), ft = vscale(n, F * 0.0625f);    // acts at the quarter-chord, shared by both skins
        add_force(n0, fl); add_force(n1, fl); add_force(n2, ft); add_force(n3, ft);
        add_force(&c->n[A->surf[i].m[0]], fl); add_force(&c->n[A->surf[i].m[1]], fl);
        add_force(&c->n[A->surf[i].m[2]], ft); add_force(&c->n[A->surf[i].m[3]], ft);
    }
    // Engines.
    float lapse = clampf(1.0f - V / 300.0f, 0.15f, 1.0f);
    for (int sd = 0; sd < 2; sd++) {
        V3 dir = vnorm(vsub(af_centre(c, A->eng[sd][0], AF_EN), af_centre(c, A->eng[sd][1], AF_EN)));
        V3 f = vscale(dir, A->wrecked ? 0.0f : (0.03f + 0.97f * A->thr) * ENGINE_THRUST * lapse / (2.0f * AF_EN));
        for (int e = 0; e < 2; e++) for (int k = 0; k < AF_EN; k++) add_force(&c->n[A->eng[sd][e][k]], f);
    }
    // Drag: skin friction on everything (more with flaps down), and the fuselage's resistance to going sideways.
    float M = 0;
    for (int i = 0; i < c->nn; i++) M += c->n[i].m;
    float kd = (1.5f + 1.6f * A->flaps + (A->wrecked ? 30.0f : 0.0f)) / M;
    for (int i = 0; i < c->nn; i++) {
        Node *N = &c->n[i];
        float sp = vlen(N->v);
        add_force(N, vscale(N->v, -kd * sp * N->m));
    }
    const float kc = 0.5f * AIR_RHO * 0.9f * 130.0f / (AF_RINGS * AF_RN);
    for (int r = 0; r < AF_RINGS; r++) for (int k = 0; k < AF_RN; k++) {
        Node *N = &c->n[A->ring[r][k]];
        V3 vp = vsub(N->v, vscale(A->fwd, vdot(N->v, A->fwd)));
        add_force(N, vscale(vp, -kc * vlen(vp)));
    }
}

// Wheels: rolling along the fuselage axis (the nose leg steers), gripping sideways, braking on the mains.
static void airframe_wheels(Airframe *A) {
    Car *c = &A->c;
    A->onGround = 0;
    float steer = A->yaw * 0.55f * clampf(1.0f - A->airspeed / 60.0f, 0.12f, 1.0f);
    for (int g = 0; g < 3; g++) for (int k = 0; k < 2; k++) {
        Node *N = &c->n[A->gearWheel[g][k]];
        float gh = terrain_height(N->p.x, N->p.z);
        if (N->p.y >= gh) continue;
        V3 n = terrain_normal(N->p.x, N->p.z);
        N->p.y = gh + 1e-4f;
        A->onGround = 1;
        float vn = vdot(N->v, n);
        if (vn >= 0) continue;
        N->v = vsub(N->v, vscale(n, vn));
        float dv = -vn;
        V3 roll = g == 0 ? vrot(A->fwd, A->up, -steer) : A->fwd;
        roll = vnorm(vsub(roll, vscale(n, vdot(roll, n))));
        V3 lat = vcross(n, roll);
        float vl = vdot(N->v, lat), vr = vdot(N->v, roll);
        float ml = 0.9f * dv, mr = (0.012f + (g ? 0.55f * A->brake : 0.0f)) * dv;
        N->v = vsub(N->v, vscale(lat, clampf(vl, -ml, ml)));
        N->v = vsub(N->v, vscale(roll, clampf(vr, -mr, mr)));
    }
}

// Anything that ends up in the bay floats low and is dragged to a stop.
static void water_node(Node *N, float h) {
    if (N->p.y >= AP_SEA_Y || g_map != MAP_AIRPORT) return;
    if (terrain_height(N->p.x, N->p.z) > AP_SEA_Y - 0.2f) return;
    float k = fminf((AP_SEA_Y - N->p.y) / 0.8f, 1.0f);
    // Drag grows with speed: at flying speed water stops a node almost dead, which is what tears airframes apart.
    N->v = vscale(N->v, 1.0f - fminf((3.5f + 0.30f * vlen(N->v)) * k * h, 0.5f));
    N->v.y += GRAV * 1.5f * k * h;
}

static void airframe_bounds(Airframe *A) {
    Car *c = &A->c;
    V3 mn = c->n[0].p, mx = c->n[0].p;
    for (int i = 1; i < c->nn; i++) {
        V3 q = c->n[i].p;
        mn = v3(fminf(mn.x, q.x), fminf(mn.y, q.y), fminf(mn.z, q.z));
        mx = v3(fmaxf(mx.x, q.x), fmaxf(mx.y, q.y), fmaxf(mx.z, q.z));
    }
    c->bmin = mn; c->bmax = mx;
}

static const float AF_RING_S[AF_RINGS] = {0.9f, 2.2f, 4.0f, 6.2f, 8.4f, 10.6f, 12.8f, 15.0f, 17.2f, 19.4f, 21.6f, 23.8f, 26.0f, 28.0f, 30.0f, 32.0f, 34.0f};

static void airframe_build(Airframe *A, V3 nose, V3 tail, unsigned seed) {
    Car *c = &A->c;
    memset(A, 0, sizeof *A);
    memset(ADJ, 0, sizeof ADJ);
    c->active = 1; c->grip = 1.0f;
    A->seed = seed;
    V3 up = v3(0, 1, 0), right = v3(tail.z, 0, -tail.x);
    // Heights below are a 36 m narrow-body's; the whole airframe sits AF_DROP lower on short gear.
    #define AF_DROP 0.35f
    #define AFP(s, y, l) vadd(vadd(nose, vscale(tail, (s))), vadd(vscale(up, (y) - AF_DROP), vscale(right, (l))))
    V3 ctrRef = AFP(18.0f, 3.55f, 0);
    const float Ys = 0.020f, Yr = 0.008f;             // stringers carry the bending; frames and skin dent sooner

    // ---- fuselage: tip, 17 frames of AF_RN nodes, tail cone tip
    int tipN = af_node(c, AFP(0.0f, 3.35f, 0), 120.0f);
    for (int i = 0; i < AF_RINGS; i++) {
        float r, y; af_shape(AF_RING_S[i], &r, &y);
        for (int k = 0; k < AF_RN; k++) {
            float a = k * (6.2831853f / AF_RN);            // k = 0 on the right side, counter-clockwise seen from the nose
            A->ring[i][k] = af_node(c, AFP(AF_RING_S[i], y + r * sinf(a), r * cosf(a)), i < 2 ? 70.0f : 115.0f);
        }
    }
    int tipT = af_node(c, AFP(36.0f, 4.9f, 0), 110.0f);
    A->noseNode = tipN; A->tailNode = tipT;
    for (int i = 0; i < AF_RINGS; i++)
        for (int k = 0; k < AF_RN; k++) {
            int a = A->ring[i][k], b = A->ring[i][(k + 1) % AF_RN];
            float Yf = (i >= 6 && i <= 10) ? Ys : Yr;                   // heavier frames where the wing box goes through
            af_beam(c, a, b, Yf);                                       // frame
            af_beam(c, a, A->ring[i][(k + 3) % AF_RN], Yf);             // frame bracing (keeps the section round)
            if (k < AF_RN / 2) af_beam(c, a, A->ring[i][k + AF_RN / 2], Yf);
            if (i + 1 < AF_RINGS) {
                af_beam(c, a, A->ring[i + 1][k], Ys);                   // stringer
                af_beam(c, a, A->ring[i + 1][(k + 1) % AF_RN], Yr);     // skin shear
                af_beam(c, b, A->ring[i + 1][k], Yr);
            }
            if (i + 2 < AF_RINGS) af_beam(c, a, A->ring[i + 2][k], Ys); // keel / crown stiffening
        }
    for (int k = 0; k < AF_RN; k++) {
        af_beam(c, tipN, A->ring[0][k], Yr); af_beam(c, tipN, A->ring[1][k], Ys);
        af_beam(c, tipT, A->ring[AF_RINGS - 1][k], Yr); af_beam(c, tipT, A->ring[AF_RINGS - 2][k], Ys);
    }
    // Skin. Paint: 0 white, 1 belly, 2 livery, 3 glass, 4 wing, 5 nacelle, 6 dark.
    for (int k = 0; k < AF_RN; k++) {
        int k1 = (k + 1) % AF_RN;
        int belly = k >= 6 && k <= 8;
        af_tri(A, tipN, A->ring[0][k], A->ring[0][k1], belly ? 1 : 0, ctrRef);
        for (int i = 0; i + 1 < AF_RINGS; i++) {
            int paint = belly ? 1 : 0;
            if (i == 0 && (k <= 1 || k == 3 || k == 4)) paint = 3;     // cockpit windows
            af_quad(A, A->ring[i][k], A->ring[i][k1], A->ring[i + 1][k1], A->ring[i + 1][k], paint, ctrRef);
        }
        af_tri(A, tipT, A->ring[AF_RINGS - 1][k], A->ring[AF_RINGS - 1][k1], 0, ctrRef);
    }

    A->nFusTris = c->nt;

    // ---- wings: 4 spanwise stations x (leading/trailing edge) x (top/bottom)
    const float span[4] = {1.9f, 6.5f, 11.5f, 17.0f};
    for (int side = 0; side < 2; side++) {
        float sg = side ? -1.0f : 1.0f;                                // side 0: right (+l)
        for (int j = 0; j < 4; j++) {
            float u = (span[j] - 1.9f) / 15.1f;
            float sLE = 14.0f + 8.5f * u, sTE = 21.0f + 4.0f * u, y = 2.4f + 0.9f * u, th = 0.6f - 0.3f * u;
            float m = j == 0 ? 300.0f : j == 3 ? 120.0f : 240.0f;
            A->wing[side][j][0] = af_node(c, AFP(sLE, y + th * 0.5f, sg * span[j]), m);
            A->wing[side][j][1] = af_node(c, AFP(sLE, y - th * 0.5f, sg * span[j]), m);
            A->wing[side][j][2] = af_node(c, AFP(sTE, y + th * 0.3f, sg * span[j]), m * 0.8f);
            A->wing[side][j][3] = af_node(c, AFP(sTE, y - th * 0.3f, sg * span[j]), m * 0.8f);
            af_brace(c, A->wing[side][j], 4, 0.06f);
            if (j) {
                for (int p = 0; p < 4; p++) for (int q = 0; q < 4; q++) af_beam(c, A->wing[side][j - 1][p], A->wing[side][j][q], p == q ? Ys : Yr);
            }
        }
        // Root: tie into the nearest fuselage nodes (wing box through the belly).
        for (int p = 0; p < 4; p++) {
            V3 q = c->n[A->wing[side][0][p]].p;
            int best[4] = {-1, -1, -1, -1}; float bd[4] = {1e9f, 1e9f, 1e9f, 1e9f};
            for (int i = 0; i < AF_RINGS; i++) for (int k = 0; k < AF_RN; k++) {
                float d = vlen(vsub(c->n[A->ring[i][k]].p, q));
                for (int s = 0; s < 4; s++) if (d < bd[s]) { for (int t = 3; t > s; t--) { bd[t] = bd[t - 1]; best[t] = best[t - 1]; } bd[s] = d; best[s] = A->ring[i][k]; break; }
            }
            for (int s = 0; s < 4; s++) af_beam(c, A->wing[side][0][p], best[s], Ys);
        }
        // The two wings meet through the centre box.
        if (side) for (int p = 0; p < 4; p++) af_beam(c, A->wing[0][0][p], A->wing[1][0][p], Ys);
        // Hidden stays from mid-span to the fuselage crown stand in for the wing's real depth.
        for (int j = 1; j <= 2; j++) {
            af_beam(c, A->wing[side][j][0], A->ring[7][2], Ys); af_beam(c, A->wing[side][j][2], A->ring[9][2], Ys);
            af_beam(c, A->wing[side][j][1], A->ring[8][side ? 7 : 8], Ys);
        }
        af_beam(c, A->wing[side][3][0], A->ring[8][2], Ys);
        // Skin: top, bottom, leading and trailing edges, tip.
        for (int j = 0; j + 1 < 4; j++) {
            int (*W)[4] = A->wing[side];
            V3 below = vsub(c->n[W[j][1]].p, v3(0, 5, 0)), above = vadd(c->n[W[j][0]].p, v3(0, 5, 0));
            V3 behind = vadd(c->n[W[j][1]].p, vscale(tail, 8)), ahead = vsub(c->n[W[j][1]].p, vscale(tail, 8));
            af_quad(A, W[j][0], W[j + 1][0], W[j + 1][2], W[j][2], 4, below);
            af_quad(A, W[j][1], W[j + 1][1], W[j + 1][3], W[j][3], 1, above);
            af_quad(A, W[j][0], W[j + 1][0], W[j + 1][1], W[j][1], 4, behind);
            af_quad(A, W[j][2], W[j + 1][2], W[j + 1][3], W[j][3], 1, ahead);
        }
        af_quad(A, A->wing[side][3][0], A->wing[side][3][2], A->wing[side][3][3], A->wing[side][3][1], 4, ctrRef);
        // Lifting panels: flaps on the inner two, aileron on the outer one (+roll = right wing down).
        for (int j = 0; j + 1 < 4; j++)
            af_surf(A, A->wing[side][j][0], A->wing[side][j + 1][0], A->wing[side][j + 1][2], A->wing[side][j][2],
                    A->wing[side][j][1], A->wing[side][j + 1][1], A->wing[side][j + 1][3], A->wing[side][j][3], up, 0.052f, 4.4f,
                    j == 2, j == 2 ? (side ? 0.20f : -0.20f) : 0.0f, j < 2 ? 0.17f : 0.0f);
        if (side) af_surf(A, A->wing[0][0][0], A->wing[1][0][0], A->wing[1][0][2], A->wing[0][0][2],
                          A->wing[0][0][1], A->wing[1][0][1], A->wing[1][0][3], A->wing[0][0][3], up, 0.052f, 4.4f, 0, 0, 0.17f);   // centre section

        // ---- engine: front ring (intake) and rear ring, on a pylon under station 1
        for (int e = 0; e < 2; e++)
            for (int k = 0; k < AF_EN; k++) {
                float a = k * (6.2831853f / AF_EN) + 0.5236f, r = e ? 0.8f : 1.0f;
                A->eng[side][e][k] = af_node(c, AFP(e ? 17.0f : 12.5f, 1.95f + r * sinf(a), sg * 6.5f + r * cosf(a)), 200.0f);
            }
        int all[2 * AF_EN];
        for (int e = 0; e < 2; e++) for (int k = 0; k < AF_EN; k++) all[e * AF_EN + k] = A->eng[side][e][k];
        af_brace(c, all, 2 * AF_EN, Ys);
        // Pylon: the top nodes of each ring (k = 1 is the top at 90 degrees) into the wing's lower surface.
        for (int e = 0; e < 2; e++) {
            int t = A->eng[side][e][1], tl = A->eng[side][e][0], tr = A->eng[side][e][2];
            const float Yp = 0.035f;                                   // pylons carry the thrust
            af_beam(c, t, A->wing[side][1][1], Yp); af_beam(c, t, A->wing[side][1][3], Yp);
            af_beam(c, tl, A->wing[side][1][1], Yp); af_beam(c, tr, A->wing[side][1][1], Yp);
            af_beam(c, tl, A->wing[side][1][3], Yp); af_beam(c, tr, A->wing[side][1][3], Yp);
            af_beam(c, t, A->wing[side][0][1], Yp); af_beam(c, t, A->wing[side][2][1], Yp);
        }
        V3 ec = AFP(14.7f, 1.95f, sg * 6.5f);
        for (int k = 0; k < AF_EN; k++) {
            int k1 = (k + 1) % AF_EN;
            af_quad(A, A->eng[side][0][k], A->eng[side][0][k1], A->eng[side][1][k1], A->eng[side][1][k], 5, ec);
        }
        for (int k = 1; k + 1 < AF_EN; k++) {
            af_tri(A, A->eng[side][0][0], A->eng[side][0][k], A->eng[side][0][k + 1], 6, ec);
            af_tri(A, A->eng[side][1][0], A->eng[side][1][k], A->eng[side][1][k + 1], 6, ec);
        }

        // ---- horizontal stabiliser
        int hs[4];
        hs[0] = af_node(c, AFP(33.6f, 4.95f, sg * 6.5f), 40.0f); hs[1] = af_node(c, AFP(33.6f, 4.75f, sg * 6.5f), 40.0f);
        hs[2] = af_node(c, AFP(35.4f, 4.9f, sg * 6.5f), 35.0f); hs[3] = af_node(c, AFP(35.4f, 4.8f, sg * 6.5f), 35.0f);
        af_brace(c, hs, 4, 0.06f);
        int rk = side ? 5 : 0;                                         // the fuselage's side nodes at the tail
        int root[4] = {A->ring[14][rk], A->ring[15][rk], A->ring[16][rk], A->ring[15][side ? 6 : 9]};
        for (int p = 0; p < 4; p++) for (int q = 0; q < 4; q++) af_beam(c, hs[p], root[q], Ys);
        af_beam(c, hs[0], A->ring[15][side ? 4 : 1], Ys);
        af_quad(A, A->ring[14][rk], hs[0], hs[2], A->ring[16][rk], 4, vsub(c->n[hs[0]].p, v3(0, 3, 0)));
        af_quad(A, A->ring[14][rk], hs[1], hs[3], A->ring[16][rk], 1, vadd(c->n[hs[0]].p, v3(0, 3, 0)));
        af_surf(A, A->ring[14][rk], hs[0], hs[2], A->ring[16][rk], A->ring[14][rk], hs[1], hs[3], A->ring[16][rk], up, -0.035f, 3.6f, 2, -0.32f, 0);    // elevator (+pitch = nose up)
    }

    // ---- fin
    int fn[4];
    fn[0] = af_node(c, AFP(33.0f, 11.0f, 0.15f), 30.0f); fn[1] = af_node(c, AFP(33.0f, 11.0f, -0.15f), 30.0f);
    fn[2] = af_node(c, AFP(36.0f, 11.0f, 0.1f), 25.0f);  fn[3] = af_node(c, AFP(36.0f, 11.0f, -0.1f), 25.0f);
    af_brace(c, fn, 4, 0.06f);
    int top[4] = {A->ring[12][2], A->ring[13][2], A->ring[15][2], tipT};
    int topL[4] = {A->ring[12][3], A->ring[13][3], A->ring[15][3], A->ring[16][2]};
    for (int p = 0; p < 4; p++) for (int q = 0; q < 4; q++) { af_beam(c, fn[p], top[q], Ys); af_beam(c, fn[p], topL[q], Ys); }
    af_quad(A, A->ring[12][2], tipT, fn[2], fn[0], 2, AFP(32, 8, -5));
    af_quad(A, A->ring[12][3], tipT, fn[3], fn[1], 2, AFP(32, 8, 5));
    af_tri(A, fn[0], fn[1], A->ring[12][2], 2, AFP(40, 8, 0));
    af_surf(A, A->ring[12][2], fn[0], fn[2], tipT, A->ring[12][3], fn[1], fn[3], tipT, right, 0.0f, 3.0f, 3, -0.35f, 0);                    // rudder (+yaw = nose right)

    // ---- landing gear: struts from the structure down to wheel nodes on the ground
    const float Yg = 0.05f;
    int ng[2] = {af_node(c, AFP(3.0f, AF_DROP, 0.35f), 120.0f), af_node(c, AFP(3.0f, AF_DROP, -0.35f), 120.0f)};
    af_beam(c, ng[0], ng[1], Yg);
    for (int g = 0; g < 2; g++) for (int k = 6; k <= 8; k++) { af_beam(c, ng[g], A->ring[1][k], Yg); af_beam(c, ng[g], A->ring[2][k], Yg); }
    A->gear[0][0] = ng[0]; A->gear[0][1] = A->ring[2][7];
    for (int side = 0; side < 2; side++) {
        float sg = side ? -1.0f : 1.0f;
        int mg[2] = {af_node(c, AFP(18.0f, AF_DROP, sg * 2.8f), 200.0f), af_node(c, AFP(19.2f, AF_DROP, sg * 2.8f), 200.0f)};
        af_beam(c, mg[0], mg[1], Yg);
        int up4[4] = {A->wing[side][0][1], A->wing[side][0][3], A->wing[side][1][1], A->ring[8][side ? 6 : 8]};
        for (int g = 0; g < 2; g++) for (int q = 0; q < 4; q++) af_beam(c, mg[g], up4[q], Yg);
        A->gear[1 + side][0] = mg[0]; A->gear[1 + side][1] = A->wing[side][0][1];
    }
    for (int g = 0; g < 3; g++) A->gearWheel[g][0] = A->gear[g][0];
    A->gearWheel[0][1] = ng[1];
    A->gearWheel[1][1] = A->gear[1][0] + 1; A->gearWheel[2][1] = A->gear[2][0] + 1;
    c->nBody = c->nn;
    #undef AFP
    // Nodes where many beams meet would exceed the explicit integrator's limits (sum of k*h^2/m and
    // c*h/m); scale back the beams there.
    const float h = 1.0f / SIM_HZ;
    for (int pass = 0; pass < 3; pass++) {
        static float ks[MAX_NODES], cs[MAX_NODES];
        memset(ks, 0, sizeof ks); memset(cs, 0, sizeof cs);
        for (int i = 0; i < c->nb; i++) { Beam *B = &c->b[i]; ks[B->a] += B->k; ks[B->b] += B->k; cs[B->a] += B->c; cs[B->b] += B->c; }
        for (int i = 0; i < c->nb; i++) {
            Beam *B = &c->b[i];
            float f = 1.0f;
            for (int e = 0; e < 2; e++) {
                int n = e ? B->b : B->a;
                float m = c->n[n].m;
                f = fminf(f, fminf(0.9f * m / (ks[n] * h * h), 0.75f * m / (cs[n] * h)));
            }
            if (f < 1.0f) { B->k *= f; B->c *= f; }
        }
    }
    for (int i = 0; i < c->nn; i++) c->rest[i] = L[i];
    airframe_bounds(A);
    airframe_update_frame(A);
}

// Let a freshly built airframe settle onto its gear, then put it to sleep with that as its
// undamaged shape.
static void airframe_forces(Airframe *A) {
    Car *c = &A->c;
    for (int i = 0; i < c->nn; i++) c->n[i].f = v3(0, -GRAV * c->n[i].m, 0);
    beam_forces(c);
}
static void airframe_integrate(World *w, Airframe *A, float h) {
    Car *c = &A->c;
    float vmax = 0;
    for (int i = 0; i < c->nn; i++) {
        Node *N = &c->n[i];
        N->v = vadd(N->v, vscale(N->f, h / N->m));
        N->v = vadd(A->vel, vscale(vsub(N->v, A->vel), 0.9999f));     // structural damping (relative to the airframe's own motion)
        N->p = vadd(N->p, vscale(N->v, h));
        float sp = fabsf(N->v.x) + fabsf(N->v.y) + fabsf(N->v.z);
        if (sp > 400.0f) N->v = vscale(N->v, 400.0f / sp);            // wreckage can be violent; keep it finite
        vmax = fmaxf(vmax, sp);
        water_node(N, h);
    }
    if (!(vmax < 1e7f)) c->exploded = 1;                              // NaN
    if (A->kind == AC_AIRLINER) airframe_wheels(A);                    // (a helicopter's skids are just nodes on the ground)
    collide_static(w, c);
    airframe_bounds(A);
    A->vmax = vmax;
}
static void airframe_settle(World *w, Airframe *A) {
    const float h = 1.0f / SIM_HZ;
    for (int s = 0; s < (int)(SIM_HZ * 3.0f); s++) { airframe_forces(A); airframe_integrate(w, A, h); }
    for (int i = 0; i < A->c.nn; i++) A->c.n[i].v = v3(0, 0, 0);
    for (int i = 0; i < A->c.nb; i++) A->c.b[i].rest0 = A->c.b[i].rest;   // the settled shape is "undamaged"
    A->asleep = 1;
    airframe_update_frame(A);
}

void world_respawn_plane(World *w) {
    if (w->flyable < 0) return;
    Airframe *A = &w->air[w->flyable];
    const Plane *P = &w->planes[w->flyable];
    int pilot = A->pilot;
    airframe_build(A, v3(P->x, terrain_height(P->x, P->z), P->z), v3(P->dx, 0, P->dz), P->seed);
    airframe_settle(w, A);
    A->pilot = pilot;
}

// ---------------------------------------------------------------- helicopter --
//
// A light single-engine utility helicopter, after the H125 class: ~2.1 t, a 10.7 m three-blade main
// rotor, a cabin of five frames, a tapering tail boom with a fin, stabiliser and tail rotor, and two
// skids on cross tubes. Like the airliner it is only nodes and beams; it flies by forces on nodes:
//  - the main rotor's thrust acts at the rotor head along the mast. It grows with the square of rotor
//    speed and the collective, falls off as the helicopter climbs through its own downwash (which is
//    what makes a fixed collective settle into a steady climb or descent), and gains a little in
//    ground effect and in forward flight (translational lift);
//  - the cyclic and pedals go through a stability-augmentation loop, as on any modern helicopter: the
//    stick commands a pitch and bank angle and the pedals a yaw rate. The loop's output is an angular
//    acceleration, applied to every node as m (alpha x r), which turns the whole airframe without
//    pushing on it. Its authority is the rotor's: nothing at rest, full only with the rotor turning
//    and carrying the weight;
//  - drag on the fuselage and rotor disc, more sideways and vertically than nose-first.
// Blade tips that touch the ground, a wall, a tree or a vehicle break the rotor: the lift is gone. A
// broken tail rotor leaves the main rotor's torque unopposed and the helicopter spins.
#define HELI_R 5.35f                    // main rotor radius
#define HELI_TR_R 0.82f                 // tail rotor radius
#define HELI_TMAX 1.85f                 // rotor thrust at full collective and rpm, in weights

static void heli_mesh(Car *c, const int *ids, int n, float r, float Y) {     // beam every pair closer than r
    for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++)
        if (vlen(vsub(L[ids[i]], L[ids[j]])) < r) af_beam(c, ids[i], ids[j], Y);
}

static void heli_build(Airframe *A, V3 at, V3 tail, unsigned seed) {
    Car *c = &A->c;
    memset(A, 0, sizeof *A);
    memset(ADJ, 0, sizeof ADJ);
    c->active = 1; c->grip = 1.0f;
    A->seed = seed; A->kind = AC_HELI; A->strikeT = -1;
    V3 fw = vscale(vnorm(tail), -1.0f), up = v3(0, 1, 0), rt = vcross(fw, up);   // heli-local x forward, y up, z right
    #define HP(x, y, z) vadd(at, vadd(vadd(vscale(fw, (x)), vscale(up, (y))), vscale(rt, (z))))
    // Cabin frames: x, floor y and half-width, belt y and half-width, roof y and half-width.
    static const float CAB[5][7] = {
        { 1.95f, 0.78f, 0.36f, 1.30f, 0.56f, 1.72f, 0.30f},        // nose (chin bubble)
        { 1.30f, 0.56f, 0.66f, 1.32f, 0.82f, 2.06f, 0.56f},        // windscreen frame
        { 0.20f, 0.50f, 0.76f, 1.34f, 0.86f, 2.16f, 0.70f},        // front doors
        {-1.15f, 0.50f, 0.76f, 1.34f, 0.86f, 2.16f, 0.70f},        // rear doors
        {-2.25f, 0.76f, 0.56f, 1.42f, 0.64f, 2.10f, 0.58f}};       // cabin back wall, under the engine
    int cab[64], nc = 0;
    for (int r = 0; r < 5; r++) {
        const float *F = CAB[r];
        float yk[6] = {F[1], F[3], F[5], F[5], F[3], F[1]}, zk[6] = {F[2], F[4], F[6], -F[6], -F[4], -F[2]};
        for (int k = 0; k < 6; k++) {                           // k: floor R, belt R, roof R, roof L, belt L, floor L
            float m = k == 0 || k == 5 ? 46.0f : 30.0f;            // seats, fuel and floor structure low down
            A->ring[r][k] = af_node(c, HP(F[0], yk[k], zk[k]), r == 0 ? 18.0f : m);
            cab[nc++] = A->ring[r][k];
        }
    }
    int tip = af_node(c, HP(2.45f, 1.12f, 0), 16.0f);
    cab[nc++] = tip;
    A->noseNode = tip;
    // Engine and gearbox deck on the roof (heavy), then the boom.
    int deck[4] = {af_node(c, HP(0.15f, 2.52f, 0.46f), 85.0f), af_node(c, HP(0.15f, 2.52f, -0.46f), 85.0f),
                   af_node(c, HP(-1.85f, 2.46f, 0.44f), 85.0f), af_node(c, HP(-1.85f, 2.46f, -0.44f), 85.0f)};
    for (int i = 0; i < 4; i++) cab[nc++] = deck[i];
    static const float BOOM[4][3] = {{-3.25f, 1.62f, 0.36f}, {-4.95f, 1.76f, 0.26f}, {-6.65f, 1.90f, 0.19f}, {-7.85f, 2.00f, 0.15f}};
    int boom[4][3];
    for (int r = 0; r < 4; r++) {
        float x = BOOM[r][0], y = BOOM[r][1], s = BOOM[r][2];
        boom[r][0] = af_node(c, HP(x, y + s, 0), 13.0f);                  // top
        boom[r][1] = af_node(c, HP(x, y - 0.6f * s, 0.85f * s), 13.0f);   // bottom right
        boom[r][2] = af_node(c, HP(x, y - 0.6f * s, -0.85f * s), 13.0f);  // bottom left
        for (int k = 0; k < 3; k++) A->ring[5 + r][k] = boom[r][k];
    }
    int finTop = af_node(c, HP(-8.55f, 3.25f, 0), 7.0f), finLE = af_node(c, HP(-7.70f, 2.95f, 0), 7.0f);
    int finLow = af_node(c, HP(-8.25f, 1.15f, 0), 7.0f);
    A->tailNode = af_node(c, HP(-8.45f, 2.05f, 0), 9.0f);
    A->trHub = af_node(c, HP(-8.25f, 2.30f, -0.38f), 10.0f);
    int stab[4] = {af_node(c, HP(-6.05f, 1.92f, 1.15f), 6.0f), af_node(c, HP(-6.55f, 1.92f, 1.15f), 6.0f),
                   af_node(c, HP(-6.05f, 1.92f, -1.15f), 6.0f), af_node(c, HP(-6.55f, 1.92f, -1.15f), 6.0f)};
    A->stabTip[0] = stab[1]; A->stabTip[1] = stab[3]; A->finTop = finTop;
    // Skids: upturned tip, front cross-tube foot, mid, rear foot, tail end; [0] right, [1] left.
    for (int sd = 0; sd < 2; sd++) {
        float z = sd ? -1.12f : 1.12f;
        const float sx[5] = {2.05f, 1.05f, 0.0f, -1.10f, -1.95f}, sy[5] = {0.32f, 0.05f, 0.05f, 0.05f, 0.08f};
        for (int i = 0; i < 5; i++) A->skid[sd][i] = af_node(c, HP(sx[i], sy[i], z), 11.0f);
    }
    // Mast and rotor head: placed over the centre of mass, so the rotor lifts it level.
    V3 com = v3(0, 0, 0); float M = 0;
    for (int i = 0; i < c->nn; i++) { com = vadd(com, vscale(L[i], c->n[i].m)); M += c->n[i].m; }
    float cx = vdot(vsub(vscale(com, 1.0f / M), at), fw);
    A->mast[0] = af_node(c, HP(cx, 2.70f, 0), 140.0f);
    A->mast[1] = af_node(c, HP(cx, 3.32f, 0), 55.0f);
    for (int k = 0; k < 4; k++) {
        float a = k * 1.5707963f + 0.7853982f;
        A->hub[k] = af_node(c, HP(cx + 0.42f * cosf(a), 3.30f, 0.42f * sinf(a)), 30.0f);   // hub and blade roots
    }
    // ---- structure
    const float Ys = 0.035f, Yb = 0.032f;
    heli_mesh(c, cab, nc, 2.05f, Ys);                           // cabin shell, nose and deck as one truss
    int head[7] = {A->mast[0], A->mast[1], A->hub[0], A->hub[1], A->hub[2], A->hub[3], deck[0]};
    af_brace(c, head, 6, 0.06f);
    for (int i = 0; i < 4; i++) { af_beam(c, A->mast[0], deck[i], 0.05f); af_beam(c, A->mast[1], deck[i], 0.05f); }
    for (int k = 0; k < 4; k++) for (int i = 0; i < 2; i++) af_beam(c, A->hub[k], deck[k < 2 ? i : 2 + i], 0.05f);
    af_beam(c, A->mast[0], A->ring[2][2], Ys); af_beam(c, A->mast[0], A->ring[2][3], Ys);
    af_beam(c, A->mast[0], A->ring[3][2], Ys); af_beam(c, A->mast[0], A->ring[3][3], Ys);
    // Boom: rings braced, each to the next and the one after, the first into the cabin back and deck.
    for (int r = 0; r < 4; r++) {
        af_brace(c, boom[r], 3, Yb);
        if (r + 1 < 4) for (int p = 0; p < 3; p++) for (int q = 0; q < 3; q++) af_beam(c, boom[r][p], boom[r + 1][q], Yb);
        if (r + 2 < 4) for (int p = 0; p < 3; p++) af_beam(c, boom[r][p], boom[r + 2][p], Yb);
    }
    for (int p = 0; p < 3; p++) {
        for (int k = 0; k < 6; k++) af_beam(c, boom[0][p], A->ring[4][k], Ys);
        af_beam(c, boom[0][p], deck[2], Ys); af_beam(c, boom[0][p], deck[3], Ys);
        af_beam(c, boom[1][p], A->ring[4][2], Yb); af_beam(c, boom[1][p], A->ring[4][3], Yb);
    }
    int tailG[7] = {finTop, finLE, finLow, A->tailNode, A->trHub, boom[3][0], boom[3][1]};
    af_brace(c, tailG, 7, Yb);
    for (int i = 0; i < 5; i++) { af_beam(c, tailG[i], boom[3][2], Yb); af_beam(c, tailG[i], boom[2][0], Yb); }
    for (int s = 0; s < 4; s++) { af_beam(c, stab[s], boom[2][0], Yb); af_beam(c, stab[s], boom[2][1], Yb); af_beam(c, stab[s], boom[2][2], Yb); af_beam(c, stab[s], boom[1][0], Yb); }
    af_beam(c, stab[0], stab[1], Yb); af_beam(c, stab[2], stab[3], Yb); af_beam(c, stab[0], stab[3], Yb); af_beam(c, stab[1], stab[2], Yb);
    // Skids: tubes along, cross tubes up into the cabin floor and across to the other skid. They
    // yield before the cabin does, as real ones are made to.
    const float Yk = 0.045f;
    for (int sd = 0; sd < 2; sd++) {
        int *S = A->skid[sd];
        for (int i = 0; i < 5; i++) { if (i + 1 < 5) af_beam(c, S[i], S[i + 1], Yk); if (i + 2 < 5) af_beam(c, S[i], S[i + 2], Yk); }
        int fl = sd ? 5 : 0, bl = sd ? 4 : 1;                    // floor and belt nodes on this side
        int feet[2] = {S[1], S[3]}, fr[2][2] = {{1, 2}, {3, 4}};
        for (int f = 0; f < 2; f++)
            for (int q = 0; q < 2; q++) {
                af_beam(c, feet[f], A->ring[fr[f][q]][fl], Yk); af_beam(c, feet[f], A->ring[fr[f][q]][sd ? 0 : 5], Yk);
                af_beam(c, feet[f], A->ring[fr[f][q]][bl], Yk);
            }
        af_beam(c, S[0], A->ring[1][fl], Yk); af_beam(c, S[2], A->ring[2][fl], Yk); af_beam(c, S[2], A->ring[3][fl], Yk);
        af_beam(c, S[4], A->ring[3][fl], Yk); af_beam(c, S[4], A->ring[4][fl], Yk);
    }
    for (int i = 0; i < 5; i++) af_beam(c, A->skid[0][i], A->skid[1][i], Yk);
    af_beam(c, A->skid[0][1], A->skid[1][3], Yk); af_beam(c, A->skid[1][1], A->skid[0][3], Yk);
    // ---- skin. Paint: 0 white, 1 belly, 2 livery, 3 glass, 6 dark. The cabin and boom are smooth-shaded.
    V3 inC = HP(-0.3f, 1.35f, 0);
    for (int r = 0; r + 1 < 5; r++)
        for (int k = 0; k < 6; k++) {
            int k1 = (k + 1) % 6;
            int paint = k == 5 ? 1 : (k == 0 || k == 4) ? 2 : 0;         // belly, livery below the belt, white above
            if ((k == 1 || k == 3) && r <= 2) paint = 3;                  // door windows
            if (k == 2 && r == 0) paint = 3;                              // windscreen (roof's leading edge)
            if (k == 2 && r >= 2) paint = 0;
            af_quad(A, A->ring[r][k], A->ring[r][k1], A->ring[r + 1][k1], A->ring[r + 1][k], paint, inC);
        }
    for (int k = 0; k < 6; k++) af_tri(A, tip, A->ring[0][k], A->ring[0][(k + 1) % 6], (k >= 1 && k <= 3) ? 3 : k == 5 ? 1 : 2, inC);
    // Cabin back into the boom root: each back-wall node to the nearest boom-root node.
    static const int BK[6] = {1, 1, 0, 0, 2, 2};
    V3 inB = HP(-2.8f, 1.5f, 0);
    for (int k = 0; k < 6; k++) {
        int k1 = (k + 1) % 6;
        af_tri(A, A->ring[4][k], A->ring[4][k1], boom[0][BK[k1]], k == 5 ? 1 : k == 0 || k == 4 ? 2 : 0, inB);
        if (BK[k] != BK[k1]) af_tri(A, A->ring[4][k], boom[0][BK[k1]], boom[0][BK[k]], 0, inB);
    }
    for (int r = 0; r + 1 < 4; r++)
        for (int k = 0; k < 3; k++) {
            int k1 = (k + 1) % 3;
            V3 ax = HP(0.5f * (BOOM[r][0] + BOOM[r + 1][0]), 0.5f * (BOOM[r][1] + BOOM[r + 1][1]), 0);
            af_quad(A, boom[r][k], boom[r][k1], boom[r + 1][k1], boom[r + 1][k], k == 1 ? 1 : 2 * (r == 0), ax);
        }
    A->nFusTris = c->nt;
    // Engine cowling over the deck, flat-shaded.
    V3 inE = HP(-0.8f, 2.2f, 0);
    af_quad(A, deck[0], deck[1], deck[3], deck[2], 0, inE);
    af_quad(A, deck[0], deck[2], A->ring[4][2], A->ring[2][2], 0, inE); af_quad(A, deck[1], deck[3], A->ring[4][3], A->ring[2][3], 0, inE);
    af_quad(A, deck[0], deck[1], A->ring[2][3], A->ring[2][2], 0, inE); af_quad(A, deck[2], deck[3], A->ring[4][3], A->ring[4][2], 6, inE);
    // Fin, stabiliser and the boom's end cap.
    af_quad(A, boom[3][0], finLE, finTop, A->tailNode, 2, HP(-8.1f, 2.4f, -1));
    af_quad(A, boom[3][0], finLE, finTop, A->tailNode, 2, HP(-8.1f, 2.4f, 1));
    af_tri(A, boom[3][1], finLow, A->tailNode, 2, HP(-8.1f, 1.6f, -1)); af_tri(A, boom[3][2], finLow, A->tailNode, 2, HP(-8.1f, 1.6f, 1));
    af_tri(A, boom[3][0], boom[3][1], A->tailNode, 0, HP(-7.0f, 2.0f, 0)); af_tri(A, boom[3][0], boom[3][2], A->tailNode, 0, HP(-7.0f, 2.0f, 0));
    af_quad(A, stab[0], stab[1], stab[3], stab[2], 2, HP(-6.3f, 0.0f, 0));
    af_quad(A, stab[0], stab[1], stab[3], stab[2], 2, HP(-6.3f, 4.0f, 0));
    for (int g = 0; g < 2; g++) for (int k = 0; k < 2; k++) A->gearWheel[g][k] = A->skid[g][1 + 2 * k];
    A->gearWheel[2][0] = A->skid[0][2]; A->gearWheel[2][1] = A->skid[1][2];
    c->nBody = c->nn;
    #undef HP
    const float h = 1.0f / SIM_HZ;                              // the same stiffness limiter as the airliner
    for (int pass = 0; pass < 3; pass++) {
        static float ks[MAX_NODES], cs[MAX_NODES];
        memset(ks, 0, sizeof ks); memset(cs, 0, sizeof cs);
        for (int i = 0; i < c->nb; i++) { Beam *B = &c->b[i]; ks[B->a] += B->k; ks[B->b] += B->k; cs[B->a] += B->c; cs[B->b] += B->c; }
        for (int i = 0; i < c->nb; i++) {
            Beam *B = &c->b[i];
            float f = 1.0f;
            for (int e = 0; e < 2; e++) { int n = e ? B->b : B->a; float m = c->n[n].m; f = fminf(f, fminf(0.9f * m / (ks[n] * h * h), 0.75f * m / (cs[n] * h))); }
            if (f < 1.0f) { B->k *= f; B->c *= f; }
        }
    }
    for (int i = 0; i < c->nn; i++) c->rest[i] = L[i];
    airframe_bounds(A);
    airframe_update_frame(A);
}

void world_respawn_heli(World *w) {
    if (w->heli < 0) return;
    Airframe *A = &w->air[w->heli];
    const Plane *P = &w->planes[w->heli];
    int pilot = A->pilot;
    heli_build(A, v3(P->x, terrain_height(P->x, P->z), P->z), v3(P->dx, 0, P->dz), P->seed);
    airframe_settle(w, A);
    A->pilot = pilot;
}

// Angular velocity of the airframe as a rigid body: solve I w = sum m r x v about the centre of mass.
static V3 heli_omega(const Airframe *A) {
    const Car *c = &A->c;
    float I[3][3] = {{0}}; V3 Lm = v3(0, 0, 0);
    for (int i = 0; i < c->nn; i++) {
        const Node *N = &c->n[i];
        V3 r = vsub(N->p, A->pos), v = vsub(N->v, A->vel);
        Lm = vadd(Lm, vscale(vcross(r, v), N->m));
        float rr = vdot(r, r), q[3] = {r.x, r.y, r.z};
        for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) I[a][b] += N->m * ((a == b ? rr : 0.0f) - q[a] * q[b]);
    }
    float det = I[0][0] * (I[1][1] * I[2][2] - I[1][2] * I[2][1]) - I[0][1] * (I[1][0] * I[2][2] - I[1][2] * I[2][0]) + I[0][2] * (I[1][0] * I[2][1] - I[1][1] * I[2][0]);
    if (fabsf(det) < 1e-6f) return v3(0, 0, 0);
    float l[3] = {Lm.x, Lm.y, Lm.z}, w[3];
    for (int col = 0; col < 3; col++) {                         // Cramer's rule
        float m[3][3];
        for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) m[a][b] = b == col ? l[a] : I[a][b];
        w[col] = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / det;
    }
    return v3(w[0], w[1], w[2]);
}

static int heli_tip_hits(const World *w, const Airframe *A, V3 p) {
    if (p.y < terrain_height(p.x, p.z) || (g_map == MAP_AIRPORT && p.y < AP_SEA_Y)) return 1;
    if (world_point_solid(w, p)) return 1;
    for (int i = 0; i < w->ncars; i++) {
        const Car *k = &w->cars[i];
        if (k->active && p.x > k->bmin.x && p.x < k->bmax.x && p.y > k->bmin.y && p.y < k->bmax.y && p.z > k->bmin.z && p.z < k->bmax.z) return 1;
    }
    for (int i = 0; i < w->nplanes; i++) {                      // other aircraft: within a metre of one of their nodes
        const Airframe *B = &w->air[i];
        if (B == A) continue;
        V3 lo = B->c.bmin, hi = B->c.bmax;
        if (p.x < lo.x - 1 || p.x > hi.x + 1 || p.y < lo.y - 1 || p.y > hi.y + 1 || p.z < lo.z - 1 || p.z > hi.z + 1) continue;
        for (int n = 0; n < B->c.nn; n++) { V3 d = vsub(B->c.n[n].p, p); if (vdot(d, d) < 1.0f) return 1; }
    }
    return 0;
}

static void heli_aero(World *w, Airframe *A, float h) {
    Car *c = &A->c;
    airframe_update_frame(A);
    float M = 0;
    for (int i = 0; i < c->nn; i++) M += c->n[i].m;
    float Wt = M * GRAV;
    // Skids on the ground?
    A->onGround = 0;
    for (int sd = 0; sd < 2; sd++) for (int i = 1; i < 5; i++) {
        V3 q = c->n[A->skid[sd][i]].p;
        if (q.y < fmaxf(terrain_height(q.x, q.z), g_map == MAP_AIRPORT ? AP_SEA_Y : -1e9f) + 0.12f) A->onGround = 1;   // (or afloat)
    }
    // Engine: spools the rotor up while someone is flying it (or while it is still in the air), down otherwise.
    int engine = !A->wrecked && (A->pilot || !A->onGround);
    float want = engine && !A->rotorBroken ? 1.0f : 0.0f;
    A->rpm += clampf(want - A->rpm, -(A->rotorBroken ? 0.9f : 0.08f) * h, 0.16f * h);
    A->rotorAngle = fmodf(A->rotorAngle + A->rpm * 6.2831853f * 6.6f * h, 6.2831853f);
    if (!A->tailBroken) A->trAngle = fmodf(A->trAngle + A->rpm * 6.2831853f * 33.0f * h, 6.2831853f);
    // Nobody at the controls in the air: slow to a hover and let it down gently at 2 m/s.
    if (!A->pilot) {
        A->pitch = clampf(0.06f * vdot(A->vel, A->fwd), -0.6f, 0.6f);
        A->roll = clampf(-0.06f * vdot(A->vel, A->right), -0.6f, 0.6f);
        A->yaw = 0;
        if (A->onGround) A->pitch = A->roll = 0;
        if (!A->onGround) A->thr = clampf(A->thr + (-2.0f - A->vel.y) * 0.25f * h, 0.0f, 1.0f);
        else A->thr = fmaxf(A->thr - 0.5f * h, 0.0f);
    }
    // ---- main rotor thrust
    float rpm2 = A->rpm * A->rpm;
    V3 u = A->up;
    float vax = A->vel.y;                                       // climbing through the downwash
    float spd = sqrtf(A->vel.x * A->vel.x + A->vel.z * A->vel.z);
    float T = HELI_TMAX * Wt * rpm2 * (0.04f + 0.96f * A->thr);
    T *= clampf(1.0f - vax / 15.0f, 0.4f, 1.6f);
    float hr = fmaxf(A->agl, 0.0f) / (2.0f * HELI_R);
    if (hr < 1.0f) T *= 1.0f + 0.14f * (1.0f - hr) * (1.0f - hr);    // ground effect
    T *= 1.0f + 0.12f * smooth01(4.0f, 16.0f, spd);               // translational lift
    if (A->rotorBroken || A->wrecked) T = 0;
    A->lift = T / Wt;
    // Blade-tip strikes: sample the tip circle (it sweeps the whole disc between samples at any rpm).
    if (A->rpm > 0.25f && !A->rotorBroken) {
        V3 hubP = c->n[A->mast[1]].p, fw = A->fwd, rt = A->right;
        for (int s = 0; s < 12 && !A->rotorBroken; s++) {
            float a = s * 0.5235988f, rr = s & 1 ? HELI_R : 0.72f * HELI_R;
            V3 p = vadd(hubP, vadd(vscale(fw, rr * cosf(a)), vscale(rt, rr * sinf(a))));
            p = vsub(p, vscale(u, 0.12f * (1.0f - A->lift)));     // the blades droop when they're not carrying the weight
            if (heli_tip_hits(w, A, p)) { A->rotorBroken = 1; A->strikeT = 0; }
        }
    }
    if (A->rpm > 0.25f && !A->tailBroken) {
        V3 trP = c->n[A->trHub].p;
        for (int s = 0; s < 6 && !A->tailBroken; s++) {
            float a = s * 1.0471976f;
            V3 p = vadd(trP, vadd(vscale(A->fwd, HELI_TR_R * cosf(a)), vscale(u, HELI_TR_R * sinf(a))));
            if (heli_tip_hits(w, A, p)) { A->tailBroken = 1; if (A->strikeT < 0) A->strikeT = 0; }
        }
    }
    if (A->strikeT >= 0) A->strikeT += h;
    if (T > 0) {
        V3 f = vscale(u, T / 5.0f);
        add_force(&c->n[A->mast[1]], f);
        for (int k = 0; k < 4; k++) add_force(&c->n[A->hub[k]], f);
    }
    // ---- stability augmentation: stick -> attitude, pedals -> yaw rate, as angular acceleration.
    float auth = rpm2 * (A->onGround ? smooth01(0.35f, 0.95f, A->lift) : 1.0f);
    if (A->rotorBroken || A->wrecked) auth = 0;
    V3 om = heli_omega(A);
    float pitchA = asinf(clampf(A->fwd.y, -1, 1)), bankA = asinf(clampf(-A->right.y, -1, 1));
    float qr = vdot(om, A->right), pr = vdot(om, A->fwd), yr = -vdot(om, u);        // nose-up, right-down, nose-right rates
    // Trim: with the stick centred in the air, slowly wind out whatever pitch and bank it has settled at
    // (the centre of mass is never exactly under the mast once it's loaded and bent).
    if (!A->onGround && auth > 0.5f) {
        if (fabsf(A->pitch) < 0.05f) A->trim = clampf(A->trim - pitchA * 0.4f * h, -0.08f, 0.08f);
        if (fabsf(A->roll) < 0.05f) A->ail = clampf(A->ail - bankA * 0.4f * h, -0.08f, 0.08f);
    }
    float pitchT = A->pitch * 0.36f + A->trim, bankT = A->roll * 0.48f + A->ail;
    float yawT = A->yaw * 1.0f;
    if (spd > 12.0f && !A->onGround) yawT += smooth01(12.0f, 25.0f, spd) * GRAV * tanf(bankA) / spd;   // coordinated turn: the fin weathervanes it
    float aq = 9.0f * (pitchT - pitchA) - 5.0f * qr, ap = 9.0f * (bankT - bankA) - 5.0f * pr, ay = A->tailBroken ? 0.0f : 5.0f * (yawT - yr);
    V3 alpha = vscale(vadd(vadd(vscale(A->right, aq), vscale(A->fwd, ap)), vscale(u, -ay)), auth);
    // A tail rotor that's gone: the main rotor's torque turns the whole helicopter the other way
    // (nose left; the rotor turns clockwise seen from above), against a little drag from the boom.
    if (A->tailBroken && !A->rotorBroken) alpha = vadd(alpha, vscale(u, 2.6f * rpm2 * (0.3f + A->thr) - 0.6f * vdot(om, u)));
    // ---- drag: nose-first, sideways and through the disc
    float vf = vdot(A->vel, A->fwd), vs = vdot(A->vel, A->right);
    V3 D = vadd(vadd(vscale(A->fwd, -2.6f * vf * fabsf(vf)), vscale(A->right, -9.0f * vs * fabsf(vs))), v3(0, -(4.0f + 30.0f * A->rpm) * vax * fabsf(vax), 0));
    D = vscale(D, 0.5f * AIR_RHO / M);
    for (int i = 0; i < c->nn; i++) {
        Node *N = &c->n[i];
        V3 r = vsub(N->p, A->pos);
        add_force(N, vscale(vadd(vcross(alpha, r), D), N->m));
    }
}

// A car node against the airframe's skin: one-sided contact with each triangle, so a node can't slip
// between two airframe nodes the way it could with node-to-node contact alone.
static void collide_shell(Car *C, Airframe *A) {
    Car *P = &A->c;
    float D = NODE_R + 0.05f;
    if (C->bmax.x + D < P->bmin.x || P->bmax.x + D < C->bmin.x || C->bmax.y + D < P->bmin.y || P->bmax.y + D < C->bmin.y ||
        C->bmax.z + D < P->bmin.z || P->bmax.z + D < C->bmin.z) return;
    static int cand[MAX_TRIS];
    int nc = 0;
    for (int t = 0; t < P->nt; t++) {
        const Tri *T = &P->t[t];
        V3 a = P->n[T->v[0]].p, b = P->n[T->v[1]].p, d = P->n[T->v[2]].p;
        if (fmaxf(a.x, fmaxf(b.x, d.x)) + D < C->bmin.x || fminf(a.x, fminf(b.x, d.x)) - D > C->bmax.x) continue;
        if (fmaxf(a.y, fmaxf(b.y, d.y)) + D < C->bmin.y || fminf(a.y, fminf(b.y, d.y)) - D > C->bmax.y) continue;
        if (fmaxf(a.z, fmaxf(b.z, d.z)) + D < C->bmin.z || fminf(a.z, fminf(b.z, d.z)) - D > C->bmax.z) continue;
        cand[nc++] = t;
    }
    if (!nc) return;
    for (int i = 0; i < C->nn; i++) {
        Node *N = &C->n[i];
        if (N->p.x < P->bmin.x - D || N->p.x > P->bmax.x + D || N->p.y < P->bmin.y - D || N->p.y > P->bmax.y + D ||
            N->p.z < P->bmin.z - D || N->p.z > P->bmax.z + D) continue;
        for (int q = 0; q < nc; q++) {
            const Tri *T = &P->t[cand[q]];
            Node *na = &P->n[T->v[0]], *nb = &P->n[T->v[1]], *nd = &P->n[T->v[2]];
            V3 e1 = vsub(nb->p, na->p), e2 = vsub(nd->p, na->p), nrm = vcross(e1, e2);
            float area2 = vlen(nrm);
            if (area2 < 1e-6f) continue;
            nrm = vscale(nrm, 1.0f / area2);
            V3 ap = vsub(N->p, na->p);
            float dist = vdot(ap, nrm);
            if (dist > NODE_R || dist < -0.25f) continue;
            // Barycentric coordinates of the projection.
            float d11 = vdot(e1, e1), d12 = vdot(e1, e2), d22 = vdot(e2, e2), dp1 = vdot(ap, e1), dp2 = vdot(ap, e2);
            float den = d11 * d22 - d12 * d12;
            if (fabsf(den) < 1e-9f) continue;
            float wb = (d22 * dp1 - d12 * dp2) / den, wd = (d11 * dp2 - d12 * dp1) / den, wa = 1.0f - wb - wd;
            if (wa < -0.02f || wb < -0.02f || wd < -0.02f) continue;
            V3 vs = vadd(vadd(vscale(na->v, wa), vscale(nb->v, wb)), vscale(nd->v, wd));
            V3 rv = vsub(N->v, vs);
            float vrel = vdot(rv, nrm);
            float F = KC * (NODE_R - dist) - CC * vrel;
            if (F <= 0) continue;
            V3 f = vscale(nrm, F);
            V3 vt = vsub(rv, vscale(nrm, vrel));
            float vtl = vlen(vt);
            if (vtl > 1e-4f) f = vsub(f, vscale(vt, 0.35f * F * fminf(1.0f, vtl / 0.5f) / vtl));
            add_force(N, f);
            add_force(na, vscale(f, -wa)); add_force(nb, vscale(f, -wb)); add_force(nd, vscale(f, -wd));
        }
    }
}

// Per substep: wake airframes that a car is near, simulate the awake ones, put quiet ones back to sleep.
static void airframes_substep(World *w, float h, int phase) {
    for (int ai = 0; ai < w->nplanes; ai++) {
        Airframe *A = &w->air[ai];
        if (phase == 0) {
            int near = 0;
            for (int ci = 0; ci < w->ncars && !near; ci++) {
                const Car *c = &w->cars[ci];
                if (!c->active) continue;
                const float m = 1.0f;
                near = !(c->bmax.x + m < A->c.bmin.x || A->c.bmax.x + m < c->bmin.x || c->bmax.y + m < A->c.bmin.y ||
                         A->c.bmax.y + m < c->bmin.y || c->bmax.z + m < A->c.bmin.z || A->c.bmax.z + m < c->bmin.z);
            }
            for (int bi = 0; bi < w->nplanes && !near; bi++) {     // another aircraft (awake, so moving) touching it
                const Airframe *B = &w->air[bi];
                if (bi == ai || B->asleep) continue;
                near = !(B->c.bmax.x + 1 < A->c.bmin.x || A->c.bmax.x + 1 < B->c.bmin.x || B->c.bmax.y + 1 < A->c.bmin.y ||
                         A->c.bmax.y + 1 < B->c.bmin.y || B->c.bmax.z + 1 < A->c.bmin.z || A->c.bmax.z + 1 < B->c.bmin.z);
            }
            if (near || A->pilot || A->rpm > 0.01f) { A->asleep = 0; A->quiet = 0; }
            if (A->asleep) continue;
            airframe_forces(A);
            if (A->kind == AC_HELI) heli_aero(w, A, h); else airframe_aero(A, h);
            for (int ci = 0; ci < w->ncars; ci++) {
                Car *c = &w->cars[ci];
                if (!c->active) continue;
                collide_cars(c, &A->c);
                collide_shell(c, A);
            }
            for (int bi = 0; bi < ai; bi++) {                       // aircraft against each other (once per pair)
                Airframe *B = &w->air[bi];
                if (B->asleep) continue;
                collide_cars(&A->c, &B->c);
                if (A->kind == AC_HELI) collide_shell(&A->c, B); else if (B->kind == AC_HELI) collide_shell(&B->c, A);
            }
        } else if (!A->asleep) {
            airframe_integrate(w, A, h);
            // Fracture: thin aluminium doesn't fold far before it lets go. A beam crushed or stretched
            // permanently past these limits snaps, so a hard crash breaks the airframe into pieces
            // rather than bouncing it or folding it like a concertina.
            static unsigned tick;
            if ((tick++ & 7) == 0) {
                int broken = 0, bent = 0;
                for (int i = 0; i < A->c.nb; i++) {
                    Beam *B = &A->c.b[i];
                    if (!B->broken && (B->rest < 0.80f * B->rest0 || B->rest > 1.12f * B->rest0)) B->broken = 1;
                    broken += B->broken;
                    bent += fabsf(B->rest - B->rest0) > 0.03f * B->rest0;
                }
                A->nbroken = broken;
                if (A->kind == AC_HELI ? (broken > 14 || bent > 90) : (broken > 30 || bent > 160)) A->wrecked = 1;
            }
            int touching = 0;
            for (int ci = 0; ci < w->ncars && !touching; ci++) {
                const Car *c = &w->cars[ci];
                touching = c->active && !(c->bmax.x + 1 < A->c.bmin.x || A->c.bmax.x + 1 < c->bmin.x || c->bmax.z + 1 < A->c.bmin.z || A->c.bmax.z + 1 < c->bmin.z || c->bmax.y + 1 < A->c.bmin.y || A->c.bmax.y + 1 < c->bmin.y);
            }
            A->quiet = (!touching && A->vmax < 0.08f && A->rpm < 0.01f && !A->pilot) ? A->quiet + h : 0.0f;
            if (A->quiet > 2.0f) {
                A->asleep = 1;
                for (int i = 0; i < A->c.nn; i++) A->c.n[i].v = v3(0, 0, 0);
            }
        }
    }
}

int airframe_damaged_beams(const Airframe *A) {
    int n = 0;
    for (int i = 0; i < A->c.nb; i++) n += A->c.b[i].broken || fabsf(A->c.b[i].rest - A->c.b[i].rest0) > 0.01f * A->c.b[i].rest0;
    return n;
}

// ---- parallel per-car work -------------------------------------------------------------------------
// Within one substep the cars only interact in collide_cars (and with the airframes), so the per-car
// force pass and the per-car integrate + static-collision pass can run on all cores. The substeps
// themselves stay sequential. Falls back to a plain loop where Grand Central Dispatch is missing, and
// for very few active cars, where waking worker threads costs more than it saves.
#ifdef __APPLE__
#include <dispatch/dispatch.h>
#define PAR_MIN_CARS 3
#endif
typedef struct { World *w; float h; int idx[MAX_CARS]; } ParJob;

static void car_force_pass(World *w, Car *c, float h) {
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
static void car_integrate_pass(World *w, Car *c, float h) {
    for (int i = 0; i < c->nn; i++) {
        Node *N = &c->n[i];
        N->v = vadd(N->v, vscale(N->f, h / N->m));
        N->p = vadd(N->p, vscale(N->v, h));
        if (!(fabsf(N->v.x) + fabsf(N->v.y) + fabsf(N->v.z) < 400.0f)) c->exploded = 1;  // also catches NaN
        water_node(N, h);
    }
    collide_static(w, c);
}
static void par_force(void *ctx, size_t k) { ParJob *j = ctx; car_force_pass(j->w, &j->w->cars[j->idx[k]], j->h); }
static void par_integrate(void *ctx, size_t k) { ParJob *j = ctx; car_integrate_pass(j->w, &j->w->cars[j->idx[k]], j->h); }

// Run fn over the active cars, in parallel when there are enough of them.
static void for_active_cars(World *w, float h, void (*fn)(void *, size_t), void (*serial)(World *, Car *, float)) {
    ParJob job = {w, h, {0}};
    int n = 0;
    for (int ci = 0; ci < w->ncars; ci++) if (w->cars[ci].active) job.idx[n++] = ci;
#ifdef __APPLE__
    static int serialOnly = -1;
    if (serialOnly < 0) serialOnly = getenv("BEAMLITE_SERIAL") != NULL;     // A/B switch for profiling
    if (!serialOnly && n >= PAR_MIN_CARS) { dispatch_apply_f((size_t)n, dispatch_get_global_queue(QOS_CLASS_USER_INTERACTIVE, 0), &job, fn); return; }
#endif
    (void)fn;
    for (int k = 0; k < n; k++) serial(w, &w->cars[job.idx[k]], h);
}

// ---- buildings: breaches and collapse -----------------------------------------------------------
static void bld_event(World *w, BldEvent e) { if (w->nbev < MAX_BLD_EVENTS) w->bev[w->nbev++] = e; }

// Hand the gathered impact over as one breach (one per crash, unless it is a very big one).
static void bld_flush_hit(World *w, int bi) {
    Building *B = &w->bld[bi];
    if (B->hitImp >= 400.0f) {
        V3 n = B->hitN, an = v3(fabsf(n.x), fabsf(n.y), fabsf(n.z));
        n = an.x >= an.y && an.x >= an.z ? v3(n.x > 0 ? 1.0f : -1.0f, 0, 0) : an.z >= an.y ? v3(0, 0, n.z > 0 ? 1.0f : -1.0f) : v3(0, n.y > 0 ? 1.0f : -1.0f, 0);
        bld_event(w, (BldEvent){bi, vscale(B->hitP, 1.0f / B->hitImp), n, B->hitImp, 0});
        if (n.y == 0) B->fallDir = vscale(n, -1.0f);
    }
    B->hitImp = 0; B->hitP = B->hitN = v3(0, 0, 0); B->hitQuiet = 0;
}

static void world_update_buildings(World *w, float h) {
    for (int bi = 0; bi < w->nbld; bi++) {
        Building *B = &w->bld[bi];
        if (B->collapsed) {
            if (B->collapseT < 1.0f) {
                float Hb = B->mx.y - B->ground;
                B->collapseT = fminf(1.0f, B->collapseT + h / clampf(1.8f + 0.025f * Hb, 1.8f, 5.0f));
                float t = B->collapseT;
                for (int m = 0; m < g_nmound; m++) if (g_moundOf[m] == bi) g_mound[m].k = t * t * (3.0f - 2.0f * t);
            }
            continue;
        }
        if (B->hitImp > 0.0f) {
            B->hitQuiet += h;
            if (B->hitQuiet > 0.05f || B->hitImp > 0.5f * B->strength) bld_flush_hit(w, bi);
        }
        if (B->damage >= B->strength) {             // it goes: no more wall, a pile rising where it stood
            if (B->hitImp > 0.0f) bld_flush_hit(w, bi);
            B->collapsed = 1; B->collapseT = 0.0f;
            if (vlen(B->fallDir) < 0.5f) B->fallDir = v3(1, 0, 0);
            w->box[B->box].dead = 1;
            w->bldDown++;
            if (g_nmound < MAX_BLD) { make_mound(B, &g_mound[g_nmound]); g_mound[g_nmound].k = 0.0f; g_moundOf[g_nmound++] = bi; }
            bld_event(w, (BldEvent){bi, vscale(vadd(B->mn, B->mx), 0.5f), B->fallDir, B->damage, 1});
        }
    }
}

void world_substep(World *w, float h) {
    update_trees(w, h);
    for_active_cars(w, h, par_force, car_force_pass);
    for (int a = 0; a < w->ncars; a++)
        for (int b = a + 1; b < w->ncars; b++)
            if (w->cars[a].active && w->cars[b].active) collide_cars(&w->cars[a], &w->cars[b]);
    if (w->nplanes) airframes_substep(w, h, 0);
    for_active_cars(w, h, par_integrate, car_integrate_pass);
    if (w->nplanes) airframes_substep(w, h, 1);
    world_update_buildings(w, h);
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
// Airport: the cross road stops at the highway; an access road runs up to the terminal forecourt.
static const Rect AP_ROADS[] = {
    {-260, 260, -7, 7},          // highway
    {-7, 7, -260, 7},            // cross road, south only
};
static const Rect AP_STREETS[] = {
    {-7, 7, 0, 46},              // access road
    {-66, 114, 36, 48},          // departures forecourt (wide enough for cabs at the kerb)
    {-64, -56, 7, 80}, {104, 112, 38, 80},  // west and east service roads
    {-64, 112, 74, 80},          // apron service road (between the terminal and the apron)
    {123, 136, 38, 44},          // east road on past the tower to the hangars
    {30, 78, -13, -7},           // service road into the warehouse district
};
typedef struct { Rect r; int type; } Area;
static const Area AP_AREAS[] = {
    {{AP_RWY_X0, AP_RWY_X1, AP_RWY_Z0, AP_RWY_Z1}, 4},                    // runway 09/27
    {{-AP_TWY_X, AP_TWY_X1, AP_TWY_Z0, AP_TWY_Z1}, 4},                    // parallel taxiway
    {{594, 606, AP_TWY_Z1, AP_RWY_Z0}, 4}, {{1228, 1240, AP_TWY_Z1, AP_RWY_Z0}, 4},   // connectors out on the causeway
    {{-110, 112, AP_APRON_Z0, AP_APRON_Z1}, 5},                           // apron
    {{-212, -110, 122, AP_APRON_Z1}, 5},                                  // hangar apron
    {{112, AP_EAPRON_X1, AP_EAPRON_Z0, AP_APRON_Z1}, 5},                  // east apron, in front of the hangar row
    {{AP_HELI_X - AP_HELI_HALF, AP_HELI_X + AP_HELI_HALF, AP_HELI_Z - AP_HELI_HALF, AP_HELI_Z + AP_HELI_HALF}, 5},   // helipad
    {{136, AP_EAPRON_X1, 44, AP_EAPRON_Z0}, 5},                           // hangar floors and the yard behind
    {{93, 105, AP_TWY_Z1, AP_RWY_Z0}, 4},                                 // connector to the east apron
    {{-241, -229, AP_TWY_Z1, AP_RWY_Z0}, 4}, {{-6, 6, AP_TWY_Z1, AP_RWY_Z0}, 4}, {{229, 241, AP_TWY_Z1, AP_RWY_Z0}, 4},   // connectors
    {{10, 90, 14, 34}, 4}, {{-50, -12, 14, 34}, 4},                       // car parks
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

static const char *MAP_NAMES[MAP_COUNT] = {"DEFAULT", "NEW YORK", "AIRPORT"};
const char *map_name(int map) { return MAP_NAMES[map < 0 || map >= MAP_COUNT ? 0 : map]; }
int map_has_broadway(void) { return g_map == MAP_NEWYORK; }
#define STREETS (g_map == MAP_NEWYORK ? NY_STREETS : g_map == MAP_AIRPORT ? AP_STREETS : DEF_STREETS)
#define NSTREETS (g_map == MAP_NEWYORK ? NELEM(NY_STREETS) : g_map == MAP_AIRPORT ? NELEM(AP_STREETS) : NELEM(DEF_STREETS))
#define ROADSM (g_map == MAP_AIRPORT ? AP_ROADS : ROADS)
#define NROADS (g_map == MAP_AIRPORT ? NELEM(AP_ROADS) : NELEM(ROADS))
#define NAREAS (g_map == MAP_AIRPORT ? NELEM(AP_AREAS) : 0)

float broadway_x(float z) { return TS_X + BW_DX * (z - TS_Z); }
float broadway_dist(float x, float z) { return fabsf(x - broadway_x(z)) / sqrtf(1.0f + BW_DX * BW_DX); }
static int on_broadway(float x, float z, float m) { return g_map == MAP_NEWYORK && z > CITY_Z0 && z < CITY_Z1 && broadway_dist(x, z) < BW_HALF + m; }
static int in_airport(float x, float z) { return g_map == MAP_AIRPORT && fabsf(x) < 268.0f && z > 6.0f && z < 225.0f; }
static int in_city(float x, float z) { return g_map == MAP_NEWYORK && fabsf(x) < CITY_X + 6 && z > CITY_Z0 - 2 && z < CITY_Z1 + 6; }

static int in_rect(const Rect *r, float x, float z, float m) {
    return x > r->x0 - m && x < r->x1 + m && z > r->z0 - m && z < r->z1 + m;
}
static int rect_overlap(const Rect *a, const Rect *b, float m) {
    return a->x0 < b->x1 + m && b->x0 < a->x1 + m && a->z0 < b->z1 + m && b->z0 < a->z1 + m;
}

int road_at(float x, float z) {
    for (int i = 0; i < NROADS; i++) if (in_rect(&ROADSM[i], x, z, 0)) return 1;
    for (int i = 0; i < NSTREETS; i++) if (in_rect(&STREETS[i], x, z, 0)) return 2;
    if (on_broadway(x, z, 0)) return fabsf(z - TS_Z) < PLAZA_HALF ? 3 : 2;
    for (int i = 0; i < NAREAS; i++) if (in_rect(&AP_AREAS[i].r, x, z, 0)) return AP_AREAS[i].type;
    return 0;
}

// ---- building reset
static void shift_car(Car *c, V3 d) {
    for (int i = 0; i < c->nn; i++) c->n[i].p = vadd(c->n[i].p, d);
    car_update_frame(c);
}
// Move a vehicle out of building B's footprint through the nearest side, if it is in it.
static int evict_from(Car *c, const Building *B) {
    if (c->nn <= 0) return 0;
    V3 lo = c->n[0].p, hi = lo;
    for (int i = 1; i < c->nn; i++) {
        V3 p = c->n[i].p;
        lo = v3(fminf(lo.x, p.x), fminf(lo.y, p.y), fminf(lo.z, p.z));
        hi = v3(fmaxf(hi.x, p.x), fmaxf(hi.y, p.y), fmaxf(hi.z, p.z));
    }
    const float gap = 0.8f;
    if (hi.x < B->mn.x - gap || lo.x > B->mx.x + gap || hi.z < B->mn.z - gap || lo.z > B->mx.z + gap || lo.y > B->mx.y) return 0;
    float px = B->mx.x + gap - lo.x, nx = hi.x - (B->mn.x - gap), pz = B->mx.z + gap - lo.z, nz = hi.z - (B->mn.z - gap);
    float m = fminf(fminf(px, nx), fminf(pz, nz));
    V3 d = m == px ? v3(px, 0, 0) : m == nx ? v3(-nx, 0, 0) : m == pz ? v3(0, 0, pz) : v3(0, 0, -nz);
    // Rest it on the ground there, not hanging where the rubble was (or buried under a slope).
    float clear = c->def ? c->def->spawnH + 0.2f : 0.3f;       // a car's lowest node is its floor, above the wheels
    d.y = terrain_base(0.5f * (lo.x + hi.x) + d.x, 0.5f * (lo.z + hi.z) + d.z) + clear - lo.y;
    shift_car(c, d);
    for (int i = 0; i < c->nn; i++) c->n[i].v = v3(0, 0, 0);
    return 1;
}
int world_reset_buildings(World *w) {
    int n = 0;
    for (int bi = 0; bi < w->nbld; bi++) {
        Building *B = &w->bld[bi];
        int was = B->collapsed;
        B->damage = 0; B->hitImp = 0; B->hitP = B->hitN = v3(0, 0, 0); B->hitQuiet = 0;
        B->collapsed = 0; B->collapseT = 0; B->fallDir = v3(0, 0, 0);
        w->box[B->box].dead = 0;
        if (!was) continue;
        n++;
        g_nmound = 0;                           // the piles go before anything is placed back on the ground
        for (int i = 0; i < w->ncars; i++) if (w->cars[i].active) evict_from(&w->cars[i], B);
        for (int i = 0; i < w->nplanes; i++) if (evict_from(&w->air[i].c, B)) { w->air[i].asleep = 0; airframe_update_frame(&w->air[i]); }
    }
    g_nmound = 0;
    w->bldDown = 0; w->nbev = 0;
    return n;
}

// ---- rubber ducks
static unsigned duck_rng;
static float dkrnd(void) { duck_rng = duck_rng * 1664525u + 1013904223u; return (duck_rng >> 8) / 16777216.0f; }
static float dkrange(float a, float b) { return a + (b - a) * dkrnd(); }

static int duck_spot_ok(const World *w, V3 p, int onGround) {
    if (fabsf(p.x) > 285.0f || fabsf(p.z) > 285.0f) return 0;
    if (p.x * p.x + p.z * p.z < 18.0f * 18.0f) return 0;               // not right where you start
    for (int k = 0; k < 5; k++) {                                       // clear of walls and trunks all round
        V3 q = v3(p.x + (k == 1 ? 0.5f : k == 2 ? -0.5f : 0), p.y + 0.2f, p.z + (k == 3 ? 0.5f : k == 4 ? -0.5f : 0));
        if (world_point_solid(w, q)) return 0;
    }
    if (onGround) {
        if (sea_dist(p.x, p.z) > -2.0f || p.y < AP_SEA_Y + 0.3f) return 0;
        V3 n = terrain_normal(p.x, p.z);
        if (n.y < 0.9f) return 0;
        for (int i = 0; i < w->nplanes; i++) {                          // not under a parked aircraft
            float dx = p.x - w->planes[i].x, dz = p.z - w->planes[i].z, r = w->planes[i].kind == AC_HELI ? 14.0f : 45.0f;
            if (dx * dx + dz * dz < r * r) return 0;
        }
    }
    for (int i = 0; i < w->nduck; i++) {
        float dx = p.x - w->duck[i].p.x, dz = p.z - w->duck[i].p.z;
        if (dx * dx + dz * dz < 22.0f * 22.0f) return 0;
    }
    return 1;
}
static void add_duck(World *w, V3 p, int bld, int water) {
    if (w->nduck >= MAX_DUCKS) return;
    w->duck[w->nduck++] = (Duck){p, dkrange(0, 6.2831853f), bld, water, 0};
}

static void place_ducks(World *w) {
    duck_rng = 0x0D0C0u + 7919u * (unsigned)w->map;
    w->nduck = 0;
    int airport = w->map == MAP_AIRPORT;
    // Tucked behind buildings, near a back corner, off the street.
    for (int t = 0, got = 0; t < 600 && got < 8 && w->nbld > 0; t++) {
        const Building *B = &w->bld[(int)(dkrnd() * w->nbld) % w->nbld];
        int f = (int)(dkrnd() * 4) & 3;
        if (f == B->front) continue;
        float u = dkrnd() < 0.5f ? dkrange(0.06f, 0.2f) : dkrange(0.8f, 0.94f), out = 0.75f;
        float x, z;
        if (f < 2) { x = f == 0 ? B->mx.x + out : B->mn.x - out; z = lerpf(B->mn.z, B->mx.z, u); }
        else { z = f == 2 ? B->mx.z + out : B->mn.z - out; x = lerpf(B->mn.x, B->mx.x, u); }
        V3 p = v3(x, terrain_height(x, z), z);
        if (road_at(x, z) || !duck_spot_ok(w, p, 1)) continue;
        add_duck(w, p, -1, 0); got++;
    }
    // At the foot of a tree.
    for (int t = 0, got = 0; t < 400 && got < 5 && w->ntree > 0; t++) {
        const Tree *T = &w->tree[(int)(dkrnd() * w->ntree) % w->ntree];
        float a = dkrange(0, 6.2831853f), r = T->r + 0.7f;
        float x = T->x + r * cosf(a), z = T->z + r * sinf(a);
        V3 p = v3(x, terrain_height(x, z), z);
        if (road_at(x, z) || !duck_spot_ok(w, p, 1)) continue;
        add_duck(w, p, -1, 0); got++;
    }
    // On flat roofs low enough to knock down with a car (anything at the airport: there is a helicopter).
    for (int t = 0, got = 0; t < 600 && got < 3 && w->nbld > 0; t++) {
        int bi = (int)(dkrnd() * w->nbld) % w->nbld;
        const Building *B = &w->bld[bi];
        if (B->roof > 0.0f || B->kind == BLD_WEDGE || B->kind == BLD_STEPS) continue;
        if (!airport && B->mx.y - B->ground > 22.0f) continue;
        if (B->mx.x - B->mn.x < 5.0f || B->mx.z - B->mn.z < 5.0f) continue;
        V3 p = v3(dkrange(B->mn.x + 1.5f, B->mx.x - 1.5f), B->mx.y, dkrange(B->mn.z + 1.5f, B->mx.z - 1.5f));
        if (!duck_spot_ok(w, p, 0)) continue;
        add_duck(w, p, bi, 0); got++;
    }
    // Afloat in the bay, not far out.
    for (int t = 0, got = 0; airport && t < 2000 && got < 2; t++) {
        float x = dkrange(-250, 280), z = dkrange(120, 284);
        float s = sea_dist(x, z);
        if (s < 6.0f || s > 30.0f) continue;
        V3 p = v3(x, AP_SEA_Y, z);
        if (!duck_spot_ok(w, p, 0)) continue;
        add_duck(w, p, -1, 1); got++;
    }
    // The rest out in the open: fields, forest, the odd corner.
    for (int t = 0; t < 6000 && w->nduck < MAX_DUCKS; t++) {
        float x = dkrange(-270, 270), z = dkrange(-270, 270);
        V3 p = v3(x, terrain_height(x, z), z);
        if (road_at(x, z) || !duck_spot_ok(w, p, 1)) continue;
        add_duck(w, p, -1, 0);
    }
}

V3 duck_pos(const World *w, const Duck *d, float time) {
    V3 p = d->p;
    if (d->bld >= 0 && w->bld[d->bld].collapsed) {      // the roof comes down with the building, onto the pile
        float t = w->bld[d->bld].collapseT, k = t * t * (3.0f - 2.0f * t);
        p.y += (terrain_height(p.x, p.z) - p.y) * k;
    }
    if (d->water) p.y = AP_SEA_Y + 0.06f * sinf(time * 1.7f + p.x * 0.3f) + 0.03f * sinf(time * 2.9f + p.z * 0.5f);
    return p;
}

int duck_touch(World *w, const Car *c) {
    if (c->nn <= 0) return -1;
    V3 ref = c->n[0].p;
    for (int i = 0; i < w->nduck; i++) {
        Duck *d = &w->duck[i];
        if (d->found) continue;
        V3 q = duck_pos(w, d, 0);
        q.y += 0.2f;
        V3 r = vsub(q, ref);
        if (vdot(r, r) > 60.0f * 60.0f) continue;
        for (int k = 0; k < c->nn; k++) {
            V3 e = vsub(c->n[k].p, q);
            if (vdot(e, e) < 1.5f * 1.5f) { d->found = 1; return i; }
        }
    }
    return -1;
}

int road_rects(float (*out)[4], int max, int *nHighway) {
    int n = 0;
    for (int i = 0; i < NROADS && n < max; i++, n++) { out[n][0] = ROADSM[i].x0; out[n][1] = ROADSM[i].x1; out[n][2] = ROADSM[i].z0; out[n][3] = ROADSM[i].z1; }
    *nHighway = n;
    for (int i = 0; i < NSTREETS && n < max; i++, n++) { out[n][0] = STREETS[i].x0; out[n][1] = STREETS[i].x1; out[n][2] = STREETS[i].z0; out[n][3] = STREETS[i].z1; }
    for (int i = 0; i < NAREAS && n < max; i++, n++) { const Rect *r = &AP_AREAS[i].r; out[n][0] = r->x0; out[n][1] = r->x1; out[n][2] = r->z0; out[n][3] = r->z1; }
    return n;
}

static unsigned rng_state;
static float frnd(void) { rng_state = rng_state * 1664525u + 1013904223u; return (rng_state >> 8) / 16777216.0f; }
static float frange(float a, float b) { return a + (b - a) * frnd(); }

static int add_box(World *w, V3 mn, V3 mx) {
    if (w->nbox >= MAX_BOXES) return -1;
    w->box[w->nbox].mn = mn; w->box[w->nbox].mx = mx;
    w->box[w->nbox].bld = -1; w->box[w->nbox].dead = 0;
    return w->nbox++;
}

// Is this footprint free of roads, keep-clear zones, boxes and trees?
static int area_free(const World *w, Rect r, float margin) {
    float rm = fminf(margin, 1.0f);             // kerb clearance is smaller than the spacing between things
    for (int i = 0; i < NROADS; i++) if (rect_overlap(&r, &ROADSM[i], rm)) return 0;
    for (int i = 0; i < NAREAS; i++) if (rect_overlap(&r, &AP_AREAS[i].r, rm)) return 0;
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
    if (sea_dist(x, z) > -6.0f) return;               // not on the beach or in the bay
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
    B->roof = kind == BLD_HOUSE ? frange(2.4f, 3.4f) : kind == BLD_WAREHOUSE ? 1.4f : kind == BLD_HANGAR ? 4.5f : 0.0f;
    B->seed = (unsigned)(frnd() * 4294967040.0f);
    // Strength grows with size (about the wall area that has to give way); sheds and hangars are light,
    // the control tower is stocky concrete. A 1.4 t hatch at ~120 km/h takes down a house; a tower
    // needs the airliner (or a lot of trucks).
    float H = B->mx.y - B->ground;
    float vol = (r.x1 - r.x0) * (r.z1 - r.z0) * fmaxf(H, 2.0f);
    float kk = kind == BLD_WAREHOUSE ? 0.6f : kind == BLD_HANGAR ? 0.5f : kind == BLD_ATC ? 1.5f : (kind == BLD_TOWER || kind == BLD_WEDGE) ? 1.1f : 1.0f;
    B->strength = BLD_STRENGTH_K * kk * powf(vol, 2.0f / 3.0f);
    B->moundH = clampf(0.13f * H + (B->roof > 0 ? 0.3f : 0.0f), 1.0f, 6.0f);
    int bi = add_box(w, B->mn, B->mx);
    if (bi >= 0) { B->box = bi; w->box[bi].bld = w->nbld; w->nbld++; }
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
            if (!B->dead && p.x > B->mn.x && p.x < B->mx.x && p.y > B->mn.y && p.y < B->mx.y && p.z > B->mn.z && p.z < B->mx.z) return 1;
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

// Airport map: terminal, gate airliners, control tower, hangars (and a parked airliner outside them).
static void add_plane(World *w, float x, float z, float dx, float dz) {
    if (w->nplanes >= MAX_PLANES) return;
    w->planes[w->nplanes++] = (Plane){x, z, dx, dz, (unsigned)(frnd() * 4294967040.0f), AC_AIRLINER};
}
static void build_airport(World *w) {
    add_building(w, (Rect){AP_TERM_X0, AP_TERM_X1, AP_TERM_Z0, AP_TERM_Z1}, BLD_TERMINAL, 16.0f, 3);
    for (int g = 0; g < AP_NGATES; g++) if (g != 2) add_plane(w, AP_GATES[g], AP_NOSE_Z, 0, 1);   // nose-in at the gates (one stand left free)
    add_building(w, (Rect){AP_ATC_X0, AP_ATC_X1, 55, 63}, BLD_ATC, 44.0f, 3);
    add_building(w, (Rect){-205, -160, 70, 120}, BLD_HANGAR, 18.0f, 2);
    add_building(w, (Rect){-150, -114, 76, 120}, BLD_HANGAR, 15.0f, 2);
    // The maintenance row east of the tower, doors open onto the east apron; the big one takes an airliner.
    for (int i = 0; i < 3; i++) add_building(w, (Rect){AP_HANGARS[i][0], AP_HANGARS[i][1], AP_HANGAR_Z0, AP_EAPRON_Z0}, BLD_HANGAR, AP_HANGARS[i][2], 2);
    add_plane(w, -128.0f, 130.0f, -1, 0);                                   // outside the west hangars
    add_plane(w, 0.5f * (AP_HANGARS[2][0] + AP_HANGARS[2][1]), 134.0f, 0, -1);   // pulled out of the big hangar, nose to the taxiway
    w->heli = w->nplanes;                                                    // on the helipad beside the tower, nose to the east
    if (w->nplanes < MAX_PLANES) w->planes[w->nplanes++] = (Plane){AP_HELI_X, AP_HELI_Z, -1, 0, (unsigned)(frnd() * 4294967040.0f), AC_HELI};
    w->flyable = w->nplanes;
    add_plane(w, AP_RWY_X0 + 46.0f, 0.5f * (AP_RWY_Z0 + AP_RWY_Z1), -1, 0);  // lined up on runway 09: the one you can fly
    Block ind = {{34, 74, -38, -14}, 3};                                     // warehouses (shared proving ground)
    fill_block(w, &ind);
}

void world_init(World *w, int map) {
    memset(w, 0, sizeof *w);
    g_nmound = 0;
    w->flyable = w->heli = -1;
    w->map = g_map = (map < 0 || map >= MAP_COUNT) ? MAP_DEFAULT : map;
    rng_state = 20240611u;
    // Crash-test wall behind the spawn point.
    add_box(w, v3(-74, -1, -14), v3(-71, 3.0f, 14));
    // A row of concrete pillars beside the main road (elsewhere than Default, moved clear of the side roads).
    const float pxD[6] = {-60, -46, -32, -18, 18, 32}, pxN[6] = {-66, -40, -30, -18, 18, 32};
    for (int i = 0; i < 6; i++) {
        float x = g_map == MAP_DEFAULT ? pxD[i] : pxN[i];
        add_box(w, v3(x - 0.6f, -1, 11.0f), v3(x + 0.6f, 4.0f, 12.2f));
    }
    // Low barrier blocks scattered on the plaza.
    add_box(w, v3(18, -1, -26), v3(30, 0.8f, -24.5f));
    add_box(w, v3(-30, -1, -30), v3(-28.5f, 1.2f, -18));

    if (g_map == MAP_NEWYORK) build_newyork(w);
    else if (g_map == MAP_AIRPORT) build_airport(w);
    else build_default(w);

    // Roadside trees along the highway and the cross road, out in the country.
    float crossNorth = g_map == MAP_NEWYORK ? 142.0f : g_map == MAP_AIRPORT ? 1e9f : 100.0f;
    for (float x = 66; x < 250; x += 11) { try_tree(w, x, 14.5f + frange(-1, 1), frange(6, 9), TREE_BROADLEAF, 2.0f); try_tree(w, x + 5, -14.5f + frange(-1, 1), frange(6, 9), TREE_BROADLEAF, 2.0f); }
    for (float x = -86; x > -250; x -= 11) { try_tree(w, x, 14.5f + frange(-1, 1), frange(6, 9), TREE_BROADLEAF, 2.0f); try_tree(w, x - 5, -14.5f + frange(-1, 1), frange(6, 9), TREE_BROADLEAF, 2.0f); }
    for (float z = crossNorth; z < 250; z += 12) { try_tree(w, 12, z, frange(6, 10), TREE_CONIFER, 2.0f); try_tree(w, -12, z + 6, frange(6, 10), TREE_CONIFER, 2.0f); }
    for (float z = -80; z > -250; z -= 12) { try_tree(w, 12, z, frange(6, 10), TREE_CONIFER, 2.0f); try_tree(w, -12, z - 6, frange(6, 10), TREE_CONIFER, 2.0f); }

    // Forest on the hills, thicker further out; a few loners on the flat.
    for (int i = 0; i < 2600 && w->ntree < MAX_TREES - 40; i++) {
        float r = frange(95, 290), a = frange(0, 6.2831853f);
        float x = r * cosf(a), z = r * sinf(a);
        if (in_city(x, z) || in_airport(x, z)) continue;
        if (frnd() > smooth01(90, 200, r) + 0.15f) continue;
        int conifer = frnd() < 0.35f + 0.5f * smooth01(110, 220, r);
        try_tree(w, x, z, conifer ? frange(8, 15) : frange(6, 10), conifer ? TREE_CONIFER : TREE_BROADLEAF, 3.2f);
    }
    for (int i = 0; i < 120; i++) {
        float x = frange(-88, 88), z = frange(-88, 88);
        if (x * x + z * z > 88 * 88 || in_city(x, z) || in_airport(x, z)) continue;
        try_tree(w, x, z, frange(5, 9), frnd() < 0.7f ? TREE_BROADLEAF : TREE_CONIFER, 6.0f);
    }

    world_build_grid(w);
    w->ncars = 0;
    for (int i = 0; i < w->nplanes; i++) {             // soft-body airframes, settled onto their gear
        const Plane *P = &w->planes[i];
        if (P->kind == AC_HELI) heli_build(&w->air[i], v3(P->x, terrain_height(P->x, P->z), P->z), v3(P->dx, 0, P->dz), P->seed);
        else airframe_build(&w->air[i], v3(P->x, terrain_height(P->x, P->z), P->z), v3(P->dx, 0, P->dz), P->seed);
        airframe_settle(w, &w->air[i]);
    }
    place_ducks(w);
}
