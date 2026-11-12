// BeamLite — soft-body (node/beam) vehicle physics. Pure C, no dependencies.
#ifndef BEAMLITE_PHYSICS_H
#define BEAMLITE_PHYSICS_H

#include <math.h>

typedef struct { float x, y, z; } V3;

static inline V3 v3(float x, float y, float z) { V3 r = {x, y, z}; return r; }
static inline V3 vadd(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline V3 vsub(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline V3 vscale(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline float vdot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 vcross(V3 a, V3 b) { return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
static inline float vlen(V3 a) { return sqrtf(vdot(a, a)); }
static inline V3 vnorm(V3 a) { float l = vlen(a); return l > 1e-9f ? vscale(a, 1.0f / l) : v3(0, 0, 0); }
static inline V3 vlerp(V3 a, V3 b, float t) { return vadd(a, vscale(vsub(b, a), t)); }
// Rotate v around unit axis k by angle a (Rodrigues).
static inline V3 vrot(V3 v, V3 k, float a) {
    float c = cosf(a), s = sinf(a);
    return vadd(vadd(vscale(v, c), vscale(vcross(k, v), s)), vscale(k, vdot(k, v) * (1 - c)));
}

#define MAX_NODES 192
#define MAX_BEAMS 2048
#define MAX_TRIS 512
#define MAX_PANELS 8
#define MAX_CARS 8
#define MAX_BOXES 64

#define SIM_HZ 1200.0f          // physics substep rate
#define WHEEL_R 0.44f           // 35" all-terrain tyre
#define WHEEL_W 0.30f
#define CAR_SPAWN_H 0.56f       // floor height above ground when a truck is dropped in

typedef struct { V3 p, v, f; float m; } Node;

enum { BEAM_NORMAL, BEAM_JOINT, BEAM_CABLE, BEAM_STOP };

typedef struct {
    int a, b;
    float rest, rest0;          // current (plastic) and original rest length
    float k, c, yield;          // stiffness, damping, plastic yield strain
    float brk;                  // JOINT/CABLE: elastic strain at which it snaps
    float stress;               // last elastic strain / yield (for display; |x|>1 means yielding)
    unsigned char type, broken;
} Beam;

typedef struct {
    int mount, upper;           // nodes the strut attaches to
    int front;
    float off;                  // lateral offset of the contact patch from the mount node
    float driveShare;           // fraction of engine force on this wheel
    int contact;
    float compress, load, slip, spin, steerAngle;
    V3 hub, cp, side;           // wheel centre, contact point, lateral axis
} Wheel;

enum { SURF_BODY, SURF_GLASS, SURF_UNDER, SURF_TONNEAU, SURF_TRIM };

typedef struct {
    int v[3];
    unsigned char kind;         // SURF_*
    unsigned char edges;        // bit e: outline edge v[e] -> v[(e+1)%3]
    signed char panel;          // glass panel id, -1 if not glass
    float e0[3];                // original edge lengths (glass breakage)
} Tri;

typedef struct { float throttle, brake, steer; int handbrake; } Controls;

enum { TG_LATCHED, TG_OPEN, TG_HANGING, TG_GONE };

typedef struct {
    Node n[MAX_NODES]; int nn, nBody;   // nodes [0, nBody) are the body; the rest is the tailgate
    Beam b[MAX_BEAMS]; int nb;
    Tri t[MAX_TRIS]; int nt;
    Wheel w[4];
    Controls ctl;
    float steer;                // actual front-wheel angle (rad)
    V3 pos, vel, fwd, right, up, bmin, bmax;
    V3 color;
    float damage;               // 0..100
    int brokenBeams;
    int exploded;
    int active;
    // Glass: each panel shatters as a whole when its frame is strained.
    int npanels, glassBrokenCount;
    unsigned char glassBroken[MAX_PANELS];
    unsigned glassNew;          // bitmask of panels that shattered since the renderer last looked
    // Tailgate: hinged at the bottom, latched at the top, check straps hold it level when open.
    int tgLatch[6], tgHingeL[3], tgHingeR[3], tgStrap[2];
    int tailgate;               // TG_*
    int lightF[4], lightR[4];   // light-bar strip nodes (top L, top R, lower L, lower R)
} Car;

typedef struct { V3 mn, mx; } Box;

typedef struct {
    Car cars[MAX_CARS]; int ncars;
    Box box[MAX_BOXES]; int nbox;
} World;

float terrain_height(float x, float z);
V3 terrain_normal(float x, float z);

void world_init(World *w);
void world_substep(World *w, float h);
void car_build(Car *c, V3 pos, float yaw, V3 color);
void car_update_frame(Car *c);
void car_update_damage(Car *c);
void car_pop_tailgate(Car *c);

#endif
