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

#define MAX_NODES 320
#define MAX_BEAMS 2048
#define MAX_TRIS 640
#define MAX_PANELS 12
#define MAX_CARS 14
#define MAX_BOXES 1000
#define MAX_BLD 900
#define MAX_TREES 2048
#define GRID_N 100               // collision grid over the 600 m world
#define GRID_CELL 6.0f
#define MAX_GRID_ITEMS 32000

#define SIM_HZ 1200.0f          // physics substep rate
#define CAR_SPAWN_H 0.56f       // floor height above ground when the pickup is dropped in (tests)
#define MAX_WHEELS 6
#define NR_MAX 24               // body rings per vehicle (at most)

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

enum { SURF_BODY, SURF_GLASS, SURF_UNDER, SURF_TONNEAU, SURF_TRIM, SURF_CHROME };

typedef struct {
    int v[3];
    unsigned char kind;         // SURF_*
    unsigned char edges;        // bit e: outline edge v[e] -> v[(e+1)%3]
    signed char panel;          // glass panel id, -1 if not glass
    float e0[3];                // original edge lengths (glass breakage)
} Tri;

typedef struct { float throttle, brake, steer; int handbrake; } Controls;

enum { TG_LATCHED, TG_OPEN, TG_HANGING, TG_GONE, TG_NONE };

// ---- vehicle definitions
// A body is nr (<= NR_MAX) cross-section rings from nose to tail. Each ring has floor-centre, floor-edge,
// sill, beltline and roof-edge nodes on each side; the table below shapes them.
typedef struct {
    float x, floorY, rkY, belt, top, tw;    // station x; heights (0 = floor underside); roof-edge half-width
    float ws;                               // width scale for sill/beltline here (1 = full width)
    unsigned char arch, bulk, seam;         // over a wheel arch; full bulkhead; door shut line
    signed char topPanel, sidePanel;        // for the gap to the next ring: -1 body, -2 tonneau, >=0 glass id
                                            // (side: left id, right = left + 1)
} Station;

typedef struct { int q[4]; float t, u0, u1, out; unsigned char rear; } LightStrip;

enum { VEH_PICKUP, VEH_HATCH, VEH_SEMI, VEH_POLICE, VEH_COUNT };

typedef struct {
    const char *name, *blurb;
    int nr;
    Station st[NR_MAX];
    float floorHalf, sillHalf, beltHalf;    // half-widths
    int axleF, axleR, doorR0, doorR1, dashRing, bRing;
    int axleR2;                             // second rear axle ring (tandem), or -1
    float wheelWR, trackHalfR;              // rear wheel width (duals) and half-track; 0 = same as the front
    int noEdges, chromeNose;                // no panel outlines; the nose face is a chrome grille
    float mass;
    float batW, batX0, batX1, engW, engX0, engX1;   // extra mass (weight units) on floor nodes in [x0, x1]
    float crumpleF, crumpleR, cabinX0, cabinX1;     // structural zones (x)
    float kScale, yScale;                           // structure stiffness / yield multipliers
    float wheelR, wheelW, trackHalf;
    float suspRest, suspTravel, ks, cs, kbump;
    float brakeMax, engineF, enginePower, reverseF, driveFront;
    float rearSteer, maxLock, cdA;
    float spawnH;                           // drop-in floor height
    V3 seat;                                // driver's eye point (car-local; left-hand drive, so z < 0)
    float dashX, dashY;                     // dashboard top: x of its leading edge, height
    float rearSeatX;                        // rear bench cushion centre (x)
    int npanels, hasTailgate, lightStyle;   // lightStyle: 0 full-width bars, 1 corner lamps, 2 drawn by the renderer
    int smooth;                             // smooth-shaded paint without panel outlines
    int wheelStyle;                         // 0 flat aero covers, 1 dark multi-spoke alloys
    int mirrorRing, spoilerRing;            // ring for door mirrors / roof spoiler (-1 = none)
    V3 paint;
} VehicleDef;

typedef struct {
    const VehicleDef *def; int type;
    Node n[MAX_NODES]; int nn, nBody;   // nodes [0, nBody) are the body; the rest is the tailgate
    Beam b[MAX_BEAMS]; int nb;
    Tri t[MAX_TRIS]; int nt;
    Wheel w[MAX_WHEELS]; int nw;        // FL, FR, RL, RR (, second rear axle L, R)
    V3 rest[MAX_NODES];                 // node positions in the car-local frame when built (paint coordinates)
    Controls ctl;
    float steer;                // actual front-wheel angle (rad)
    V3 pos, vel, fwd, right, up, bmin, bmax;
    V3 comLocal;                // body centre of mass in car-local coordinates (at build)
    V3 color;
    float damage;               // 0..100
    int brokenBeams;
    int exploded;
    int siren;                  // police: lights and siren running
    int active;
    // Glass: each panel shatters as a whole when its frame is strained.
    int npanels, glassBrokenCount;
    unsigned char glassBroken[MAX_PANELS];
    unsigned glassNew;          // bitmask of panels that shattered since the renderer last looked
    // Tailgate: hinged at the bottom, latched at the top, check straps hold it level when open.
    int tgLatch[6], tgHingeL[3], tgHingeR[3], tgStrap[2];
    int tailgate;               // TG_*
    int tgNode[4];              // tailgate corners: top L, top R, bottom L, bottom R
    LightStrip lights[4]; int nlights;
    int mirrorNode[4];          // belt nodes L, R at the mirror ring and the ring behind it (-1 = none)
    int spoilerNode[4];         // roof-edge nodes L, R at the spoiler ring and the ring ahead of it
    int aPillar[2][8], naPillar;  // roof-edge nodes along each windshield side (L, R), base to top
    int bPillar[4];             // belt and roof-edge nodes at the B-pillar: L belt, L roof, R belt, R roof
    float grip;                 // road-surface grip multiplier (1 dry, ~0.85 wet, ~0.5 snow)
} Car;

typedef struct { V3 mn, mx; int bld; int dead; } Box;   // bld: the building it belongs to, or -1; dead: knocked down

// Buildings collide as their box; the rest of this is for the renderer.
enum { BLD_HOUSE, BLD_SHOP, BLD_APARTMENT, BLD_TOWER, BLD_WAREHOUSE, BLD_WEDGE, BLD_STEPS, BLD_TERMINAL, BLD_ATC, BLD_HANGAR };   // wedge: the screen-clad tower in the bowtie; steps: the red grandstand
typedef struct {
    V3 mn, mx;                  // walls (mn.y is below ground on slopes)
    float ground;               // street level at the building
    float roof;                 // gable height above the walls (houses, warehouses)
    int kind, front;            // front face: 0 +x, 1 -x, 2 +z, 3 -z
    unsigned seed;
    int box;                    // its collision box
    // Destruction (see physics.c): hard hits add up to permanent damage; past `strength` it collapses.
    float strength, damage;     // N s of impact
    float hitImp;               // impact gathered for the next breach event, and where it landed
    V3 hitP, hitN;              // impulse-weighted sums of contact point and wall normal
    float hitQuiet;             // time since the last contact that added to hitImp
    int collapsed;              // 1 once it has gone
    float collapseT;            // 0..1 through the collapse (the rubble mound rises with it)
    float moundH;               // height of the rubble pile it leaves
    V3 fallDir;                 // horizontal direction it leans as it goes (away from what hit it)
} Building;

// Something the renderer should show: a wall breached at p (outward normal n) by `imp` N s of impact,
// or (collapse = 1) building `bld` starting to come down.
typedef struct { int bld; V3 p, n; float imp; int collapse; } BldEvent;
#define MAX_BLD_EVENTS 64

// Trees collide as a vertical trunk cylinder. Thin ones snap and fall when hit hard enough.
enum { TREE_CONIFER, TREE_BROADLEAF, TREE_POLE };     // a pole is a street lamp: a thin steel post that shears off
typedef struct {
    float x, y, z;              // trunk base
    float r, h;                 // trunk radius, overall height
    float breakImp;             // impulse (N s) that snaps it; 0 = unbreakable
    float imp;                  // recent impulse (decays)
    float tilt, tiltVel;        // fall angle (rad) once broken
    V3 fall;                    // horizontal fall direction
    unsigned char kind, broken;
} Tree;

// ---- maps
enum { MAP_DEFAULT, MAP_NEWYORK, MAP_AIRPORT, MAP_COUNT };
const char *map_name(int map);

// A parked aircraft: an airliner (nose position and the unit direction from nose to tail) or a
// helicopter (mast position on the pad, and the direction from nose to tail).
enum { AC_AIRLINER, AC_HELI };
typedef struct { float x, z, dx, dz; unsigned seed; int kind; } Plane;
#define MAX_PLANES 10

// Its soft-body airframe (~40 t of nodes and beams on its landing gear; see physics.c).
#define AF_RINGS 17             // fuselage frames
#define AF_RN 10                // nodes per frame (k = 0 on the right side, counter-clockwise seen from the nose)
#define AF_EN 6                 // nodes per engine ring
typedef struct {
    Car c;                      // nodes, beams, skin triangles (no def, no wheels)
    unsigned char paint[MAX_TRIS];  // per triangle: 0 white, 1 belly, 2 livery, 3 glass, 4 wing, 5 nacelle, 6 dark
    int ring[AF_RINGS][AF_RN];
    int wing[2][4][4];          // [right, left][span station][LE top, LE bottom, TE top, TE bottom]
    int eng[2][2][AF_EN];       // [side][intake ring, exhaust ring][k]
    int gear[3][2];             // nose, right main, left main: wheel node, strut top node
    int gearWheel[3][2];        // the two wheel nodes of each leg
    int noseNode, tailNode, nFusTris;
    unsigned seed;
    int asleep;                 // not simulated until a car comes near
    float quiet, vmax;
    // ---- flight (see physics.c): lifting surfaces, engines, wheels, and the pilot's controls
    struct { int n[4], m[4]; float a0, slope, gain, flap; unsigned char ctl; signed char flip; } surf[12];   // LE in, LE out, TE out, TE in (m: the same corners on the other skin)
    int nsurf;
    int pilot;                  // flown by the player: never sleeps, controls below apply
    float thr, pitch, roll, yaw, flaps, brake;   // inputs: throttle 0..1, stick/pedals -1..1, flaps 0..1, wheel brakes 0..1
    float elev, ail, rud, trim; // control deflections after rate limits and speed scaling; pitch trim
    V3 pos, vel, fwd, up, right;    // centre of mass and body axes
    float airspeed, aoa, agl;   // m/s, wing angle of attack (rad), height of the wheels above the ground
    int onGround, stall;
    int wrecked;                // too broken to fly: engines and controls are dead, the wings no longer lift
    int nbroken;                // snapped beams (updated as it breaks up)
    // ---- helicopter (kind AC_HELI; see physics.c). thr is the collective lever, pitch/roll the cyclic, yaw the pedals.
    int kind;                   // AC_*
    int mast[2], hub[4];        // mast base and rotor head; four nodes around the head that carry the rotor thrust
    int trHub, skid[2][5];      // tail-rotor hub; skids [right, left] from the upturned tip back
    int stabTip[2], finTop;     // stabiliser tips [right, left] and the top of the fin (navigation lights)
    float rpm;                  // main rotor speed, 0..1 of nominal (the engine spools it up and down)
    float rotorAngle, trAngle;  // blade angles, for drawing
    float lift;                 // rotor thrust / weight, last substep
    int rotorBroken, tailBroken;    // blades struck something: no lift / no anti-torque
    float strikeT;              // seconds since a rotor strike (for the renderer's debris), -1 never
} Airframe;
int airframe_damaged_beams(const Airframe *A);
void airframe_update_frame(Airframe *A);

// Rubber ducks hidden around the map: behind buildings, under trees, out in the fields, on low roofs
// (knock the building down, or land on it), afloat in the bay. Get your vehicle up to one and press Y.
#define MAX_DUCKS 24
typedef struct {
    V3 p;                       // where it sits (on a roof: the roof top; afloat: the sea surface)
    float yaw;
    int bld;                    // the building whose roof it is on, or -1
    int water;                  // afloat: bobs on the waves
    int found;
} Duck;

typedef struct {
    int map;                    // MAP_*
    Car cars[MAX_CARS]; int ncars;
    Box box[MAX_BOXES]; int nbox;
    Building bld[MAX_BLD]; int nbld;
    Tree tree[MAX_TREES]; int ntree;
    int treesDown;
    int bldDown;                // buildings knocked down
    BldEvent bev[MAX_BLD_EVENTS]; int nbev;   // breaches and collapses since the renderer last took them
    Plane planes[MAX_PLANES]; int nplanes;
    Airframe air[MAX_PLANES];   // one per plane
    int flyable;                // the airliner on the runway the player can fly, or -1
    int heli;                   // the helicopter on the helipad, or -1
    Duck duck[MAX_DUCKS]; int nduck;
    // Uniform grid (CSR) of static obstacles: item < MAX_BOXES is a box, otherwise a tree.
    int gridStart[GRID_N * GRID_N + 1];
    int gridItem[MAX_GRID_ITEMS];
} World;

float terrain_height(float x, float z);
float sea_dist(float x, float z);       // airport map: metres out to sea from the shore (negative on land); elsewhere far inland
void world_respawn_plane(World *w);     // put the flyable airliner back on the runway threshold
void world_respawn_heli(World *w);      // put the helicopter back on its pad, rotor stopped
V3 terrain_normal(float x, float z);
float building_mound(const World *w, int b, float x, float z);   // full height of building b's rubble pile above the ground at (x, z)
// Stand every knocked-down building back up and clear the rubble. Vehicles caught inside a footprint
// are moved out to the nearest side. Returns how many buildings came back.
int world_reset_buildings(World *w);
V3 duck_pos(const World *w, const Duck *d, float time);   // where to draw it now (rides a collapsing roof down, bobs afloat)
// The unfound duck within reach of the player's vehicle (any node within DUCK_REACH m), else -1.
// The player then presses Y to collect it.
#define DUCK_REACH 3.0f
int duck_near(const World *w, const Car *c);

// New York map: a Midtown street grid north of the highway with Broadway cutting across 7th Avenue on a
// diagonal; the "bowtie" where they cross is centred on (TS_X, TS_Z). Broadway is pedestrian plaza
// near the crossing.
#define TS_X 0.0f
#define TS_Z 70.0f
#define BW_DX 0.9f                      // Broadway: x = TS_X + BW_DX * (z - TS_Z)
#define BW_HALF 7.0f                    // Broadway half-width (perpendicular)
#define CITY_X 104.0f                   // Midtown spans |x| < CITY_X ...
// Around that core the same grid carries on to the edge of the map: avenues every 54 m, streets every 30.
#define NY_EDGE 299.0f
#define NY_NAV 11                // avenues (x centre, half-width), 7th in the middle
static const float NY_AV[NY_NAV][2] = {{0, 8}, {54, 6}, {-54, 6}, {110, 6}, {-110, 6}, {164, 6}, {-164, 6}, {218, 6}, {-218, 6}, {272, 6}, {-272, 6}};
#define NY_NST 9                 // cross streets (z0, z1)
static const float NY_ST[NY_NST][2] = {{44, 50}, {72, 78}, {100, 106}, {124, 132}, {156, 162}, {186, 192}, {216, 222}, {246, 252}, {276, 282}};
#define NY_GRID_X 278.0f         // the cross streets run this far east and west; the avenues this far north
#define NY_GRID_Z 282.0f
#define CITY_Z0 7.0f                    // ... and CITY_Z0 < z < CITY_Z1
#define CITY_Z1 132.0f
#define PLAZA_HALF 30.0f                // Broadway is plaza within this distance (in z) of the bowtie
float broadway_x(float z);              // centreline x at z
float broadway_dist(float x, float z);  // perpendicular distance from the centreline

// Airport map: runway 09/27 along x north of the terminal, parallel taxiway, apron with gates.
#define AP_RWY_X0 -245.0f
#define AP_RWY_X1 1250.0f       // the runway runs out into the bay on a causeway
#define AP_TWY_X1 1240.0f
#define AP_SEA_Y -1.2f           // sea level of the bay
#define AP_FAR 6000.0f           // the airport map's land and sea reach this far from the origin
#define AP_RWY_Z0 168.0f
#define AP_RWY_Z1 198.0f
#define AP_TWY_Z0 140.0f
#define AP_TWY_Z1 150.0f
#define AP_TWY_X 235.0f
#define AP_APRON_Z0 80.0f
#define AP_APRON_Z1 138.0f
#define AP_TERM_X0 -45.0f
#define AP_TERM_X1 100.0f
#define AP_TERM_Z0 50.0f
#define AP_TERM_Z1 70.0f
#define AP_NOSE_Z 86.0f                 // where a gate airliner's nose stops
#define AP_NGATES 5
static const float AP_GATES[AP_NGATES] = {-30.0f, 0.0f, 30.0f, 60.0f, 90.0f};
// East side: the control tower, the helipad beside it, and a row of hangars on their own apron.
#define AP_ATC_X0 115.0f
#define AP_ATC_X1 123.0f
#define AP_HELI_X 122.0f                // helipad centre
#define AP_HELI_Z 84.0f
#define AP_HELI_HALF 10.0f
#define AP_EAPRON_X1 262.0f             // the east apron runs from the main apron out to here
#define AP_EAPRON_Z0 94.0f
static const float AP_HANGARS[3][3] = {{138, 170, 14.0f}, {176, 208, 14.0f}, {214, 258, 20.0f}};   // x0, x1, wall height (doors face north)
#define AP_HANGAR_Z0 46.0f
// North side, across the runway from the terminal: an apron along the runway and a row of hangars and
// service buildings behind it, doors facing the runway. Taxiway crossings link it to the south side.
#define AP_NAPRON_X0 -200.0f
#define AP_NAPRON_X1 250.0f
#define AP_NAPRON_Z0 206.0f
#define AP_NAPRON_Z1 226.0f
#define AP_NROW_Z1 262.0f               // back walls of the north row
#define AP_NHANGARS 4
static const float AP_NHANGAR[AP_NHANGARS][3] = {{-195, -150, 16.0f}, {-140, -95, 16.0f}, {130, 180, 20.0f}, {190, 245, 18.0f}};   // x0, x1, wall height

// 0 none, 1 highway, 2 street/avenue, 3 pedestrian plaza, 4 runway/taxiway/car park asphalt, 5 apron concrete
int road_at(float x, float z);
int dirt_at(float x, float z);          // on the dirt washboard section (current map)
int map_has_broadway(void);
int road_rects(float (*out)[4], int max, int *nHighway);   // (x0, x1, z0, z1); highways first

void world_init(World *w, int map);     // terrain features, the map's streets and buildings, forest
int world_add_tree(World *w, float x, float z, float h, int kind);
void world_build_grid(World *w);        // call after adding obstacles by hand
int world_point_solid(const World *w, V3 p);
void world_substep(World *w, float h);
// A car-local point (as in the vehicle definition) in the world, following the moving body frame.
static inline V3 car_local_to_world(const Car *c, V3 l) {
    V3 d = vsub(l, c->comLocal);
    return vadd(c->pos, vadd(vadd(vscale(c->fwd, d.x), vscale(c->up, d.y)), vscale(c->right, d.z)));
}

const VehicleDef *vehicle_def(int type);
void car_build_vehicle(Car *c, int type, V3 pos, float yaw, V3 color);
void car_build(Car *c, V3 pos, float yaw, V3 color);       // the pickup
void car_update_frame(Car *c);
void car_update_damage(Car *c);
float car_engine_health(const Car *c);  // 1 = full power .. 0 = engine dead, from the car's damage
void car_pop_tailgate(Car *c);

#endif
