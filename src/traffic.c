// BeamLite — traffic: signalised junctions and AI drivers. See traffic.h.

#include "traffic.h"
#include <string.h>

// ---------------------------------------------------------------- the grid --
// Junction columns at xs (avenues along z, half-width xh) and rows at zs (streets along x, half-width zh).

// Junctions can be switched off (`off` bit per node): the highway past the launch ramp (which
// ends in a drop) and through the crash wall is no place for traffic. Every remaining junction
// keeps at least two connections, so cars never reach a dead end.
// Individual links can be blocked too (`blk`: pairs of node indices), and a grid can go without
// signals (`noSignals`: always green, nothing drawn).
typedef struct { int nx, nz; float xs[5], zs[5], xh[5], zh[5]; unsigned off; int nblk; int blk[2][2]; int noSignals; } Grid;
static const Grid GRID_DEFAULT = {3, 3, {-40, 0, 40}, {0, 48, 97}, {4, 7, 4}, {7, 4, 3}, 0, 0, {{0}}, 0};
static const Grid GRID_NEWYORK = {3, 5, {-54, 0, 54}, {0, 47, 75, 103, 128}, {6, 8, 6}, {7, 3, 3, 3, 4}, 1u << 2, 0, {{0}}, 0};
// Airport landside loop: highway, access road, forecourt, the two service roads and the apron road.
// The access road stops at the terminal (link 4-7 blocked); the highway east of the access road crosses the ramp.
static const Grid GRID_AIRPORT = {3, 3, {-60, 0, 108}, {0, 42, 77}, {4, 7, 4}, {7, 6, 3}, 1u << 2, 1, {{4, 7}}, 1};
static Grid G;
static float timer;
#define CYCLE 30.0f

static int ix_of(int n) { return n % G.nx; }
static int iz_of(int n) { return n / G.nx; }
static float node_x(int n) { return G.xs[ix_of(n)]; }
static float node_z(int n) { return G.zs[iz_of(n)]; }

static int node_on(int n) { return !(G.off >> n & 1); }
static int blocked(int a, int b) {
    for (int i = 0; i < G.nblk; i++) if ((G.blk[i][0] == a && G.blk[i][1] == b) || (G.blk[i][0] == b && G.blk[i][1] == a)) return 1;
    return 0;
}
static int neighbours(int n, int out[4]) {
    int k = 0, ix = ix_of(n), iz = iz_of(n);
    if (!node_on(n)) return 0;
    if (ix > 0 && node_on(n - 1) && !blocked(n, n - 1)) out[k++] = n - 1;
    if (ix < G.nx - 1 && node_on(n + 1) && !blocked(n, n + 1)) out[k++] = n + 1;
    if (iz > 0 && node_on(n - G.nx) && !blocked(n, n - G.nx)) out[k++] = n - G.nx;
    if (iz < G.nz - 1 && node_on(n + G.nx) && !blocked(n, n + G.nx)) out[k++] = n + G.nx;
    return k;
}

// Travel a -> b: direction (dx, dz), lane offset to the right, and whether it runs along z.
static void edge_frame(int a, int b, float *dx, float *dz, float *off, int *axisZ) {
    if (ix_of(a) == ix_of(b)) { *axisZ = 1; *dx = 0; *dz = iz_of(b) > iz_of(a) ? 1.0f : -1.0f; *off = G.xh[ix_of(a)] * 0.5f; }
    else { *axisZ = 0; *dz = 0; *dx = ix_of(b) > ix_of(a) ? 1.0f : -1.0f; *off = G.zh[iz_of(a)] * 0.5f; }
}
// Half-width of the road that crosses our path at junction n, and of the road we're on.
static float cross_half(int n, int axisZ) { return axisZ ? G.zh[iz_of(n)] : G.xh[ix_of(n)]; }
static float own_half(int n, int axisZ) { return axisZ ? G.xh[ix_of(n)] : G.zh[iz_of(n)]; }
// Right of travel (dx, dz): a car facing +x has +z on its right.
static void right_of(float dx, float dz, float *rx, float *rz) { *rx = -dz; *rz = dx; }

// Signals alternate: traffic along z gets green first, then traffic along x. Offsets stagger junctions.
static int light(int node, int axisZ) {
    if (G.noSignals) return LIGHT_GREEN;
    float t = fmodf(timer + (float)((ix_of(node) * 4 + iz_of(node) * 7) % 30), CYCLE);
    // z: green 0-11, yellow 11-13; all red 13-15.5; x: green 15.5-25.5, yellow 25.5-27.5; all red to 30.
    if (axisZ) return t < 11 ? LIGHT_GREEN : t < 13 ? LIGHT_YELLOW : LIGHT_RED;
    return (t >= 15.5f && t < 25.5f) ? LIGHT_GREEN : (t >= 25.5f && t < 27.5f) ? LIGHT_YELLOW : LIGHT_RED;
}

// ----------------------------------------------------------------- drivers --

typedef struct { float x, z; short stopNode; signed char axisZ; } Wp;
typedef struct {
    int active, slot, prev, cur;                // heading from junction prev toward cur
    Wp wp[32]; int n;
    float lastX, lastZ;                         // the waypoint reached most recently (start of the current leg)
    float stuck, bad;
} Ai;
static Ai ai[MAX_CARS];
static int aiBase = -1, aiCount, respawns;
static int why[4];                              // respawn reasons: stuck, off route, flipped, damaged
static unsigned rng = 12345u;
static float rnd(void) { rng = rng * 1664525u + 1013904223u; return (rng >> 8) / 16777216.0f; }

static void push(Ai *a, float x, float z, int stopNode, int axisZ) {
    if (a->n >= 32) return;
    a->wp[a->n++] = (Wp){x, z, (short)stopNode, (signed char)axisZ};
}

// Lane from junction a to the stop line before b.
static void append_edge(Ai *a, int from, int to) {
    float dx, dz, off, rx, rz; int ax;
    edge_frame(from, to, &dx, &dz, &off, &ax);
    right_of(dx, dz, &rx, &rz);
    float back = cross_half(to, ax) + 1.5f;
    push(a, node_x(to) - dx * back + rx * off, node_z(to) - dz * back + rz * off, to, ax);
}

// Through junction b, from the stop line (the last waypoint) onto the lane toward c.
static void append_turn(Ai *a, int from, int b, int c) {
    float d1x, d1z, o1, d2x, d2z, o2, r2x, r2z; int ax1, ax2;
    edge_frame(from, b, &d1x, &d1z, &o1, &ax1);
    edge_frame(b, c, &d2x, &d2z, &o2, &ax2);
    right_of(d2x, d2z, &r2x, &r2z);
    float out = cross_half(b, ax2) + 1.5f;
    float ex = node_x(b) + d2x * out + r2x * o2, ez = node_z(b) + d2z * out + r2z * o2;
    Wp s = a->wp[a->n - 1];
    if (ax1 == ax2) { push(a, ex, ez, -1, 0); return; }        // straight on
    float qx = ax1 ? s.x : ex, qz = ax1 ? ez : s.z;            // where the two lane lines cross
    if (d1z * d2x - d1x * d2z < 0) {                           // right turn: swing wide, away from the kerb corner
        qx += (d1x - d2x) * 2.0f; qz += (d1z - d2z) * 2.0f;
    }
    for (int k = 1; k <= 5; k++) {
        float t = k / 5.0f, u = 1 - t;
        push(a, u * u * s.x + 2 * t * u * qx + t * t * ex, u * u * s.z + 2 * t * u * qz + t * t * ez, -1, 0);
    }
}

static void extend(Ai *a) {
    while (a->n < 20) {
        int nb[4], k = neighbours(a->cur, nb), opts[4], m = 0, lefts[4], nl = 0;
        float d1x, d1z, o, d2x, d2z; int ax;
        edge_frame(a->prev, a->cur, &d1x, &d1z, &o, &ax);
        for (int i = 0; i < k; i++) {
            if (nb[i] == a->prev) continue;
            edge_frame(a->cur, nb[i], &d2x, &d2z, &o, &ax);
            if (d1z * d2x - d1x * d2z > 0.5f) lefts[nl++] = nb[i];      // a left turn: only if nothing else goes
            else opts[m++] = nb[i];
        }
        if (!m) { memcpy(opts, lefts, sizeof lefts); m = nl; }
        int next = m ? opts[(int)(rnd() * m) % m] : a->prev;
        append_turn(a, a->prev, a->cur, next);
        append_edge(a, a->cur, next);
        a->prev = a->cur; a->cur = next;
    }
}

static const V3 PAINT[] = {{0.96f, 0.74f, 0.08f}, {0.12f, 0.30f, 0.70f}, {0.80f, 0.80f, 0.82f}, {0.10f, 0.10f, 0.12f},
                           {0.60f, 0.08f, 0.08f}, {0.20f, 0.45f, 0.30f}, {0.55f, 0.56f, 0.60f}};

// Place an AI car on a random lane at least `minDist` from `avoid` and clear of other cars.
static void spawn(World *w, Ai *a, V3 avoid, float minDist) {
    int nNodes = G.nx * G.nz;
    for (int tries = 0; tries < 60; tries++) {
        int from = (int)(rnd() * nNodes) % nNodes, nb[4], k = neighbours(from, nb);
        if (!k) continue;
        int to = nb[(int)(rnd() * k) % k];
        float dx, dz, off, rx, rz; int ax;
        edge_frame(from, to, &dx, &dz, &off, &ax);
        right_of(dx, dz, &rx, &rz);
        float len = fabsf(node_x(to) - node_x(from)) + fabsf(node_z(to) - node_z(from));
        float s = cross_half(from, ax) + 4.0f + rnd() * fmaxf(1.0f, len - cross_half(from, ax) - cross_half(to, ax) - 14.0f);
        float x = node_x(from) + dx * s + rx * off, z = node_z(from) + dz * s + rz * off;
        if (tries < 50) {
            float ddx = x - avoid.x, ddz = z - avoid.z;
            if (ddx * ddx + ddz * ddz < minDist * minDist) continue;
        }
        int clear = 1;
        for (int i = 0; i < w->ncars && clear; i++) {
            if (i == a->slot || !w->cars[i].active) continue;
            float ox = w->cars[i].pos.x - x, oz = w->cars[i].pos.z - z;
            clear = ox * ox + oz * oz > 14.0f * 14.0f;
        }
        if (!clear && tries < 55) continue;
        // Mostly yellow cabs in New York; a mix elsewhere; one in six is a pickup.
        int type = rnd() < 0.16f ? VEH_PICKUP : VEH_HATCH;
        V3 paint = (w->map == MAP_NEWYORK && rnd() < 0.6f) ? PAINT[0] : PAINT[1 + (int)(rnd() * 6) % 6];
        float yaw = atan2f(-dz, dx);                    // car_build: forward = (cos yaw, 0, -sin yaw)
        car_build_vehicle(&w->cars[a->slot], type, v3(x, terrain_height(x, z) + vehicle_def(type)->spawnH, z), yaw, paint);
        a->prev = from; a->cur = to; a->n = 0;
        a->lastX = x; a->lastZ = z;
        append_edge(a, from, to);
        extend(a);
        a->stuck = a->bad = 0;
        a->active = 1;
        return;
    }
    a->active = 0;
    w->cars[a->slot].active = 0;
}

void traffic_set_map(int map) { G = map == MAP_NEWYORK ? GRID_NEWYORK : map == MAP_AIRPORT ? GRID_AIRPORT : GRID_DEFAULT; }

void traffic_init(World *w, int count, V3 avoid) {
    traffic_set_map(w->map);
    memset(ai, 0, sizeof ai);
    aiBase = w->ncars; aiCount = 0; respawns = 0; memset(why, 0, sizeof why);
    for (int i = 0; i < count && w->ncars < MAX_CARS; i++) {
        Ai *a = &ai[aiCount++];
        a->slot = w->ncars++;
        spawn(w, a, avoid, 25.0f);
    }
}

void traffic_clear(World *w) {
    if (aiBase >= 0 && aiCount) {
        for (int i = 0; i < aiCount; i++) if (ai[i].active) w->cars[ai[i].slot].active = 0;
        if (w->ncars > aiBase) w->ncars = aiBase;
    }
    memset(ai, 0, sizeof ai);
    aiCount = 0; aiBase = -1;
}

void traffic_forget_slot(int slot) { for (int i = 0; i < aiCount; i++) if (ai[i].slot == slot) ai[i].active = 0; }
int traffic_respawns(void) { return respawns; }
void traffic_debug(int out[4]) { memcpy(out, why, sizeof why); }
int traffic_is_ai(int slot) { for (int i = 0; i < aiCount; i++) if (ai[i].active && ai[i].slot == slot) return 1; return 0; }

static void drive(World *w, Ai *a, float dt, V3 player) {
    Car *c = &w->cars[a->slot];
    if (!c->active || c->exploded) { respawns++; spawn(w, a, player, 60.0f); return; }
    float px = c->pos.x, pz = c->pos.z;
    float fl = sqrtf(c->fwd.x * c->fwd.x + c->fwd.z * c->fwd.z) + 1e-6f;
    float fx = c->fwd.x / fl, fz = c->fwd.z / fl, rx, rz;
    right_of(fx, fz, &rx, &rz);
    float v = vdot(c->vel, c->fwd);

    // Drop waypoints we've reached or passed, keep the path topped up.
    int drop = 0;
    while (a->n - drop > 1) {
        const Wp *q = &a->wp[drop];
        float dx = q->x - px, dz = q->z - pz, d = sqrtf(dx * dx + dz * dz), ahead = dx * fx + dz * fz;
        // A stop line stays until the light is green or we're actually past it, so its signal keeps applying.
        if (q->stopNode >= 0 && ahead > -0.5f && light(q->stopNode, q->axisZ) != LIGHT_GREEN) break;
        if (d < 2.5f || (ahead < 0 && d < 8.0f)) drop++; else break;
    }
    if (drop) {
        a->lastX = a->wp[drop - 1].x; a->lastZ = a->wp[drop - 1].z;
        memmove(a->wp, a->wp + drop, (a->n - drop) * sizeof(Wp)); a->n -= drop;
    }
    extend(a);

    // Pure pursuit: aim at the first waypoint beyond the look-ahead distance.
    float L = 4.0f + 0.5f * fabsf(v);
    Wp t = a->wp[a->n - 1];
    for (int k = 0; k < a->n; k++) {
        float dx = a->wp[k].x - px, dz = a->wp[k].z - pz;
        if (dx * dx + dz * dz >= L * L) { t = a->wp[k]; break; }
    }
    float tx = t.x - px, tz = t.z - pz, tl = sqrtf(tx * tx + tz * tz) + 1e-6f;
    tx /= tl; tz /= tl;
    float ang = atan2f(fz * tx - fx * tz, fx * tx + fz * tz);         // positive = target to the left
    float lock = c->def->maxLock / (1.0f + fabsf(v) / 22.0f);
    float steer = fmaxf(-1.0f, fminf(1.0f, ang / lock * 1.1f));

    // Target speed: cruise, slow for the turn ahead, stop for signals, keep the gap.
    float target = 11.0f;
    int waitingRed = 0;
    for (int k = 1; k < a->n; k++) {
        float dx = a->wp[k].x - px, dz = a->wp[k].z - pz;
        if (dx * dx + dz * dz > 22.0f * 22.0f) break;
        float sx = a->wp[k].x - a->wp[k - 1].x, sz = a->wp[k].z - a->wp[k - 1].z, sl = sqrtf(sx * sx + sz * sz) + 1e-6f;
        if ((sx * fx + sz * fz) / sl < 0.8f) { target = 5.0f; break; }
    }
    for (int k = 0; k < a->n; k++) {
        if (a->wp[k].stopNode < 0) continue;
        float d = (a->wp[k].x - px) * fx + (a->wp[k].z - pz) * fz;
        if (d > 45.0f) break;
        int st = light(a->wp[k].stopNode, a->wp[k].axisZ);
        if (d > -0.5f && st != LIGHT_GREEN) {
            int canStop = d > v * v / (2.0f * 5.0f) + 1.0f;       // yellow: go if we can't stop comfortably
            if (st == LIGHT_RED || canStop) {
                target = fminf(target, d < 1.0f ? 0.0f : sqrtf(2.0f * 3.0f * (d - 1.0f)));
                waitingRed = d < 30.0f;
            }
        }
        break;
    }
    for (int j = 0; j < w->ncars; j++) {
        if (j == a->slot || !w->cars[j].active) continue;
        float ox = w->cars[j].pos.x - px, oz = w->cars[j].pos.z - pz;
        float along = ox * fx + oz * fz, lat = fabsf(ox * rx + oz * rz);
        // Two stopped cars blocking each other across a junction (not one queue): lower slot goes first.
        Car *o = &w->cars[j];
        float ol = sqrtf(o->fwd.x * o->fwd.x + o->fwd.z * o->fwd.z) + 1e-6f;
        int sameWay = (o->fwd.x * fx + o->fwd.z * fz) / ol > 0.8f;
        if (!sameWay && fabsf(v) < 0.5f && vlen(o->vel) < 0.5f && a->slot < j) continue;
        if (along > 0 && along < 35.0f && lat < 2.4f) target = fminf(target, fmaxf(0.0f, along - 8.5f) * 0.6f);   // ~3 m bumper gap
    }

    float err = target - v;
    Controls ctl = {fmaxf(0.0f, fminf(1.0f, err * 0.35f)), fmaxf(0.0f, fminf(1.0f, -err * 0.45f)), steer, 0};
    if (target < 0.3f && fabsf(v) < 0.6f) { ctl.throttle = 0; ctl.brake = 1; }
    c->ctl = ctl;

    // Wrecked, flipped, shoved off the route or stuck: respawn out of sight.
    // Sideways distance from the current leg (last reached waypoint -> next one).
    float sx = a->wp[0].x - a->lastX, sz = a->wp[0].z - a->lastZ, sl2 = sx * sx + sz * sz + 1e-6f;
    float u = fmaxf(0.0f, fminf(1.0f, ((px - a->lastX) * sx + (pz - a->lastZ) * sz) / sl2));
    float ex = a->lastX + sx * u - px, ez = a->lastZ + sz * u - pz;
    int off = ex * ex + ez * ez > 8.0f * 8.0f;
    if (c->up.y < 0.45f || c->damage > 45.0f || off) a->bad += dt; else a->bad = 0;
    // Stalled (not moving, and not first in line at a red): blocked, queued, or deadlocked.
    if (fabsf(v) < 0.4f && !waitingRed) a->stuck += dt; else a->stuck = fmaxf(0.0f, a->stuck - 2.0f * dt);
    if (a->bad > 4.0f || a->stuck > 35.0f) {          // longer than a light cycle, so queues don't count
        why[a->stuck > 35.0f ? 0 : off ? 1 : c->up.y < 0.45f ? 2 : 3]++;
        respawns++; spawn(w, a, player, 60.0f);
    }
}

void traffic_update(World *w, float dt, V3 player) {
    timer += dt;
    for (int i = 0; i < aiCount; i++) if (ai[i].active) drive(w, &ai[i], dt, player);
}

// ----------------------------------------------------------------- signals --

int traffic_signal_count(void) {
    if (G.noSignals) return 0;
    int n = 0;
    for (int node = 0; node < G.nx * G.nz; node++) { int nb[4]; n += neighbours(node, nb); }
    return n;
}

void traffic_signal(int i, V3 *head, V3 *facing, V3 *pole, int *state) {
    for (int node = 0; node < G.nx * G.nz; node++) {
        int nb[4], k = neighbours(node, nb);
        if (i >= k) { i -= k; continue; }
        int from = nb[i];                                // approach from this neighbour
        float dx, dz, off, rx, rz; int ax;
        edge_frame(from, node, &dx, &dz, &off, &ax);
        right_of(dx, dz, &rx, &rz);
        float far = cross_half(node, ax) + 1.0f, side = own_half(node, ax) + 0.9f;
        float hx = node_x(node) + dx * far + rx * off, hz = node_z(node) + dz * far + rz * off;
        float qx = node_x(node) + dx * far + rx * side, qz = node_z(node) + dz * far + rz * side;
        *head = v3(hx, terrain_height(hx, hz) + 5.4f, hz);
        *pole = v3(qx, terrain_height(qx, qz), qz);
        *facing = v3(-dx, 0, -dz);
        *state = light(node, ax);
        return;
    }
    *state = LIGHT_RED;
}
