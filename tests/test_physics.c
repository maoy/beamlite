// Headless physics checks + wireframe snapshots (PPM) — not part of the game build.
#include "../src/physics.h"
#include "../src/traffic.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static World W;
static const float H = 1.0f / SIM_HZ;

static void run(float secs, Controls ctl) {
    int steps = (int)(secs * SIM_HZ);
    for (int i = 0; i < steps; i++) { W.cars[0].ctl = ctl; world_substep(&W, H); }
    for (int c = 0; c < W.ncars; c++) { car_update_frame(&W.cars[c]); car_update_damage(&W.cars[c]); }
}

#define IW 900
#define IH 520
static unsigned char img[IH][IW][3];
static void px(int x, int y, int r, int g, int b) { if (x >= 0 && y >= 0 && x < IW && y < IH) { img[y][x][0] = r; img[y][x][1] = g; img[y][x][2] = b; } }
static void line(int x0, int y0, int x1, int y1, int r, int g, int b) {
    int dx = abs(x1 - x0), dy = -abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, e = dx + dy;
    for (int n = 0; n < 4000; n++) { px(x0, y0, r, g, b); if (x0 == x1 && y0 == y1) break; int e2 = 2 * e; if (e2 >= dy) { e += dy; x0 += sx; } if (e2 <= dx) { e += dx; y0 += sy; } }
}
// Two views of car c: side (car-local x/y) on top, plan (x/z) below.
static void snapshot(const char *path, Car *c) {
    memset(img, 30, sizeof img);
    car_update_frame(c);
    for (int view = 0; view < 2; view++) {
        int oy = view == 0 ? 150 : 390;
        for (int i = 0; i < c->nb; i++) {
            Beam *B = &c->b[i];
            if (B->broken) continue;
            V3 pa = vsub(c->n[B->a].p, c->pos), pb = vsub(c->n[B->b].p, c->pos);
            float ax = vdot(pa, c->fwd), bx = vdot(pb, c->fwd);
            float ay = view == 0 ? vdot(pa, c->up) : vdot(pa, c->right);
            float by = view == 0 ? vdot(pb, c->up) : vdot(pb, c->right);
            float d = fabsf(B->rest - B->rest0) / B->rest0;
            int r = (int)fminf(255, 80 + d * 2000), g = (int)fmaxf(40, 220 - d * 2000);
            line(IW / 2 + (int)(ax * 110), oy - (int)(ay * 110), IW / 2 + (int)(bx * 110), oy - (int)(by * 110), r, g, 60);
        }
    }
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); return; }
    fprintf(f, "P6 %d %d 255\n", IW, IH);
    fwrite(img, 1, sizeof img, f);
    fclose(f);
}

static void report(const char *tag) {
    Car *c = &W.cars[0];
    static const char *TG[] = {"latched", "open", "hanging", "gone", "none"};
    printf("%-26s pos=(%7.1f %5.2f %6.1f) v=%6.1f km/h  up.y=%.2f  dmg=%5.1f%%  broken=%d  glass=%d  tg=%s  expl=%d\n",
           tag, c->pos.x, c->pos.y, c->pos.z, vdot(c->vel, c->fwd) * 3.6f, c->up.y, c->damage, c->brokenBeams,
           c->glassBrokenCount, TG[c->tailgate], c->exploded);
}

int main(void) {
    Controls idle = {0, 0, 0, 0}, gas = {1, 0, 0, 0}, brake = {0, 1, 0, 0};
    printf("nodes per truck / beams / tris: ");
    world_init(&W, MAP_DEFAULT); W.ncars = 1;
    car_build(&W.cars[0], v3(0, terrain_height(0, 0) + CAR_SPAWN_H, 0), 0, v3(1, 0, 0));
    printf("%d / %d / %d\n", W.cars[0].nn, W.cars[0].nb, W.cars[0].nt);

    run(3, idle); report("settle 3s");
    float bottom = 1e9f; for (int i = 0; i < W.cars[0].nn; i++) bottom = fminf(bottom, W.cars[0].n[i].p.y);
    printf("  ride height (lowest node above ground) = %.3f m\n", bottom);

    // Acceleration and braking on flat ground, heading -x.
    world_init(&W, MAP_DEFAULT); W.ncars = 1;
    car_build(&W.cars[0], v3(60, CAR_SPAWN_H, -60), 3.14159f, v3(1, 0, 0)); // facing -x on flat ground
    run(1, idle);
    float t = 0; int hit100 = 0;
    for (int i = 0; i < 100; i++) {
        run(0.1f, gas); t += 0.1f;
        if (!hit100 && vdot(W.cars[0].vel, W.cars[0].fwd) * 3.6f >= 100) { printf("  0-100 km/h: %.1f s\n", t); hit100 = 1; break; }
    }
    report("after accel");
    run(4, brake); report("after 4s braking");

    // Cornering
    world_init(&W, MAP_DEFAULT); W.ncars = 1;
    car_build(&W.cars[0], v3(0, CAR_SPAWN_H, -60), 3.14159f, v3(1, 0, 0));
    run(1, idle);
    run(3.0f, gas);
    report("before turn");
    Controls turn = {0.4f, 0, 1.0f, 0};
    for (int i = 0; i < 4; i++) { run(1.0f, turn); report("  turning"); }

    // Tailgate drop while parked: should fold down level on its straps.
    world_init(&W, MAP_DEFAULT); W.ncars = 1;
    car_build(&W.cars[0], v3(0, CAR_SPAWN_H, -60), 0, v3(1, 0, 0));
    run(1, idle); car_pop_tailgate(&W.cars[0]); run(3, idle); report("tailgate dropped");
    {
        Car *c = &W.cars[0];
        V3 tl = c->n[c->tgNode[0]].p, bl = c->n[c->tgNode[2]].p;
        printf("  tailgate top edge %.2f m behind / %.2f m above its hinge\n", -vdot(vsub(tl, bl), c->fwd), vdot(vsub(tl, bl), c->up));
    }

    // Wall crash at ~60 km/h
    world_init(&W, MAP_DEFAULT); W.ncars = 1;
    car_build(&W.cars[0], v3(-30, CAR_SPAWN_H, 0), 3.14159f, v3(1, 0, 0));
    run(1, idle);
    while (vdot(W.cars[0].vel, W.cars[0].fwd) * 3.6f < 60 && W.cars[0].pos.x > -60) run(0.05f, gas);
    report("pre-impact");
    while (W.cars[0].pos.x > -67.5f && W.cars[0].pos.x < 0) run(0.05f, (Controls){0.3f, 0, 0, 0});
    run(2.0f, idle); report("after wall @60");
    snapshot("/tmp/crash_wall.ppm", &W.cars[0]);

    // Ramp jump
    world_init(&W, MAP_DEFAULT); W.ncars = 1;
    car_build(&W.cars[0], v3(-40, CAR_SPAWN_H, 0), 0, v3(1, 0, 0));
    run(1, idle);
    float maxY = 0;
    for (int i = 0; i < 200; i++) { run(0.05f, gas); maxY = fmaxf(maxY, W.cars[0].pos.y); if (W.cars[0].pos.x > 140) break; }
    report("after ramp"); printf("  peak height %.1f m\n", maxY);
    run(3, brake); report("  stopped");

    // T-bone another car
    world_init(&W, MAP_DEFAULT); W.ncars = 2;
    car_build(&W.cars[0], v3(-30, CAR_SPAWN_H, -50), 0, v3(1, 0, 0));
    car_build(&W.cars[1], v3(0, CAR_SPAWN_H, -50), 3.14159f / 2, v3(0, 0, 1));
    W.cars[1].ctl = (Controls){0, 0.5f, 0, 1};
    run(1, idle);
    for (int i = 0; i < 80; i++) run(0.05f, gas);
    run(2, idle);
    report("T-bone striker");
    printf("  victim: pos=(%.1f %.2f %.1f) dmg=%.1f%% broken=%d expl=%d\n", W.cars[1].pos.x, W.cars[1].pos.y, W.cars[1].pos.z, W.cars[1].damage, W.cars[1].brokenBeams, W.cars[1].exploded);
    snapshot("/tmp/tbone_victim.ppm", &W.cars[1]);

    // Trees: a sapling snaps and the truck drives on; a mature trunk stops it.
    for (int big = 0; big < 2; big++) {
        world_init(&W, MAP_DEFAULT); W.ncars = 1;
        int ti = world_add_tree(&W, 0, -60, big ? 14.0f : 4.5f, big ? TREE_CONIFER : TREE_BROADLEAF);
        world_build_grid(&W);
        car_build(&W.cars[0], v3(-45, CAR_SPAWN_H, -60), 0, v3(1, 0, 0));
        run(1, idle);
        while (W.cars[0].pos.x < -12) run(0.05f, gas);
        report(big ? "pre big tree" : "pre sapling");
        run(3, idle);
        report(big ? "  after big tree" : "  after sapling");
        printf("  tree r=%.2f m broken=%d tilt=%.0f deg\n", W.tree[ti].r, W.tree[ti].broken, W.tree[ti].tilt * 57.3f);
    }
    {   // The Sparrow hatchback: light, front-wheel drive, softer structure.
        const VehicleDef *hd = vehicle_def(VEH_HATCH);
        world_init(&W, MAP_DEFAULT); W.ncars = 1;
        car_build_vehicle(&W.cars[0], VEH_HATCH, v3(0, hd->spawnH, -60), 0, v3(1, 0, 0));
        printf("hatch nodes / beams / tris: %d / %d / %d\n", W.cars[0].nn, W.cars[0].nb, W.cars[0].nt);
        run(3, idle); report("hatch settle 3s");
        float lo = 1e9f; for (int i = 0; i < W.cars[0].nn; i++) lo = fminf(lo, W.cars[0].n[i].p.y);
        printf("  ride height (lowest node above ground) = %.3f m\n", lo);
        world_init(&W, MAP_DEFAULT); W.ncars = 1;
        car_build_vehicle(&W.cars[0], VEH_HATCH, v3(60, hd->spawnH, -60), 3.14159f, v3(1, 0, 0));
        run(1, idle);
        float tt = 0;
        for (int i = 0; i < 300; i++) { run(0.1f, gas); tt += 0.1f; if (vdot(W.cars[0].vel, W.cars[0].fwd) * 3.6f >= 100) break; }
        printf("  hatch 0-100 km/h: %.1f s\n", tt);
        // Braking from 100 km/h on the flat (the acceleration run ends up in the hills).
        world_init(&W, MAP_DEFAULT); W.ncars = 1;
        car_build_vehicle(&W.cars[0], VEH_HATCH, v3(60, hd->spawnH, -60), 3.14159f, v3(1, 0, 0));
        run(1, idle);
        for (int i = 0; i < W.cars[0].nn; i++) W.cars[0].n[i].v = vadd(W.cars[0].n[i].v, vscale(W.cars[0].fwd, 100 / 3.6f));
        car_update_frame(&W.cars[0]);
        float x0 = W.cars[0].pos.x;
        while (vdot(W.cars[0].vel, W.cars[0].fwd) > 0.3f) run(0.05f, brake);
        printf("  hatch 100-0 braking distance: %.1f m\n", fabsf(W.cars[0].pos.x - x0));
        report("  hatch stopped");
        // Same stop on snow (grip 0.5).
        world_init(&W, MAP_DEFAULT); W.ncars = 1;
        car_build_vehicle(&W.cars[0], VEH_HATCH, v3(60, hd->spawnH, -60), 3.14159f, v3(1, 0, 0));
        W.cars[0].grip = 0.5f;
        run(1, idle);
        for (int i = 0; i < W.cars[0].nn; i++) W.cars[0].n[i].v = vadd(W.cars[0].n[i].v, vscale(W.cars[0].fwd, 100 / 3.6f));
        car_update_frame(&W.cars[0]);
        x0 = W.cars[0].pos.x;
        while (vdot(W.cars[0].vel, W.cars[0].fwd) > 0.3f) run(0.05f, brake);
        printf("  hatch 100-0 on snow: %.1f m\n", fabsf(W.cars[0].pos.x - x0));
        world_init(&W, MAP_DEFAULT); W.ncars = 1;
        car_build_vehicle(&W.cars[0], VEH_HATCH, v3(0, hd->spawnH, -60), 3.14159f, v3(1, 0, 0));
        run(1, idle); run(4.0f, gas); report("hatch before turn");
        for (int i = 0; i < 3; i++) { run(1.0f, (Controls){0.4f, 0, 1.0f, 0}); report("  hatch turning"); }
        world_init(&W, MAP_DEFAULT); W.ncars = 1;
        car_build_vehicle(&W.cars[0], VEH_HATCH, v3(-30, hd->spawnH, 0), 3.14159f, v3(1, 0, 0));
        run(1, idle);
        while (vdot(W.cars[0].vel, W.cars[0].fwd) * 3.6f < 50 && W.cars[0].pos.x > -64) run(0.05f, gas);
        report("hatch pre-impact");
        while (W.cars[0].pos.x > -68.0f && W.cars[0].pos.x < 0) run(0.05f, (Controls){0.3f, 0, 0, 0});
        run(2.0f, idle); report("hatch after wall @50");
        snapshot("/tmp/hatch_wall.ppm", &W.cars[0]);
    }
    // Buildings: hard hits breach walls and add up; past its strength a building collapses into a rubble mound.
    {
        const int kinds[3] = {BLD_HOUSE, BLD_WAREHOUSE, BLD_APARTMENT}, vehs[3] = {VEH_HATCH, VEH_SEMI, VEH_SEMI};
        const float kmh[3] = {140, 90, 100};
        const char *names[3] = {"hatch 140 km/h into a house", "semi 90 km/h into a warehouse", "semi 100 km/h into an apartment block"};
        for (int t = 0; t < 3; t++) {
            world_init(&W, MAP_DEFAULT); W.ncars = 1;
            int done = 0;
            for (int b = 0; b < W.nbld && !done; b++) {
                Building *B = &W.bld[b];
                if (B->kind != kinds[t] || B->mx.z - B->mn.z < 4) continue;
                for (int sd = 0; sd < 2 && !done; sd++) {
                    float nx = sd ? -1.0f : 1.0f, x = sd ? B->mn.x - 7.0f : B->mx.x + 7.0f, z = 0.5f * (B->mn.z + B->mx.z);
                    if (world_point_solid(&W, v3(x, terrain_height(x, z) + 1, z)) || world_point_solid(&W, v3(x - nx * 3.5f, terrain_height(x, z) + 1, z))) continue;
                    car_build_vehicle(&W.cars[0], vehs[t], v3(x, terrain_height(x, z) + vehicle_def(vehs[t])->spawnH, z), nx > 0 ? 3.14159f : 0.0f, v3(1, 0, 0));
                    for (int i = 0; i < W.cars[0].nn; i++) W.cars[0].n[i].v = v3(-nx * kmh[t] / 3.6f, 0, 0);
                    run(4.0f, idle);
                    float cx = 0.5f * (B->mn.x + B->mx.x), cz = 0.5f * (B->mn.z + B->mx.z);
                    printf("%-38s damage %3.0f%% of %6.0f N s  %s  breaches %d  rubble %.1f m  car %4.1f km/h  expl=%d\n", names[t], 100 * B->damage / B->strength, B->strength,
                           B->collapsed ? "collapsed" : "standing ", W.nbev - B->collapsed, building_mound(&W, b, cx, cz), vlen(W.cars[0].vel) * 3.6f, W.cars[0].exploded);
                    done = 1;
                }
            }
        }
    }
    for (int m = 0; m < MAP_COUNT; m++) {   // Traffic: 8 AI cars drive the junction grid for 90 s.
        world_init(&W, m); W.ncars = 1;
        car_build(&W.cars[0], v3(-30, CAR_SPAWN_H, -60), 0, v3(1, 0, 0));    // player parked out of the way
        W.cars[0].ctl = (Controls){0, 1, 0, 1};
        traffic_init(&W, 8, W.cars[0].pos);
        float dist[MAX_CARS] = {0}, stopped = 0, samples = 0;
        V3 last[MAX_CARS];
        for (int i = 0; i < W.ncars; i++) last[i] = W.cars[i].pos;
        for (int step = 0; step < 900; step++) {             // 0.1 s steps
            traffic_update(&W, 0.1f, W.cars[0].pos);
            int n = (int)(0.1f * SIM_HZ);
            for (int k = 0; k < n; k++) world_substep(&W, H);
            for (int i = 1; i < W.ncars; i++) {
                car_update_frame(&W.cars[i]); car_update_damage(&W.cars[i]);
                float d = vlen(vsub(W.cars[i].pos, last[i]));
                if (d < 20) dist[i] += d;                        // ignore respawn jumps
                last[i] = W.cars[i].pos;
                if (traffic_is_ai(i)) { samples++; if (vlen(W.cars[i].vel) < 0.5f) stopped++; }
            }
        }
        float total = 0; int moving = 0, wrecked = 0;
        for (int i = 1; i < W.ncars; i++) { total += dist[i]; if (dist[i] > 100) moving++; if (W.cars[i].damage > 20) wrecked++; }
        printf("traffic %-9s cars %d  moving %d  avg %.0f m in 90 s (%.0f km/h)  stopped %.0f%% of the time  respawns %d  damaged now %d\n",
               map_name(m), W.ncars - 1, moving, total / (W.ncars - 1), total / (W.ncars - 1) / 90 * 3.6f, 100 * stopped / samples, traffic_respawns(), wrecked);
        int why[4]; traffic_debug(why);
        printf("        respawn reasons: stuck %d, off route %d, flipped %d, damaged %d\n", why[0], why[1], why[2], why[3]);
        traffic_clear(&W);
    }
    {   // The long-nose semi: six wheels (tandem drive), 8.2 t.
        const VehicleDef *D = vehicle_def(VEH_SEMI);
        world_init(&W, MAP_DEFAULT); W.ncars = 1;
        car_build_vehicle(&W.cars[0], VEH_SEMI, v3(-60, terrain_height(-60, 3.5f) + D->spawnH, 3.5f), 0, D->paint);
        printf("semi nodes / beams / tris / wheels: %d / %d / %d / %d\n", W.cars[0].nn, W.cars[0].nb, W.cars[0].nt, W.cars[0].nw);
        run(3, idle); report("semi settle 3s");
        for (int i = 0; i < W.cars[0].nn; i++) W.cars[0].n[i].v = vadd(W.cars[0].n[i].v, v3(80 / 3.6f, 0, 0));
        car_update_frame(&W.cars[0]);
        float x0 = W.cars[0].pos.x; int k = 0;
        for (; k < 100 && vlen(W.cars[0].vel) > 0.3f; k++) run(0.1f, (Controls){0, 1, 0, 0});
        printf("  semi 80-0 braking distance: %.1f m, damage %.1f\n", W.cars[0].pos.x - x0, W.cars[0].damage);
        world_init(&W, MAP_DEFAULT); W.ncars = 1;
        car_build_vehicle(&W.cars[0], VEH_SEMI, v3(-50, terrain_height(-50, 0) + D->spawnH, 0), 3.14159f, D->paint);
        run(0.5f, idle);
        for (int i = 0; i < W.cars[0].nn; i++) W.cars[0].n[i].v = vadd(W.cars[0].n[i].v, v3(-60 / 3.6f, 0, 0));
        run(3, idle);
        printf("  semi into the wall at 60 km/h: damage %.0f%%, torn beams %d, glass %d/%d, exploded %d\n", W.cars[0].damage, W.cars[0].brokenBeams, W.cars[0].glassBrokenCount, W.cars[0].npanels, W.cars[0].exploded);
    }
    {   // Airport: the parked airliners are soft bodies. Settled on their gear, they sleep; a pickup at
        // 100 km/h into an engine wakes one, stops dead, and dents and bends it without blowing up.
        world_init(&W, MAP_AIRPORT);
        Airframe *A = &W.air[1];
        float g = terrain_height(W.planes[1].x, W.planes[1].z);
        printf("airliner: %d nodes, %d beams; settled nose %.2f m, tail %.2f m up; asleep %d\n", A->c.nn, A->c.nb,
               A->c.n[A->noseNode].p.y - g, A->c.n[A->tailNode].p.y - g, A->asleep);
        V3 e0 = A->c.n[A->eng[0][0][0]].p;
        W.ncars = 1;
        car_build_vehicle(&W.cars[0], VEH_PICKUP, v3(e0.x - 25, terrain_height(e0.x - 25, e0.z + 2) + CAR_SPAWN_H, e0.z + 2), 0, v3(.8f, .8f, .8f));
        for (int i = 0; i < W.cars[0].nn; i++) W.cars[0].n[i].v = v3(100 / 3.6f, 0, 0);
        car_update_frame(&W.cars[0]);
        run(4.0f, (Controls){0, 0, 0, 0});
        car_update_frame(&W.cars[0]); car_update_damage(&W.cars[0]);
        printf("airliner crash 100 km/h into an engine: truck %.1f km/h after, damage %.0f%%; airliner woke %d, deformed beams %d, engine pushed %.2f m, exploded %d\n",
               vlen(W.cars[0].vel) * 3.6f, W.cars[0].damage, !A->asleep, airframe_damaged_beams(A), vlen(vsub(A->c.n[A->eng[0][0][0]].p, e0)), A->c.exploded);
    }
    {   // Flying the airliner on the runway: full throttle, take-off flaps, rotate at 62 m/s, climb out;
        // then a steep dive into the bay, which should wreck it without the simulation blowing up.
        world_init(&W, MAP_AIRPORT); W.ncars = 0;
        Airframe *A = &W.air[W.flyable];
        A->pilot = 1; A->thr = 1; A->flaps = 0.5f;
        float x0 = A->pos.x, liftoff = -1, vmax = 0;
        int t = 0;
        for (; t < (int)(40 * SIM_HZ); t++) {
            float pitch = asinf(A->fwd.y);
            A->pitch = A->airspeed > 62 ? fminf(fmaxf((0.14f - pitch) * 6.0f, -1), 1) : 0;
            world_substep(&W, H);
            if (liftoff < 0 && A->agl > 2.0f) liftoff = A->pos.x - x0;
            if (A->airspeed > vmax) vmax = A->airspeed;
        }
        printf("airliner flight: lift-off after %.0f m; 40 s in: %.0f m up, %.0f m/s, climbing %.1f m/s, bent beams %d, wrecked %d\n",
               liftoff, A->pos.y, A->airspeed, A->vel.y, airframe_damaged_beams(A), A->wrecked);
        for (t = 0; t < (int)(25 * SIM_HZ) && !A->c.exploded; t++) {
            float pitch = asinf(A->fwd.y);
            A->thr = 0.3f; A->pitch = fminf(fmaxf((-0.9f - pitch) * 6.0f, -1), 1);
            world_substep(&W, H);
        }
        printf("airliner dive into the bay: wrecked %d, snapped beams %d, speed now %.1f m/s, lowest point %.1f m, exploded %d\n",
               A->wrecked, A->nbroken, A->airspeed, A->c.bmin.y, A->c.exploded);
    }
    {   // The generated map: buildings and trees exist and the test routes above are clear of them.
        world_init(&W, MAP_DEFAULT);
        printf("map: %d buildings, %d trees\n", W.nbld, W.ntree);
    }
    snapshot("/tmp/tbone_striker.ppm", &W.cars[0]);
    return 0;
}
