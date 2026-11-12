// Headless physics checks + wireframe snapshots (PPM) — not part of the game build.
#include "../src/physics.h"
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
    static const char *TG[] = {"latched", "open", "hanging", "gone"};
    printf("%-26s pos=(%7.1f %5.2f %6.1f) v=%6.1f km/h  up.y=%.2f  dmg=%5.1f%%  broken=%d  glass=%d  tg=%s  expl=%d\n",
           tag, c->pos.x, c->pos.y, c->pos.z, vdot(c->vel, c->fwd) * 3.6f, c->up.y, c->damage, c->brokenBeams,
           c->glassBrokenCount, TG[c->tailgate], c->exploded);
}

int main(void) {
    Controls idle = {0, 0, 0, 0}, gas = {1, 0, 0, 0}, brake = {0, 1, 0, 0};
    printf("nodes per truck / beams / tris: ");
    world_init(&W); W.ncars = 1;
    car_build(&W.cars[0], v3(0, terrain_height(0, 0) + CAR_SPAWN_H, 0), 0, v3(1, 0, 0));
    printf("%d / %d / %d\n", W.cars[0].nn, W.cars[0].nb, W.cars[0].nt);

    run(3, idle); report("settle 3s");
    float bottom = 1e9f; for (int i = 0; i < W.cars[0].nn; i++) bottom = fminf(bottom, W.cars[0].n[i].p.y);
    printf("  ride height (lowest node above ground) = %.3f m\n", bottom);

    // Acceleration and braking on flat ground, heading -x.
    world_init(&W); W.ncars = 1;
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
    world_init(&W); W.ncars = 1;
    car_build(&W.cars[0], v3(0, CAR_SPAWN_H, -60), 3.14159f, v3(1, 0, 0));
    run(1, idle);
    run(3.0f, gas);
    report("before turn");
    Controls turn = {0.4f, 0, 1.0f, 0};
    for (int i = 0; i < 4; i++) { run(1.0f, turn); report("  turning"); }

    // Tailgate drop while parked: should fold down level on its straps.
    world_init(&W); W.ncars = 1;
    car_build(&W.cars[0], v3(0, CAR_SPAWN_H, -60), 0, v3(1, 0, 0));
    run(1, idle); car_pop_tailgate(&W.cars[0]); run(3, idle); report("tailgate dropped");
    {
        Car *c = &W.cars[0];
        V3 tl = c->n[c->lightR[0]].p, bl = c->n[c->lightR[2]].p;
        printf("  tailgate top edge %.2f m behind / %.2f m above its hinge\n", -vdot(vsub(tl, bl), c->fwd), vdot(vsub(tl, bl), c->up));
    }

    // Wall crash at ~60 km/h
    world_init(&W); W.ncars = 1;
    car_build(&W.cars[0], v3(-30, CAR_SPAWN_H, 0), 3.14159f, v3(1, 0, 0));
    run(1, idle);
    while (vdot(W.cars[0].vel, W.cars[0].fwd) * 3.6f < 60 && W.cars[0].pos.x > -60) run(0.05f, gas);
    report("pre-impact");
    while (W.cars[0].pos.x > -67.5f && W.cars[0].pos.x < 0) run(0.05f, (Controls){0.3f, 0, 0, 0});
    run(2.0f, idle); report("after wall @60");
    snapshot("/tmp/crash_wall.ppm", &W.cars[0]);

    // Ramp jump
    world_init(&W); W.ncars = 1;
    car_build(&W.cars[0], v3(-40, CAR_SPAWN_H, 0), 0, v3(1, 0, 0));
    run(1, idle);
    float maxY = 0;
    for (int i = 0; i < 200; i++) { run(0.05f, gas); maxY = fmaxf(maxY, W.cars[0].pos.y); if (W.cars[0].pos.x > 140) break; }
    report("after ramp"); printf("  peak height %.1f m\n", maxY);
    run(3, brake); report("  stopped");

    // T-bone another car
    world_init(&W); W.ncars = 2;
    car_build(&W.cars[0], v3(-30, CAR_SPAWN_H, -50), 0, v3(1, 0, 0));
    car_build(&W.cars[1], v3(0, CAR_SPAWN_H, -50), 3.14159f / 2, v3(0, 0, 1));
    W.cars[1].ctl = (Controls){0, 0.5f, 0, 1};
    run(1, idle);
    for (int i = 0; i < 80; i++) run(0.05f, gas);
    run(2, idle);
    report("T-bone striker");
    printf("  victim: pos=(%.1f %.2f %.1f) dmg=%.1f%% broken=%d expl=%d\n", W.cars[1].pos.x, W.cars[1].pos.y, W.cars[1].pos.z, W.cars[1].damage, W.cars[1].brokenBeams, W.cars[1].exploded);
    snapshot("/tmp/tbone_victim.ppm", &W.cars[1]);
    snapshot("/tmp/tbone_striker.ppm", &W.cars[0]);
    return 0;
}
