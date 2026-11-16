// BeamLite — sound synthesis. See audio.h. One stereo float stream; the device pulls audio_mix().
//
// Continuous voices follow targets the game sets each frame (smoothed here, 64 samples at a time, so
// nothing zips or clicks). One-shots come in through a small lock-free queue and play on a pool of
// voices. Nothing here allocates, locks or touches the game's data, so it is safe on the audio thread.
#include "audio.h"
#include <math.h>
#include <string.h>
#ifndef AUDIO_HEADLESS
#include "raylib.h"
#endif

#define SR ((float)AUD_RATE)
#define TAU 6.2831853f

static unsigned nz = 0x2545F491u;
static inline float nrand(void) { nz ^= nz << 13; nz ^= nz >> 17; nz ^= nz << 5; return (float)(int)nz * (1.0f / 2147483648.0f); }   // white, -1..1
static inline float urand(void) { return 0.5f + 0.5f * nrand(); }

// State-variable filter (Chamberlin). q is damping: smaller rings more.
typedef struct { float lo, bd; } Svf;
static inline float svf_f(float hz) { return 2.0f * sinf(3.14159265f * fminf(hz, 7500.0f) / SR); }
static inline float svf_bp(Svf *s, float in, float f, float q) { s->lo += f * s->bd; float hi = in - s->lo - q * s->bd; s->bd += f * hi; return s->bd; }
static inline float lp_a(float hz) { return 1.0f - expf(-TAU * hz / SR); }
static inline float lp1(float *z, float in, float a) { *z += a * (in - *z); return *z; }
static inline float wrap1(float p) { return p - floorf(p); }

// ---------------------------------------------------------------- listener ----
static float Lp[3], Lr[3] = {1, 0, 0};
void audio_listener(float px, float py, float pz, float rx, float ry, float rz) { Lp[0] = px; Lp[1] = py; Lp[2] = pz; Lr[0] = rx; Lr[1] = ry; Lr[2] = rz; }
void audio_spatial(float x, float y, float z, float ref, float *gl, float *gr) {
    float d[3] = {x - Lp[0], y - Lp[1], z - Lp[2]};
    float dist = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    float g = ref / fmaxf(ref, dist);                       // inverse distance beyond the reference
    g /= 1.0f + (dist / 500.0f) * (dist / 500.0f);          // and the air soaks up the rest
    float pan = dist > 0.5f ? (d[0] * Lr[0] + d[1] * Lr[1] + d[2] * Lr[2]) / dist : 0.0f;
    float a = (pan * 0.75f + 1.0f) * 0.7853982f;            // constant power, never hard left/right
    *gl = g * cosf(a) * 1.4142f; *gr = g * sinf(a) * 1.4142f;
}

// ---------------------------------------------------------------- one-shots ----
typedef struct { int snd; float gain, gl, gr; } Ev;
#define QN 64
static Ev queue[QN];
static volatile unsigned qw, qr;        // written by the game / by the mixer

#define NV 20
typedef struct {
    int on, snd;
    float t, dur, gain, gl, gr;
    float f[6], ph[7], d[6];            // partial frequencies, phases, decay rates or onsets
    Svf a, b;
    float z, click;
} Voice;
static Voice voice[NV];

static unsigned played[SND_COUNT];
static void push(int snd, float gain, float gl, float gr) {
    if (snd < 0 || snd >= SND_COUNT || gain <= 0.004f || (gl + gr) * gain < 0.004f) return;
    unsigned w = qw;
    if (w - qr >= QN) return;           // full: drop it
    queue[w % QN] = (Ev){snd, fminf(gain, 1.0f), gl, gr};
    qw = w + 1;
    played[snd]++;
}
unsigned audio_played(int snd) { return snd >= 0 && snd < SND_COUNT ? played[snd] : 0; }
void audio_play(int snd, float gain, float x, float y, float z) {
    static const float ref[SND_COUNT] = {14, 9, 60, 45, 18, 14, 22, 8, 22, 1};
    float gl, gr;
    audio_spatial(x, y, z, ref[snd < 0 || snd >= SND_COUNT ? 0 : snd], &gl, &gr);
    push(snd, gain, gl, gr);
}
void audio_play_ui(int snd, float gain) { push(snd, gain, 1.0f, 1.0f); }

static void voice_start(const Ev *e) {
    Voice *v = &voice[0];
    for (int i = 0; i < NV; i++) {      // a free voice, else the one nearest its end
        if (!voice[i].on) { v = &voice[i]; break; }
        if (voice[i].t / voice[i].dur > v->t / v->dur) v = &voice[i];
    }
    memset(v, 0, sizeof *v);
    v->on = 1; v->snd = e->snd; v->gain = e->gain; v->gl = e->gl; v->gr = e->gr;
    float g = e->gain;
    switch (e->snd) {
    case SND_CRASH:
        v->dur = 0.45f + 1.0f * g;
        v->f[0] = (900.0f - 500.0f * g) * (0.7f + 0.6f * urand());       // crunch band: bigger hits sit lower
        v->f[1] = 2200.0f + 2600.0f * urand();
        for (int k = 2; k < 5; k++) { v->f[k] = (350.0f + 2300.0f * urand()) * (1.2f - 0.5f * g); v->d[k] = 9.0f + 18.0f * urand(); }
        break;
    case SND_GLASS:
        v->dur = 1.1f;
        for (int k = 0; k < 6; k++) { v->f[k] = 2600.0f + 5200.0f * urand(); v->d[k] = 0.02f + 0.4f * urand() * urand(); }
        break;
    case SND_EXPLODE: v->dur = 3.2f; break;
    case SND_COLLAPSE: v->dur = 5.5f; v->f[0] = 500.0f + 500.0f * urand(); break;
    case SND_BREACH: v->dur = 0.8f; v->f[0] = 420.0f + 420.0f * urand(); break;
    case SND_TREE: v->dur = 1.3f; v->f[0] = 170.0f + 110.0f * urand(); break;
    case SND_SPLASH: v->dur = 1.4f; break;
    case SND_QUACK: v->dur = 0.38f; v->f[0] = 0.92f + 0.16f * urand(); break;
    case SND_ROTOR:
        v->dur = 1.1f;
        for (int k = 0; k < 5; k++) { v->f[k] = 650.0f + 3200.0f * urand(); v->d[k] = 5.0f + 10.0f * urand(); }
        break;
    case SND_CHIME: v->dur = 0.7f; break;
    }
}

// A burst of clicks thinning out: returns the click envelope (0..1).
static inline float crackle(Voice *v, float perSec, float dt, float keep) {
    if (urand() < perSec * dt) v->click = 1.0f;
    v->click *= keep;
    return v->click;
}

static float voice_sample(Voice *v, float dt) {
    float t = v->t, g = v->gain, n = nrand(), o = 0.0f;
    switch (v->snd) {
    case SND_CRASH: {                   // a low thump, crunching metal, a few ringing panels, bits settling
        v->ph[6] = wrap1(v->ph[6] + (42.0f + 75.0f * expf(-t * 18.0f)) * dt);
        o = sinf(TAU * v->ph[6]) * expf(-t * (11.0f - 5.0f * g)) * 0.9f;
        o += svf_bp(&v->a, n, svf_f(v->f[0]), 0.5f) * expf(-t * (9.0f - 4.0f * g)) * 1.1f;
        o += svf_bp(&v->b, n, svf_f(v->f[1]), 0.8f) * expf(-t * 16.0f) * 0.6f;
        float m = 0;
        for (int k = 2; k < 5; k++) { v->ph[k] = wrap1(v->ph[k] + v->f[k] * dt); m += sinf(TAU * v->ph[k]) * expf(-t * v->d[k]); }
        o += m * 0.11f + crackle(v, 90.0f * g * expf(-t * 5.0f), dt, 0.985f) * n * 0.45f;
        o *= 0.25f + 0.75f * g;
    } break;
    case SND_GLASS: {                   // the shatter, then pieces ringing as they land
        float burst = svf_bp(&v->a, n, svf_f(5200.0f), 1.2f) * expf(-t * 14.0f) * 1.1f + (n - lp1(&v->z, n, 0.35f)) * expf(-t * 60.0f) * 0.5f;
        float s = 0;
        for (int k = 0; k < 6; k++) {
            float tt = t - v->d[k];
            if (tt < 0) continue;
            v->ph[k] = wrap1(v->ph[k] + v->f[k] * dt);
            s += sinf(TAU * v->ph[k]) * expf(-tt * 24.0f);
        }
        o = (burst + s * 0.17f) * (0.4f + 0.6f * g);
    } break;
    case SND_EXPLODE: {                 // a crack, a boom sliding down, a long rumble
        v->ph[6] = wrap1(v->ph[6] + (27.0f + 62.0f * expf(-t * 6.0f)) * dt);
        o = n * expf(-t * 55.0f) * 0.8f + sinf(TAU * v->ph[6]) * expf(-t * 2.0f) * 1.0f;
        o += lp1(&v->z, n, lp_a(90.0f + 650.0f * expf(-t * 3.0f))) * 3.2f * expf(-t * 1.5f);
        o += crackle(v, 45.0f * expf(-t * 2.0f), dt, 0.985f) * n * 0.25f;
        o *= 0.5f + 0.5f * g;
    } break;
    case SND_COLLAPSE: {                // masonry coming down: a swelling rumble with lumps landing in it
        float env = (1.0f - expf(-t * 5.0f)) * expf(-t * 0.8f);
        v->ph[6] = wrap1(v->ph[6] + 36.0f * dt);
        o = lp1(&v->z, n, lp_a(260.0f)) * 4.2f * env + sinf(TAU * v->ph[6]) * 0.4f * env;
        o += svf_bp(&v->b, n * crackle(v, 60.0f * env, dt, 0.9985f), svf_f(v->f[0]), 0.6f) * 1.6f;
        o *= 0.55f + 0.45f * g;
    } break;
    case SND_BREACH: {                  // a wall punched in
        v->ph[6] = wrap1(v->ph[6] + (48.0f + 50.0f * expf(-t * 20.0f)) * dt);
        o = sinf(TAU * v->ph[6]) * expf(-t * 9.0f) + svf_bp(&v->a, n, svf_f(v->f[0]), 0.7f) * expf(-t * 8.0f) * 1.2f;
        o += crackle(v, 80.0f * expf(-t * 6.0f), dt, 0.985f) * n * 0.5f;
        o *= 0.3f + 0.7f * g;
    } break;
    case SND_TREE: {                    // two cracks, the trunk creaking over, leaves
        float t2 = t - 0.07f;
        o = n * expf(-t * 90.0f) + (t2 > 0 ? n * expf(-t2 * 70.0f) * 0.7f : 0.0f);
        v->ph[6] = wrap1(v->ph[6] + v->f[0] * expf(-t * 1.2f) * dt);
        o += svf_bp(&v->a, 2.0f * v->ph[6] - 1.0f, svf_f(900.0f), 0.3f) * expf(-t * 5.0f) * 0.35f;
        if (t > 0.15f) { float s = sinf(3.14159265f * (t - 0.15f) / 1.15f); o += svf_bp(&v->b, n, svf_f(3500.0f), 1.0f) * s * s * 0.28f; }
        o *= 0.4f + 0.6f * g;
    } break;
    case SND_SPLASH: {
        float env = (1.0f - expf(-t * 70.0f)) * expf(-t * 3.5f);
        o = svf_bp(&v->a, n, svf_f(500.0f + 1600.0f * expf(-t * 4.0f)), 0.9f) * env * 1.5f + lp1(&v->z, n, lp_a(150.0f)) * 3.0f * expf(-t * 6.0f);
        o *= 0.3f + 0.7f * g;
    } break;
    case SND_QUACK: {                   // two squeezed-rubber syllables through a nasal pair of formants
        float seg = t < 0.18f ? t : t - 0.19f, base = (t < 0.18f ? 640.0f : 520.0f) * v->f[0];
        if (seg >= 0 && seg < 0.16f) {
            v->ph[6] = wrap1(v->ph[6] + base * (1.15f - 1.6f * seg) * dt);
            float saw = 2.0f * v->ph[6] - 1.0f;
            o = (svf_bp(&v->a, saw, svf_f(1350.0f), 0.35f) + 0.5f * svf_bp(&v->b, saw, svf_f(2600.0f), 0.3f)) * sinf(3.14159265f * seg / 0.16f) * 0.5f;
        }
    } break;
    case SND_ROTOR: {                   // blades hitting something: clang, a whoosh of fragments, a thump
        float s = 0;
        for (int k = 0; k < 5; k++) { v->ph[k] = wrap1(v->ph[k] + v->f[k] * dt); s += sinf(TAU * v->ph[k]) * expf(-t * v->d[k]); }
        v->ph[6] = wrap1(v->ph[6] + (45.0f + 40.0f * expf(-t * 15.0f)) * dt);
        o = s * 0.17f + svf_bp(&v->a, n, svf_f(1200.0f + 2500.0f * expf(-t * 8.0f)), 0.8f) * expf(-t * 7.0f) * 0.8f + sinf(TAU * v->ph[6]) * expf(-t * 10.0f) * 0.6f;
    } break;
    case SND_CHIME: {
        float t2 = t - 0.12f;
        v->ph[0] = wrap1(v->ph[0] + 880.0f * dt); v->ph[1] = wrap1(v->ph[1] + 1318.5f * dt);
        o = sinf(TAU * v->ph[0]) * expf(-t * 7.0f) * 0.25f + (t2 > 0 ? sinf(TAU * v->ph[1]) * expf(-t2 * 6.0f) * 0.25f : 0.0f);
        o *= g;
    } break;
    }
    v->t += dt;
    if (v->t >= v->dur) v->on = 0;
    float fade = fminf(1.0f, (v->dur - t) * 25.0f);     // no click at the cut-off
    return o * fade;
}

// ---------------------------------------------------------------- continuous voices ----
static AudioParams tgt, cur;            // what the game asked for; where the synth has got to
static int started;
typedef struct { float ph, z, z2; Svf cl; } Eng;
static Eng eng[AUD_CARS];
static struct {
    float windZ, gustZ, roadZ, rainZ, fireZ, fireClick, dripClick;
    Svf skid1, skid2, scr; float scrRough, lfoZ;
    float rph, tph, turbPh, heliZ, heliHiss;
    float jn1, jph1, jph2, jetZ, jetHiss, jetRum;
} S;

void audio_set(const AudioParams *p) { tgt = *p; started = 1; }

static void smooth(unsigned n) {
    float k = 1.0f - expf(-(float)n / SR / 0.035f), kr = 1.0f - expf(-(float)n / SR / 0.07f);
    #define GO(f) cur.f += k * (tgt.f - cur.f)
    GO(master); GO(rate); GO(wind); GO(road); GO(skid); GO(rain); GO(scrape); GO(fire);
    GO(heliRpm); GO(heliLoad); GO(heliL); GO(heliR); GO(heliTail); GO(jetL); GO(jetR);
    #undef GO
    for (int i = 0; i < AUD_CARS; i++) {
        AudCar *c = &cur.car[i]; const AudCar *t = &tgt.car[i];
        if (t->id != c->id) {                               // another car takes this voice: fade out, swap, fade in
            c->gl += k * (0 - c->gl); c->gr += k * (0 - c->gr);
            if (c->gl + c->gr < 0.004f) { c->id = t->id; c->type = t->type; c->rev = t->rev; c->load = t->load; }
            continue;
        }
        c->type = t->type;
        c->rev += kr * (t->rev - c->rev); c->load += kr * (t->load - c->load);
        c->gl += k * (t->gl - c->gl); c->gr += k * (t->gr - c->gr);
    }
    // Jet engines take seconds to spool up, longer to wind down.
    float want = tgt.jetN1;
    S.jn1 += (1.0f - expf(-(float)n / SR / (want > S.jn1 ? 2.0f : 3.2f))) * (want - S.jn1);
}

static inline float engine_sample(Eng *e, const AudCar *c, float rate) {
    float rev = c->rev, load = c->load, n = nrand(), s = 0;
    if (c->type == ENG_ELECTRIC) {                          // motor and inverter whine; nearly silent at rest
        e->ph = wrap1(e->ph + (70.0f + 1500.0f * rev) * rate / SR);
        s = 0.5f * sinf(TAU * e->ph) + 0.22f * sinf(TAU * 2.0f * e->ph) + 0.1f * sinf(TAU * 3.0f * e->ph);
        return s * (0.05f + 0.5f * load) * (0.3f + 0.7f * rev) * 0.3f;
    }
    if (c->type == ENG_PETROL) {                            // four cylinders: firing at 2x crank speed
        float fc = (850.0f + 5900.0f * rev) / 120.0f * rate;    // engine cycles per second (two crank turns each)
        e->ph = wrap1(e->ph + fc / SR);
        float br = 0.48f + 0.42f * load + 0.1f * rev, a = 1.0f, x = TAU * e->ph;
        static const float A[6] = {1.0f, 0.55f, 0.38f, 0.26f, 0.17f, 0.10f};
        for (int k = 0; k < 6; k++) { s += A[k] * a * sinf(x * 4.0f * (k + 1)); a *= br; }
        s += 0.28f * sinf(x * 2.0f) + 0.2f * br * sinf(x * 6.0f) + 0.1f * br * br * sinf(x * 10.0f) + 0.08f * sinf(x);
        s += lp1(&e->z, n, lp_a(900.0f + 2500.0f * load)) * (0.5f + 0.5f * sinf(x * 4.0f)) * (0.25f + 0.9f * load);
        return s * (0.35f + 0.65f * load) * (0.7f + 0.5f * rev) * 0.2f;
    }
    // Diesel straight six: slow, lumpy, with injector clatter on every firing.
    float fc = (600.0f + 1500.0f * rev) / 120.0f * rate;
    e->ph = wrap1(e->ph + fc / SR);
    float br = 0.5f + 0.35f * load, a = 1.0f, x = TAU * e->ph;
    static const float A[5] = {1.0f, 0.6f, 0.4f, 0.25f, 0.15f};
    for (int k = 0; k < 5; k++) { s += A[k] * a * sinf(x * 6.0f * (k + 1)); a *= br; }
    s += 0.5f * sinf(x * 3.0f) + 0.35f * br * sinf(x * 9.0f) + 0.2f * br * sinf(x * 15.0f) + 0.15f * sinf(x) + 0.15f * sinf(x * 2.0f);
    float pulse = 0.5f + 0.5f * cosf(x * 6.0f); pulse *= pulse; pulse *= pulse; pulse *= pulse;
    s += svf_bp(&e->cl, n, svf_f(1900.0f), 0.7f) * pulse * (0.5f + 0.8f * load) * 1.2f;
    s += lp1(&e->z, n, lp_a(400.0f)) * (0.6f + 0.8f * load);
    return s * (0.4f + 0.6f * load) * (0.75f + 0.4f * rev) * 0.26f;
}

void audio_mix(float *out, unsigned frames) {
    if (!started) { memset(out, 0, sizeof(float) * 2 * frames); return; }
    for (unsigned i = 0; i < frames;) {
        unsigned nb = frames - i < 64 ? frames - i : 64;
        smooth(nb);
        while (qr != qw) { voice_start(&queue[qr % QN]); qr = qr + 1; }
        float rate = fmaxf(cur.rate, 0.05f), dt = rate / SR;
        float windA = lp_a(260.0f + 1000.0f * cur.wind), roadA = lp_a(170.0f + 330.0f * cur.road);
        float hr = cur.heliRpm, hgain = hr * sqrtf(hr), heliA = lp_a(450.0f + 550.0f * cur.heliLoad);
        float n1 = S.jn1, jetA = lp_a(170.0f + 1150.0f * n1), jroar = powf(n1, 2.2f), jon = n1 > 0.02f ? 1.0f : 0.0f;
        for (unsigned j = 0; j < nb; j++, i++) {
            float L = 0, R = 0, n = nrand();
            for (int c = 0; c < AUD_CARS; c++) {
                if (cur.car[c].id < 0 || cur.car[c].gl + cur.car[c].gr < 0.002f) continue;
                float s = engine_sample(&eng[c], &cur.car[c], rate);
                L += s * cur.car[c].gl; R += s * cur.car[c].gr;
            }
            // Ambience at the listener: wind with slow gusts, tyre roar and squeal, rain, scraping metal, fire.
            float amb = 0;
            float lfo = lp1(&S.lfoZ, nrand(), 0.00012f) * 60.0f;        // a slow wander, about -1..1
            if (cur.wind > 0.003f) amb += lp1(&S.windZ, n, windA) * cur.wind * (0.6f + 0.4f * cur.wind) * (1.0f + 0.35f * lfo) * 2.2f;
            if (cur.road > 0.003f) amb += lp1(&S.roadZ, nrand(), roadA) * cur.road * 2.4f;
            if (cur.skid > 0.003f) {
                float e = nrand();
                amb += (svf_bp(&S.skid1, e, svf_f(1050.0f + 180.0f * lfo), 0.07f) * 0.05f + svf_bp(&S.skid2, e, svf_f(1720.0f + 260.0f * lfo), 0.1f) * 0.035f) * cur.skid;
            }
            if (cur.rain > 0.003f) {
                float e = nrand();
                if (urand() < 30.0f / SR) S.dripClick = 1.0f;
                S.dripClick *= 0.97f;
                amb += ((e - lp1(&S.rainZ, e, 0.25f)) * 0.07f + S.dripClick * e * 0.05f) * cur.rain;
            }
            if (cur.scrape > 0.003f) {
                float rough = lp1(&S.scrRough, nrand(), 0.004f) * 9.0f;
                amb += svf_bp(&S.scr, nrand() * (0.55f + rough), svf_f(1100.0f + 500.0f * lfo), 0.3f) * cur.scrape * 0.5f;
            }
            if (cur.fire > 0.003f) {
                if (urand() < 22.0f / SR) S.fireClick = 0.4f + 0.6f * urand();
                S.fireClick *= 0.992f;
                float e = nrand();
                amb += (lp1(&S.fireZ, e, lp_a(520.0f)) * 1.6f * (1.0f + 0.4f * lfo) + S.fireClick * e * 0.5f) * cur.fire;
            }
            L += amb; R += amb;
            // Helicopter: blade slap at three per rotor turn, the turbine, the tail rotor's buzz.
            if (hgain > 0.002f) {
                S.rph = wrap1(S.rph + 3.0f * 6.6f * hr * dt);
                float x = TAU * S.rph, pulse = 0.5f + 0.5f * cosf(x);
                pulse *= pulse; pulse *= pulse; pulse *= pulse * pulse;      // ^12: a short slap
                float h = lp1(&S.heliZ, nrand(), heliA) * pulse * 4.5f * (0.45f + 0.75f * cur.heliLoad);
                h += (sinf(x) + 0.5f * sinf(2 * x) + 0.33f * sinf(3 * x) + 0.25f * sinf(4 * x) + 0.2f * sinf(5 * x) + 0.16f * sinf(6 * x)) * (0.22f + 0.2f * cur.heliLoad);
                S.turbPh = wrap1(S.turbPh + 6200.0f * hr * dt);
                float e = nrand();
                h += sinf(TAU * S.turbPh) * 0.035f * hr + (e - lp1(&S.heliHiss, e, 0.3f)) * 0.05f * hr;
                S.tph = wrap1(S.tph + 2.0f * 33.0f * hr * dt);
                float y = TAU * S.tph;
                h += (sinf(y) + 0.5f * sinf(2 * y) + 0.33f * sinf(3 * y) + 0.25f * sinf(4 * y) + 0.2f * sinf(5 * y)) * 0.1f * cur.heliTail;
                h *= hgain * 0.55f;
                L += h * cur.heliL; R += h * cur.heliR;
            }
            // Airliner: the roar of the exhaust, the fan's whine (two engines a hair apart), a hiss on top.
            if (jon > 0) {
                float e = nrand();
                float jet = lp1(&S.jetZ, e, jetA) * 4.0f * jroar + lp1(&S.jetRum, nrand(), lp_a(70.0f)) * 5.0f * jroar;
                float wf = (320.0f + 3300.0f * n1) * dt;
                S.jph1 = wrap1(S.jph1 + wf); S.jph2 = wrap1(S.jph2 + wf * 1.013f);
                jet += (sinf(TAU * S.jph1) + sinf(TAU * S.jph2)) * (0.012f + 0.03f * n1) * fminf(n1 * 8.0f, 1.0f);
                jet += (e - lp1(&S.jetHiss, e, 0.3f)) * 0.14f * n1 * n1;
                L += jet * cur.jetL; R += jet * cur.jetR;
            }
            for (int v = 0; v < NV; v++) {
                if (!voice[v].on) continue;
                float s = voice_sample(&voice[v], dt);
                L += s * voice[v].gl; R += s * voice[v].gr;
            }
            out[2 * i] = tanhf(L * cur.master); out[2 * i + 1] = tanhf(R * cur.master);
        }
    }
}

// ---------------------------------------------------------------- device ----
#ifndef AUDIO_HEADLESS
static AudioStream stream;
static int devOn;
static void stream_cb(void *buf, unsigned int frames) { audio_mix((float *)buf, frames); }
void audio_init(void) {
    InitAudioDevice();
    if (!IsAudioDeviceReady()) return;
    SetAudioStreamBufferSizeDefault(1024);
    stream = LoadAudioStream(AUD_RATE, 32, 2);
    SetAudioStreamCallback(stream, stream_cb);
    PlayAudioStream(stream);
    devOn = 1;
}
void audio_shutdown(void) {
    if (!devOn) return;
    StopAudioStream(stream);
    UnloadAudioStream(stream);
    CloseAudioDevice();
    devOn = 0;
}
#endif
