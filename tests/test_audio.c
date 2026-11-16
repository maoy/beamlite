// Headless sound check: renders each voice through the synthesiser and reports its level, and writes
// the lot to a WAV to listen to. Build: cc -O2 -DAUDIO_HEADLESS tests/test_audio.c src/audio.c -lm -o ta && ./ta [out.wav]
#include "../src/audio.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static float *all; static size_t nall, capall;
static int bad;
static int oneShot;
static void render(const char *name, float secs, float lo, float hi) {
    unsigned n = (unsigned)(secs * AUD_RATE);
    if (nall + 2 * n > capall) { capall = (nall + 2 * n) * 2; all = realloc(all, capall * sizeof(float)); }
    float *b = all + nall;
    for (unsigned i = 0; i < n; i += 512) audio_mix(b + 2 * i, n - i < 512 ? n - i : 512);
    double ss = 0; float pk = 0; int nan = 0;
    unsigned skip = !oneShot && n > AUD_RATE ? AUD_RATE / 2 : 0;    // let continuous voices settle
    for (unsigned i = 2 * skip; i < 2 * n; i++) { if (b[i] != b[i]) nan = 1; ss += b[i] * b[i]; if (fabsf(b[i]) > pk) pk = fabsf(b[i]); }
    float rms = sqrtf((float)(ss / (2 * (n - skip))));
    int ok = !nan && rms >= lo && rms <= hi && pk <= 1.0f;
    if (!ok) bad++;
    printf("%-34s rms %.3f  peak %.2f  %s\n", name, rms, pk, ok ? "ok" : nan ? "NaN" : "OUT OF RANGE");
    nall += 2 * n;
}
static AudioParams base(void) {
    AudioParams p; memset(&p, 0, sizeof p);
    p.master = 0.8f; p.rate = 1.0f; p.heliTail = 1.0f;
    for (int i = 0; i < AUD_CARS; i++) p.car[i].id = -1;
    return p;
}
static void car(const char *name, int type, float rev, float load) {
    AudioParams p = base();
    p.car[0] = (AudCar){0, type, rev, load, 1, 1};
    audio_set(&p); render(name, 1.5f, 0.01f, 0.45f);
}
int main(int argc, char **argv) {
    audio_listener(0, 0, 0, 1, 0, 0);
    AudioParams p = base();
    audio_set(&p); render("silence", 0.5f, 0.0f, 0.0001f);
    car("petrol idle", ENG_PETROL, 0.0f, 0.0f); car("petrol mid, half throttle", ENG_PETROL, 0.5f, 0.5f); car("petrol flat out", ENG_PETROL, 1.0f, 1.0f);
    car("diesel idle", ENG_DIESEL, 0.0f, 0.0f); car("diesel pulling", ENG_DIESEL, 0.7f, 1.0f);
    car("electric cruising", ENG_ELECTRIC, 0.4f, 0.3f); car("electric flat out", ENG_ELECTRIC, 1.0f, 1.0f);
    p = base(); p.wind = 1; audio_set(&p); render("wind, full", 1.5f, 0.02f, 0.4f);
    p = base(); p.road = 1; audio_set(&p); render("tyre roar, full", 1.5f, 0.02f, 0.4f);
    p = base(); p.skid = 1; audio_set(&p); render("tyre squeal, full", 1.5f, 0.02f, 0.4f);
    p = base(); p.rain = 1; audio_set(&p); render("rain", 1.5f, 0.01f, 0.2f);
    p = base(); p.scrape = 1; audio_set(&p); render("scraping metal", 1.5f, 0.02f, 0.45f);
    p = base(); p.fire = 1; audio_set(&p); render("fire", 1.5f, 0.02f, 0.3f);
    p = base(); p.heliRpm = 1; p.heliLoad = 0.2f; p.heliL = p.heliR = 1; audio_set(&p); render("helicopter, flat pitch", 2.0f, 0.03f, 0.45f);
    p.heliLoad = 0.6f; audio_set(&p); render("helicopter, hover", 2.0f, 0.05f, 0.5f);
    p = base(); p.jetN1 = 0.22f; p.jetL = p.jetR = 1; audio_set(&p); render("jet spooling to idle", 6.0f, 0.005f, 0.2f);
    p.jetN1 = 1.0f; audio_set(&p); render("jet spooling to takeoff", 8.0f, 0.08f, 0.6f);
    p = base(); audio_set(&p); render("(jet winding down)", 8.0f, 0.0f, 0.6f);
    static const char *nm[SND_COUNT] = {"crash", "glass", "explosion", "building collapse", "wall breach", "tree snap", "splash", "duck", "rotor strike", "chime"};
    static const float len[SND_COUNT] = {1.6f, 1.2f, 3.3f, 5.6f, 0.9f, 1.4f, 1.5f, 0.5f, 1.2f, 0.8f};
    oneShot = 1;
    for (int s = 0; s < SND_COUNT; s++) {
        audio_play_ui(s, 1.0f); render(nm[s], len[s], 0.02f, 0.6f);
        if (s == SND_CRASH) { audio_play_ui(s, 0.25f); render("crash, light", 0.8f, 0.005f, 0.4f); }
    }
    // Everything at once must still stay inside the limiter.
    p = base(); p.car[0] = (AudCar){0, ENG_PETROL, 1, 1, 1, 1}; p.car[1] = (AudCar){1, ENG_DIESEL, 1, 1, 1, 1};
    p.wind = p.road = p.skid = p.rain = p.scrape = p.fire = 1; p.heliRpm = 1; p.heliLoad = 1; p.heliL = p.heliR = 1; p.jetN1 = 1; p.jetL = p.jetR = 1;
    audio_set(&p);
    for (int s = 0; s < SND_COUNT; s++) audio_play_ui(s, 1.0f);
    render("everything at once (limited)", 3.0f, 0.3f, 1.0f);
    if (argc > 1) {
        FILE *f = fopen(argv[1], "wb");
        unsigned bytes = (unsigned)(nall * 2), rate = AUD_RATE, br = rate * 4; unsigned short ch = 2, fmt = 1, bits = 16, al = 4; unsigned x;
        fwrite("RIFF", 1, 4, f); x = 36 + bytes; fwrite(&x, 4, 1, f); fwrite("WAVEfmt ", 1, 8, f); x = 16; fwrite(&x, 4, 1, f);
        fwrite(&fmt, 2, 1, f); fwrite(&ch, 2, 1, f); fwrite(&rate, 4, 1, f); fwrite(&br, 4, 1, f); fwrite(&al, 2, 1, f); fwrite(&bits, 2, 1, f);
        fwrite("data", 1, 4, f); fwrite(&bytes, 4, 1, f);
        for (size_t i = 0; i < nall; i++) { short s = (short)(all[i] * 32767.0f); fwrite(&s, 2, 1, f); }
        fclose(f);
        printf("wrote %s (%.0f s)\n", argv[1], nall / 2.0 / AUD_RATE);
    }
    printf(bad ? "%d voice(s) out of range\n" : "all voices in range\n", bad);
    return bad != 0;
}
