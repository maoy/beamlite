// BeamLite — sound. Everything is synthesised as it plays (no sample files): engines, rotor, jets,
// wind, tyres and rain as continuous voices driven by the game each frame, and impacts as one-shots.
#ifndef BEAMLITE_AUDIO_H
#define BEAMLITE_AUDIO_H

#define AUD_RATE 48000
#define AUD_CARS 4              // engine voices: the player's car, then the nearest others

enum { ENG_ELECTRIC, ENG_PETROL, ENG_DIESEL };

typedef struct {
    int id;                     // which car this voice is (-1 none); a change fades out and back in
    int type;                   // ENG_*
    float rev, load;            // 0..1 of the rev range; throttle 0..1
    float gl, gr;               // gains, left and right (distance and pan already applied)
} AudCar;

// What the continuous voices should be doing now. Gains are 0..1 and already include distance.
typedef struct {
    float master;               // 0 silent .. 1
    float rate;                 // pitch / time scale: 1 normal, lower in slow motion
    AudCar car[AUD_CARS];
    float wind;                 // air rushing past the listener
    float road, skid;           // tyre roar and squeal
    float rain;
    float scrape;               // metal dragged along the ground
    float fire;                 // burning wreckage
    float heliRpm, heliLoad, heliL, heliR;      // rotor speed 0..1, collective 0..1
    float heliTail;             // 1 while the tail rotor still turns
    float jetN1, jetL, jetR;    // airliner engines: fan speed 0..1 (the synth adds the spool lag)
} AudioParams;

enum { SND_CRASH, SND_GLASS, SND_EXPLODE, SND_COLLAPSE, SND_BREACH, SND_TREE, SND_SPLASH, SND_QUACK, SND_ROTOR, SND_CHIME, SND_COUNT };

void audio_init(void);                          // opens the device and starts the stream
void audio_shutdown(void);
void audio_listener(float px, float py, float pz, float rx, float ry, float rz);   // ear position and the unit vector to its right
// Left/right gains for a source at (x, y, z) that is at full volume within `ref` metres.
void audio_spatial(float x, float y, float z, float ref, float *gl, float *gr);
void audio_set(const AudioParams *p);
void audio_play(int snd, float gain, float x, float y, float z);   // a one-shot in the world (gain 0..1 sets its size too)
void audio_play_ui(int snd, float gain);                           // a one-shot at the listener
unsigned audio_played(int snd);                                    // how many of these have been queued (for checks)
void audio_mix(float *out, unsigned frames);    // the synthesiser itself: interleaved stereo float (the device calls this)

#endif
