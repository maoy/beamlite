// BeamLite — HDR post-processing: sky, atmospheric haze, bloom, tone mapping, grading, anti-aliasing.
#ifndef BEAMLITE_RENDER_H
#define BEAMLITE_RENDER_H

#include "raylib.h"

// Lighting environment. Colours are linear.
typedef struct {
    const char *name;
    Vector3 sunDir, sunCol, ambient;            // sun/ambient also drive the baked vertex lighting
    Vector3 zenith, horizon, horizonAway;       // sky: overhead, toward the sun, away from it
    Vector3 fogCol;
    float fogDensity, fogFalloff;               // exponential height fog
    float exposure, bloom;
    float lights;                               // how strongly artificial lights glow (0 = off)
    Vector3 bakeAmbient, bakeSun;               // vertex-lighting multipliers (display space)
} Env;

void post_init(void);                           // after InitWindow
void post_begin_scene(void);                    // render the world between these two...
void post_capture_camera(Camera3D cam);         // ...call right after BeginMode3D
void post_end_scene(void);
void post_begin_objects(void);                  // draw non-ground geometry between these (keeps it dry)
void post_end_objects(void);
void post_present(const Env *e, float time, float wet, float rain);   // draws the graded image to the screen
void post_set_roads(const float (*rects)[4], int n, int nHighway);    // road rectangles (x0, x1, z0, z1) for wetness
void post_set_broadway(float x0, float dxdz, float halfX, float z0, float zmin, float zmax);   // a diagonal road (zmin >= zmax: none)
Shader post_billboard_shader(void);             // animated LED screens (see render.c for the vertex data layout)
void post_set_billboards(float time, float intensity);
void post_emissive_begin(float intensity);      // draw glowing things (colour * intensity, can exceed 1)
void post_emissive_end(void);
Shader post_emissive_shader(void);              // for models: material shader...
void post_set_emissive(float intensity);        // ...and its intensity
void post_shutdown(void);

#endif
