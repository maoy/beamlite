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
// Vehicle paint: per-pixel shading for immediate-mode geometry whose vertices carry a normal and a
// material id in the colour's alpha (see render.c). bakeAmb/bakeSun are the vertex-light multipliers.
void post_paint_begin(const Env *e, Vector3 bakeAmb, Vector3 bakeSun, float time);
void post_paint_end(void);
// Sun shadows. post_shadow_begin returns 1 when the map is due a refresh: then draw the casters
// (depth only) and call post_shadow_end. post_shadow_active: shadows are in effect this frame.
int post_shadow_begin(Vector3 focus, const Env *e, Vector3 bakeAmb, Vector3 bakeSun);
void post_shadow_end(void);
int post_shadow_active(void);
Shader post_depth_shader(void);
void post_set_quality(int shadows, int ao);     // toggle sun shadows / ambient occlusion
void post_shutdown(void);

#endif
