// BeamLite — HDR post-processing.
//
// The world is drawn into a half-float colour buffer with a depth texture. Then:
//   1. atmosphere: rebuild each pixel's world position from depth; draw sky (gradient, sun,
//      procedural clouds) where nothing was drawn, and blend exponential height fog elsewhere
//   2. wet surfaces (half resolution): classify up-facing pixels as asphalt / pavement / other
//      plus procedural puddles, then march a screen-space reflection ray through the depth
//      buffer (sky on a miss); blur it, mostly vertically, the way wet asphalt streaks lights
//   3. composite: darken wet surfaces and add the reflection
//   4. bloom: bright-pass at half resolution, then three blurred mip levels
//   5. present: anti-alias (FXAA-style), add bloom, ACES tone map, colour grade, vignette, dither

#include "render.h"
#include "rlgl.h"
#include "raymath.h"
#include <stddef.h>

#define NEAR_PLANE 0.1
#define FAR_PLANE 1500.0

#define GLSL "#version 330\nin vec2 fragTexCoord;\nin vec4 fragColor;\nuniform sampler2D texture0;\nuniform vec4 colDiffuse;\nout vec4 finalColor;\n"

// Shared by the atmosphere and reflection passes.
#define SKY_GLSL \
    "uniform sampler2D depthTex;\n" \
    "uniform mat4 invVP, VP;\n" \
    "uniform vec3 camPos, sunDir, sunCol, zenith, horizon, horizonAway, fogCol;\n" \
    "uniform float fogDensity, fogFalloff, time;\n" \
    "float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }\n" \
    "float noise(vec2 p) { vec2 i = floor(p), f = fract(p); vec2 u = f * f * (3.0 - 2.0 * f);\n" \
    "  return mix(mix(hash(i), hash(i + vec2(1, 0)), u.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), u.x), u.y); }\n" \
    "float fbm(vec2 p) { float a = 0.5, s = 0.0; for (int i = 0; i < 5; i++) { s += a * noise(p); p = p * 2.03 + vec2(1.7, 9.2); a *= 0.5; } return s; }\n" \
    "vec3 horizonCol(vec3 r) {\n" \
    "  vec2 a = normalize(r.xz + 1e-5), b = normalize(sunDir.xz + 1e-5);\n" \
    "  return mix(horizonAway, horizon, pow(dot(a, b) * 0.5 + 0.5, 3.0)); }\n" \
    "vec3 sky(vec3 r, float detail) {\n" \
    "  float sd = max(dot(r, sunDir), 0.0);\n" \
    "  vec3 hz = horizonCol(r);\n" \
    "  vec3 c = mix(hz, zenith, pow(clamp(r.y, 0.0, 1.0), 0.5));\n" \
    "  c += sunCol * (pow(sd, 6.0) * 0.30 + pow(sd, 90.0) * 0.9);\n" \
    "  c += sunCol * smoothstep(0.99935, 0.9997, sd) * 10.0;\n" \
    "  if (r.y > 0.0) {\n" \
    "    vec2 p = r.xz / (r.y + 0.10) * 1.4 + vec2(time * 0.006, time * 0.002);\n" \
    "    float n = fbm(p);\n" \
    "    float cl = smoothstep(0.42, 0.80, n) * smoothstep(0.0, 0.18, r.y);\n" \
    "    float lit = pow(sd, 2.5);\n" \
    "    vec3 cc = mix(zenith * 1.3 + hz * 0.25, sunCol * 1.1 + hz * 0.5, lit) * (0.55 + 0.7 * (detail > 0.5 ? fbm(p * 2.3 + 5.0) : 0.5));\n" \
    "    c = mix(c, cc, cl * 0.9);\n" \
    "  } else c = mix(hz, fogCol, clamp(-r.y * 5.0, 0.0, 1.0));\n" \
    "  return c; }\n" \
    "vec3 worldAt(vec2 uv, float d) { vec4 w = invVP * vec4(uv * 2.0 - 1.0, d * 2.0 - 1.0, 1.0); return w.xyz / w.w; }\n"

static const char *EMISSIVE_FS = GLSL
    "uniform float intensity;\n"
    "void main() { vec4 c = fragColor * colDiffuse; finalColor = vec4(c.rgb * intensity, c.a); }\n";

static const char *ATMOS_FS = GLSL SKY_GLSL
    "void main() {\n"
    "  vec2 uv = fragTexCoord;\n"
    "  float d = texture(depthTex, uv).r;\n"
    "  vec3 w = worldAt(uv, d);\n"
    "  vec3 r = normalize(w - camPos);\n"
    "  if (d >= 0.999999) { finalColor = vec4(sky(r, 1.0), 0.0); return; }\n"
    "  vec4 sc = texture(texture0, uv);\n"
    "  vec3 col = pow(max(sc.rgb, 0.0), vec3(2.2));\n"   // scene colours are authored in sRGB
    "  float dist = length(w - camPos);\n"
    "  float ry = r.y * dist * fogFalloff;\n"
    "  float f = fogDensity * exp(-fogFalloff * camPos.y) * dist * (abs(ry) > 1e-4 ? (1.0 - exp(-ry)) / ry : 1.0);\n"
    "  float fog = 1.0 - exp(-max(f, 0.0));\n"
    "  vec3 fc = mix(horizonCol(r), fogCol, 0.5) + sunCol * 0.5 * pow(max(dot(r, sunDir), 0.0), 8.0);\n"
    "  finalColor = vec4(mix(col, fc, fog), clamp(sc.a, 0.0, 1.0));\n"   // alpha: 1 = ground (may be wet)
    "}\n";

// texture0 = the fogged HDR scene. Output: rgb = reflection * strength (premultiplied), a = wetness.
static const char *SSR_FS = GLSL SKY_GLSL
    "uniform float wet, rain;\n"
    "uniform vec4 roads[16];\n"
    "uniform int nRoads, nHighway;\n"
    "float linz(float d) { float z = d * 2.0 - 1.0; return 2.0 * 0.1 * 1500.0 / (1500.1 - z * 1499.9); }\n"
    "float surface(vec2 p) {\n"      // 1 asphalt, 0.6 pavement, 0.22 anything else
    "  float s = 0.10;\n"
    "  for (int i = 0; i < 16; i++) {\n"
    "    if (i >= nRoads) break;\n"
    "    vec4 r = roads[i];\n"
    "    if (p.x > r.x && p.x < r.y && p.y > r.z && p.y < r.w) return 1.0;\n"
    "    if (i >= nHighway && p.x > r.x - 2.2 && p.x < r.y + 2.2 && p.y > r.z - 2.2 && p.y < r.w + 2.2) s = 0.6;\n"
    "  }\n"
    "  return s; }\n"
    "void main() {\n"
    "  vec2 uv = fragTexCoord;\n"
    "  float d = texture(depthTex, uv).r;\n"
    "  vec3 P = worldAt(uv, d);\n"
    "  vec3 Ng = normalize(cross(dFdx(P), dFdy(P)));\n"
    "  if (Ng.y < 0.0) Ng = -Ng;\n"
    "  if (wet <= 0.0 || d >= 0.999999 || Ng.y < 0.85 || texture(texture0, uv).a < 0.5) { finalColor = vec4(0.0); return; }\n"
    "  float surf = surface(P.xz);\n"
    "  float pud = smoothstep(0.60, 0.645, fbm(P.xz * 0.13 + vec2(3.1, 7.7))) * step(0.5, surf);\n"
    "  float w = wet * mix(surf, 1.0, pud);\n"
    "  vec3 N = vec3(0.0, 1.0, 0.0);\n"
    "  float rip = 0.010 + rain * 0.022 * (0.5 + pud);\n"   // fine rain ripples: roughen, don't warp
    "  vec2 q = P.xz * 9.0; float tt = time * 6.0;\n"
    "  N.x += (noise(q + vec2(0.0, tt)) - 0.5) * rip * 4.0;\n"
    "  N.z += (noise(q.yx + vec2(tt, 3.0)) - 0.5) * rip * 4.0;\n"
    "  N = normalize(N);\n"
    "  vec3 V = normalize(P - camPos);\n"
    "  vec3 R = reflect(V, N);\n"
    "  float F = 0.03 + 0.97 * pow(1.0 - max(dot(-V, N), 0.0), 5.0);\n"
    "  float k = w * mix(0.10 + 0.55 * F, 0.38 + 0.62 * F, pud);\n"   // damp asphalt: sheen; puddles: mirror
    "  vec3 O = P + N * 0.03;\n"
    "  vec3 col = vec3(-1.0);\n"
    "  float t = 0.25, tp = 0.0;\n"
    "  for (int i = 0; i < 30; i++) {\n"
    "    vec4 c = VP * vec4(O + R * t, 1.0);\n"
    "    if (c.w <= 0.0) break;\n"
    "    vec3 n = c.xyz / c.w; vec2 s = n.xy * 0.5 + 0.5;\n"
    "    if (s.x < 0.0 || s.x > 1.0 || s.y < 0.0 || s.y > 1.0) break;\n"
    "    float sd = texture(depthTex, s).r;\n"
    "    if (n.z * 0.5 + 0.5 > sd) {\n"
    "      if (sd >= 0.999999) break;\n"
    "      float a = tp, b = t;\n"
    "      for (int j = 0; j < 6; j++) {\n"
    "        float m = 0.5 * (a + b); vec4 cm = VP * vec4(O + R * m, 1.0); vec3 nm = cm.xyz / cm.w;\n"
    "        if (nm.z * 0.5 + 0.5 > texture(depthTex, nm.xy * 0.5 + 0.5).r) b = m; else a = m;\n"
    "      }\n"
    "      vec4 cb = VP * vec4(O + R * b, 1.0); vec2 sb = cb.xy / cb.w * 0.5 + 0.5;\n"
    "      vec2 e = smoothstep(0.0, 0.07, sb) * (1.0 - smoothstep(0.93, 1.0, sb));\n"
    "      float fade = e.x * e.y * (1.0 - smoothstep(70.0, 140.0, b)) * (1.0 - smoothstep(0.15, 0.6, dot(R, -V)));\n"
    "      col = mix(sky(R, 0.0), texture(texture0, sb).rgb, fade);\n"
    "      break;\n"
    "    }\n"
    "    tp = t; t = t * 1.20 + 0.25;\n"
    "  }\n"
    "  if (col.x < 0.0) col = sky(R, 0.0);\n"
    "  col = min(col, vec3(40.0));\n"
    "  finalColor = vec4(col * k, w);\n"
    "}\n";

static const char *COMPOSITE_FS = GLSL
    "uniform sampler2D reflTex;\n"
    "void main() {\n"
    "  vec3 c = texture(texture0, fragTexCoord).rgb;\n"
    "  vec4 r = texture(reflTex, fragTexCoord);\n"
    "  finalColor = vec4(c * (1.0 - 0.42 * r.a) + r.rgb, 1.0);\n"   // wet surfaces darken, then reflect
    "}\n";

static const char *BRIGHT_FS = GLSL
    "uniform vec2 texel;\n"
    "uniform float threshold;\n"
    "void main() {\n"
    "  vec3 c = 0.25 * (texture(texture0, fragTexCoord + texel * vec2(-0.5, -0.5)).rgb + texture(texture0, fragTexCoord + texel * vec2(0.5, -0.5)).rgb\n"
    "                 + texture(texture0, fragTexCoord + texel * vec2(-0.5, 0.5)).rgb + texture(texture0, fragTexCoord + texel * vec2(0.5, 0.5)).rgb);\n"
    "  c = min(c, vec3(64.0));\n"
    "  float l = max(c.r, max(c.g, c.b));\n"
    "  float k = max(l - threshold, 0.0); k = k * k / (k + 0.6);\n"
    "  finalColor = vec4(c * (k / max(l, 1e-4)), 1.0);\n"
    "}\n";

static const char *BLUR_FS = GLSL
    "uniform vec2 stepv;\n"
    "void main() {\n"
    "  float w[5] = float[](0.227027, 0.1945946, 0.1216216, 0.054054, 0.016216);\n"
    "  vec4 c = texture(texture0, fragTexCoord) * w[0];\n"
    "  for (int i = 1; i < 5; i++) c += (texture(texture0, fragTexCoord + stepv * float(i)) + texture(texture0, fragTexCoord - stepv * float(i))) * w[i];\n"
    "  finalColor = c;\n"
    "}\n";

static const char *FINAL_FS = GLSL
    "uniform sampler2D bloom1, bloom2, bloom3;\n"
    "uniform vec2 texel;\n"
    "uniform float exposure, bloomStrength;\n"
    "vec3 aces(vec3 x) { return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0); }\n"
    "float luma(vec3 c) { return dot(c, vec3(0.299, 0.587, 0.114)); }\n"
    "float tl(vec2 uv) { return luma(aces(texture(texture0, uv).rgb * exposure)); }\n"
    "float hash(vec2 p) { return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453); }\n"
    "void main() {\n"
    "  vec2 uv = fragTexCoord;\n"
    "  vec3 hdr = texture(texture0, uv).rgb;\n"
    // Edge-directed anti-aliasing on tone-mapped luma (the scene buffer has no MSAA).
    "  float lM = luma(aces(hdr * exposure)), lN = tl(uv + vec2(0, texel.y)), lS = tl(uv - vec2(0, texel.y));\n"
    "  float lE = tl(uv + vec2(texel.x, 0)), lW = tl(uv - vec2(texel.x, 0));\n"
    "  float lmin = min(lM, min(min(lN, lS), min(lE, lW))), lmax = max(lM, max(max(lN, lS), max(lE, lW)));\n"
    "  if (lmax - lmin > max(0.03, lmax * 0.10)) {\n"
    "    vec2 g = vec2(lE - lW, lN - lS);\n"
    "    vec2 dir = normalize(vec2(-g.y, g.x) + 1e-6) * texel;\n"
    "    vec3 a = 0.5 * (texture(texture0, uv - dir * 0.5).rgb + texture(texture0, uv + dir * 0.5).rgb);\n"
    "    vec3 b = a * 0.5 + 0.25 * (texture(texture0, uv - dir * 2.0).rgb + texture(texture0, uv + dir * 2.0).rgb);\n"
    "    float lb = luma(aces(b * exposure));\n"
    "    hdr = (lb < lmin || lb > lmax) ? a : b;\n"
    "  }\n"
    "  vec3 bl = texture(bloom1, uv).rgb * 0.45 + texture(bloom2, uv).rgb * 0.35 + texture(bloom3, uv).rgb * 0.35;\n"
    "  vec3 c = aces((hdr + bl * bloomStrength) * exposure);\n"
    // Grade: cool the shadows, warm the highlights, a touch more saturation.
    "  float L = luma(c);\n"
    "  c *= mix(vec3(0.86, 0.94, 1.14), vec3(1.05, 1.0, 0.93), smoothstep(0.05, 0.6, L));\n"
    "  c = mix(vec3(luma(c)), c, 1.12);\n"
    "  vec2 q = uv - 0.5; c *= 1.0 - dot(q, q) * 0.6;\n"
    "  c = pow(max(c, 0.0), vec3(1.0 / 2.2));\n"
    "  c += (hash(uv * 1733.0) - 0.5) / 255.0;\n"
    "  finalColor = vec4(c, 1.0);\n"
    "}\n";

enum { U_DEPTH, U_INVVP, U_VP, U_CAM, U_SUNDIR, U_SUNCOL, U_ZEN, U_HOR, U_HORAWAY, U_FOG, U_FOGD, U_FOGF, U_TIME, U_COUNT };
static const char *SKY_UNIFORMS[U_COUNT] = {"depthTex", "invVP", "VP", "camPos", "sunDir", "sunCol", "zenith", "horizon", "horizonAway",
                                            "fogCol", "fogDensity", "fogFalloff", "time"};

static Shader shEmissive, shAtmos, shSSR, shComposite, shBright, shBlur, shFinal;
static int locIntensity, locAtmos[U_COUNT], locSSR[U_COUNT], locWet, locRain, locRoads, locNRoads, locNHighway, locRefl;
static int locBrightTexel, locBrightThr, locBlurStep, locFinal[6];
static RenderTexture2D scene, hdr, hdr2, refl[2], bl[3][2];
static int curW, curH;
static Matrix invVP, VP;
static Vector3 camPos;
static float roadRects[16 * 4];
static int nRoadRects, nHighwayRects;

static RenderTexture2D make_rt(int w, int h, int depth) {
    RenderTexture2D t = {0};
    t.id = rlLoadFramebuffer();
    rlEnableFramebuffer(t.id);
    t.texture.id = rlLoadTexture(NULL, w, h, PIXELFORMAT_UNCOMPRESSED_R16G16B16A16, 1);
    t.texture.width = w; t.texture.height = h; t.texture.mipmaps = 1;
    t.texture.format = PIXELFORMAT_UNCOMPRESSED_R16G16B16A16;
    rlFramebufferAttach(t.id, t.texture.id, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
    if (depth) {
        t.depth.id = rlLoadTextureDepth(w, h, false);
        t.depth.width = w; t.depth.height = h; t.depth.mipmaps = 1;
        rlFramebufferAttach(t.id, t.depth.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
    }
    if (!rlFramebufferComplete(t.id)) TraceLog(LOG_WARNING, "post: framebuffer %dx%d incomplete", w, h);
    rlDisableFramebuffer();
    SetTextureFilter(t.texture, TEXTURE_FILTER_BILINEAR);
    SetTextureWrap(t.texture, TEXTURE_WRAP_CLAMP);
    return t;
}

static void free_targets(void) {
    if (!curW) return;
    UnloadRenderTexture(scene); UnloadRenderTexture(hdr); UnloadRenderTexture(hdr2);
    UnloadRenderTexture(refl[0]); UnloadRenderTexture(refl[1]);
    for (int i = 0; i < 3; i++) { UnloadRenderTexture(bl[i][0]); UnloadRenderTexture(bl[i][1]); }
    curW = curH = 0;
}

#define MAX_INTERNAL_W 2160                     // render at most this wide, upscale to the screen

static void ensure_targets(void) {
    int w = GetRenderWidth(), h = GetRenderHeight();
    if (w > MAX_INTERNAL_W) { h = h * MAX_INTERNAL_W / w; w = MAX_INTERNAL_W; }
    if (w == curW && h == curH) return;
    free_targets();
    if (w < 8 || h < 8) return;
    scene = make_rt(w, h, 1);
    hdr = make_rt(w, h, 0);
    hdr2 = make_rt(w, h, 0);
    refl[0] = make_rt(w / 3, h / 3, 0);                // reflections are blurred anyway: 1/3 resolution
    refl[1] = make_rt(w / 3, h / 3, 0);
    for (int i = 0; i < 3; i++) {
        int d = 2 << i;
        bl[i][0] = make_rt(w / d, h / d, 0);
        bl[i][1] = make_rt(w / d, h / d, 0);
    }
    curW = w; curH = h;
}

void post_init(void) {
    rlSetClipPlanes(NEAR_PLANE, FAR_PLANE);
    shEmissive = LoadShaderFromMemory(NULL, EMISSIVE_FS);
    shAtmos = LoadShaderFromMemory(NULL, ATMOS_FS);
    shSSR = LoadShaderFromMemory(NULL, SSR_FS);
    shComposite = LoadShaderFromMemory(NULL, COMPOSITE_FS);
    shBright = LoadShaderFromMemory(NULL, BRIGHT_FS);
    shBlur = LoadShaderFromMemory(NULL, BLUR_FS);
    shFinal = LoadShaderFromMemory(NULL, FINAL_FS);
    locIntensity = GetShaderLocation(shEmissive, "intensity");
    for (int i = 0; i < U_COUNT; i++) {
        locAtmos[i] = GetShaderLocation(shAtmos, SKY_UNIFORMS[i]);
        locSSR[i] = GetShaderLocation(shSSR, SKY_UNIFORMS[i]);
    }
    locWet = GetShaderLocation(shSSR, "wet");
    locRain = GetShaderLocation(shSSR, "rain");
    locRoads = GetShaderLocation(shSSR, "roads");
    locNRoads = GetShaderLocation(shSSR, "nRoads");
    locNHighway = GetShaderLocation(shSSR, "nHighway");
    locRefl = GetShaderLocation(shComposite, "reflTex");
    locBrightTexel = GetShaderLocation(shBright, "texel");
    locBrightThr = GetShaderLocation(shBright, "threshold");
    locBlurStep = GetShaderLocation(shBlur, "stepv");
    const char *fn[6] = {"bloom1", "bloom2", "bloom3", "texel", "exposure", "bloomStrength"};
    for (int i = 0; i < 6; i++) locFinal[i] = GetShaderLocation(shFinal, fn[i]);
    ensure_targets();
}

void post_set_roads(const float (*rects)[4], int n, int nHighway) {
    if (n > 16) n = 16;
    for (int i = 0; i < n; i++) for (int k = 0; k < 4; k++) roadRects[i * 4 + k] = rects[i][k];
    nRoadRects = n; nHighwayRects = nHighway;
}

void post_begin_scene(void) {
    ensure_targets();
    BeginTextureMode(scene);
    ClearBackground(BLACK);
}

void post_capture_camera(Camera3D cam) {
    Matrix view = rlGetMatrixModelview(), proj = rlGetMatrixProjection();
    VP = MatrixMultiply(view, proj);
    invVP = MatrixInvert(VP);
    camPos = cam.position;
}

void post_end_scene(void) { EndTextureMode(); }

// Everything except the ground: normal colour blending, but zero the buffer's alpha (= "not ground").
void post_begin_objects(void) {
    rlDrawRenderBatchActive();
    rlSetBlendFactorsSeparate(0x0302, 0x0303, 0, 0, 0x8006, 0x8006);   // SRC_ALPHA, ONE_MINUS_SRC_ALPHA | ZERO, ZERO
    rlSetBlendMode(BLEND_CUSTOM_SEPARATE);
}
void post_end_objects(void) { rlSetBlendMode(BLEND_ALPHA); }

void post_emissive_begin(float intensity) {
    BeginShaderMode(shEmissive);
    SetShaderValue(shEmissive, locIntensity, &intensity, SHADER_UNIFORM_FLOAT);
}
void post_emissive_end(void) { EndShaderMode(); }
Shader post_emissive_shader(void) { return shEmissive; }
void post_set_emissive(float intensity) { SetShaderValue(shEmissive, locIntensity, &intensity, SHADER_UNIFORM_FLOAT); }

// Draw src through shader sh into dst (render textures are stored flipped, hence -height).
static void blit(RenderTexture2D dst, Shader sh, Texture2D src) {
    BeginTextureMode(dst);
    BeginShaderMode(sh);
    DrawTexturePro(src, (Rectangle){0, 0, (float)src.width, -(float)src.height},
                   (Rectangle){0, 0, (float)dst.texture.width, (float)dst.texture.height}, (Vector2){0, 0}, 0, WHITE);
    EndShaderMode();
    EndTextureMode();
}

// Sky/fog/camera uniforms shared by the atmosphere and reflection shaders (call inside shader mode).
static void set_sky_uniforms(Shader sh, const int *loc, const Env *e, float time) {
    SetShaderValueTexture(sh, loc[U_DEPTH], scene.depth);
    SetShaderValueMatrix(sh, loc[U_INVVP], invVP);
    SetShaderValueMatrix(sh, loc[U_VP], VP);
    SetShaderValue(sh, loc[U_CAM], &camPos, SHADER_UNIFORM_VEC3);
    Vector3 sd = Vector3Normalize(e->sunDir);
    SetShaderValue(sh, loc[U_SUNDIR], &sd, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, loc[U_SUNCOL], &e->sunCol, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, loc[U_ZEN], &e->zenith, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, loc[U_HOR], &e->horizon, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, loc[U_HORAWAY], &e->horizonAway, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, loc[U_FOG], &e->fogCol, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, loc[U_FOGD], &e->fogDensity, SHADER_UNIFORM_FLOAT);
    SetShaderValue(sh, loc[U_FOGF], &e->fogFalloff, SHADER_UNIFORM_FLOAT);
    SetShaderValue(sh, loc[U_TIME], &time, SHADER_UNIFORM_FLOAT);
}

static void blur_pass(RenderTexture2D a, RenderTexture2D b, float sx, float sy) {
    Vector2 tx = {1.0f / a.texture.width, 1.0f / a.texture.height};
    Vector2 h = {tx.x * sx, 0}, v = {0, tx.y * sy};
    SetShaderValue(shBlur, locBlurStep, &h, SHADER_UNIFORM_VEC2);
    blit(b, shBlur, a.texture);
    SetShaderValue(shBlur, locBlurStep, &v, SHADER_UNIFORM_VEC2);
    blit(a, shBlur, b.texture);
}

void post_present(const Env *e, float time, float wet, float rain) {
    if (!curW) return;
    Rectangle full = {0, 0, (float)curW, -(float)curH}, dst = {0, 0, (float)curW, (float)curH};

    // 1. Atmosphere (alpha written as-is: it carries the ground mask).
    BeginTextureMode(hdr);
    rlSetBlendFactors(1, 0, 0x8006);
    rlSetBlendMode(BLEND_CUSTOM);
    BeginShaderMode(shAtmos);
    set_sky_uniforms(shAtmos, locAtmos, e, time);
    DrawTexturePro(scene.texture, full, dst, (Vector2){0, 0}, 0, WHITE);
    EndShaderMode();
    rlSetBlendMode(BLEND_ALPHA);
    EndTextureMode();

    // 2-3. Wet surfaces: reflections at half resolution, streaky blur, composite.
    RenderTexture2D *lit = &hdr;
    if (wet > 0.0f) {
        BeginTextureMode(refl[0]);
        ClearBackground(BLANK);
        BeginShaderMode(shSSR);
        set_sky_uniforms(shSSR, locSSR, e, time);
        SetShaderValue(shSSR, locWet, &wet, SHADER_UNIFORM_FLOAT);
        SetShaderValue(shSSR, locRain, &rain, SHADER_UNIFORM_FLOAT);
        SetShaderValueV(shSSR, locRoads, roadRects, SHADER_UNIFORM_VEC4, nRoadRects);
        SetShaderValue(shSSR, locNRoads, &nRoadRects, SHADER_UNIFORM_INT);
        SetShaderValue(shSSR, locNHighway, &nHighwayRects, SHADER_UNIFORM_INT);
        rlSetBlendFactors(1, 0, 0x8006);               // write rgba as-is (alpha carries wetness): ONE, ZERO, ADD
        rlSetBlendMode(BLEND_CUSTOM);
        DrawTexturePro(hdr.texture, full, (Rectangle){0, 0, (float)refl[0].texture.width, (float)refl[0].texture.height}, (Vector2){0, 0}, 0, WHITE);
        EndShaderMode();
        rlSetBlendMode(BLEND_ALPHA);
        EndTextureMode();
        rlSetBlendFactors(1, 0, 0x8006);
        rlSetBlendMode(BLEND_CUSTOM);
        blur_pass(refl[0], refl[1], 0.5f, 1.8f);       // wet asphalt stretches reflections vertically
        rlSetBlendMode(BLEND_ALPHA);

        BeginTextureMode(hdr2);
        BeginShaderMode(shComposite);
        SetShaderValueTexture(shComposite, locRefl, refl[0].texture);
        DrawTexturePro(hdr.texture, full, dst, (Vector2){0, 0}, 0, WHITE);
        EndShaderMode();
        EndTextureMode();
        lit = &hdr2;
    }

    // 4. Bloom: bright pass, then blur at 1/2, 1/4 and 1/8 resolution.
    Vector2 texel = {1.0f / curW, 1.0f / curH};
    float thr = 1.0f;
    SetShaderValue(shBright, locBrightTexel, &texel, SHADER_UNIFORM_VEC2);
    SetShaderValue(shBright, locBrightThr, &thr, SHADER_UNIFORM_FLOAT);
    blit(bl[0][0], shBright, lit->texture);
    Vector2 zero = {0, 0};
    for (int i = 0; i < 3; i++) {
        if (i > 0) { SetShaderValue(shBlur, locBlurStep, &zero, SHADER_UNIFORM_VEC2); blit(bl[i][0], shBlur, bl[i - 1][0].texture); }
        blur_pass(bl[i][0], bl[i][1], 1.5f, 1.5f);
        blur_pass(bl[i][0], bl[i][1], 1.5f, 1.5f);
    }

    // 5. Present to the screen.
    BeginShaderMode(shFinal);
    SetShaderValueTexture(shFinal, locFinal[0], bl[0][0].texture);
    SetShaderValueTexture(shFinal, locFinal[1], bl[1][0].texture);
    SetShaderValueTexture(shFinal, locFinal[2], bl[2][0].texture);
    SetShaderValue(shFinal, locFinal[3], &texel, SHADER_UNIFORM_VEC2);
    SetShaderValue(shFinal, locFinal[4], &e->exposure, SHADER_UNIFORM_FLOAT);
    SetShaderValue(shFinal, locFinal[5], &e->bloom, SHADER_UNIFORM_FLOAT);
    DrawTexturePro(lit->texture, full, (Rectangle){0, 0, (float)GetScreenWidth(), (float)GetScreenHeight()}, (Vector2){0, 0}, 0, WHITE);
    EndShaderMode();
}

void post_shutdown(void) {
    free_targets();
    UnloadShader(shEmissive); UnloadShader(shAtmos); UnloadShader(shSSR); UnloadShader(shComposite);
    UnloadShader(shBright); UnloadShader(shBlur); UnloadShader(shFinal);
}
