// BeamLite — HDR post-processing.
//
// The world is drawn into a half-float colour buffer with a depth texture. Then:
//   1. atmosphere: rebuild each pixel's world position from depth; draw sky (gradient, sun,
//      procedural clouds) where nothing was drawn, and blend exponential height fog elsewhere
//   2. bloom: bright-pass at half resolution, then three blurred mip levels
//   3. present: anti-alias (FXAA-style), add bloom, ACES tone map, colour grade, vignette, dither

#include "render.h"
#include "rlgl.h"
#include "raymath.h"
#include <stddef.h>

#define GLSL "#version 330\nin vec2 fragTexCoord;\nin vec4 fragColor;\nuniform sampler2D texture0;\nuniform vec4 colDiffuse;\nout vec4 finalColor;\n"

static const char *EMISSIVE_FS = GLSL
    "uniform float intensity;\n"
    "void main() { vec4 c = fragColor * colDiffuse; finalColor = vec4(c.rgb * intensity, c.a); }\n";

static const char *ATMOS_FS = GLSL
    "uniform sampler2D depthTex;\n"
    "uniform mat4 invVP;\n"
    "uniform vec3 camPos, sunDir, sunCol, zenith, horizon, horizonAway, fogCol;\n"
    "uniform float fogDensity, fogFalloff, time;\n"
    "float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }\n"
    "float noise(vec2 p) { vec2 i = floor(p), f = fract(p); vec2 u = f * f * (3.0 - 2.0 * f);\n"
    "  return mix(mix(hash(i), hash(i + vec2(1, 0)), u.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), u.x), u.y); }\n"
    "float fbm(vec2 p) { float a = 0.5, s = 0.0; for (int i = 0; i < 5; i++) { s += a * noise(p); p = p * 2.03 + vec2(1.7, 9.2); a *= 0.5; } return s; }\n"
    "vec3 horizonCol(vec3 r) {\n"
    "  vec2 a = normalize(r.xz + 1e-5), b = normalize(sunDir.xz + 1e-5);\n"
    "  return mix(horizonAway, horizon, pow(dot(a, b) * 0.5 + 0.5, 3.0)); }\n"
    "vec3 sky(vec3 r) {\n"
    "  float sd = max(dot(r, sunDir), 0.0);\n"
    "  vec3 hz = horizonCol(r);\n"
    "  vec3 c = mix(hz, zenith, pow(clamp(r.y, 0.0, 1.0), 0.5));\n"
    "  c += sunCol * (pow(sd, 6.0) * 0.30 + pow(sd, 90.0) * 0.9);\n"
    "  c += sunCol * smoothstep(0.99935, 0.9997, sd) * 10.0;\n"
    "  if (r.y > 0.0) {\n"
    "    vec2 p = r.xz / (r.y + 0.10) * 1.4 + vec2(time * 0.006, time * 0.002);\n"
    "    float n = fbm(p);\n"
    "    float cl = smoothstep(0.42, 0.80, n) * smoothstep(0.0, 0.18, r.y);\n"
    "    float lit = pow(sd, 2.5);\n"
    "    vec3 cc = mix(zenith * 1.3 + hz * 0.25, sunCol * 1.1 + hz * 0.5, lit) * (0.55 + 0.7 * fbm(p * 2.3 + 5.0));\n"
    "    c = mix(c, cc, cl * 0.9);\n"
    "  } else c = mix(hz, fogCol, clamp(-r.y * 5.0, 0.0, 1.0));\n"
    "  return c; }\n"
    "void main() {\n"
    "  vec2 uv = fragTexCoord;\n"
    "  float d = texture(depthTex, uv).r;\n"
    "  vec4 w = invVP * vec4(uv * 2.0 - 1.0, d * 2.0 - 1.0, 1.0); w /= w.w;\n"
    "  vec3 r = normalize(w.xyz - camPos);\n"
    "  if (d >= 0.999999) { finalColor = vec4(sky(r), 1.0); return; }\n"
    "  vec3 col = pow(max(texture(texture0, uv).rgb, 0.0), vec3(2.2));\n"   // scene colours are authored in sRGB
    "  float dist = length(w.xyz - camPos);\n"
    "  float ry = r.y * dist * fogFalloff;\n"
    "  float f = fogDensity * exp(-fogFalloff * camPos.y) * dist * (abs(ry) > 1e-4 ? (1.0 - exp(-ry)) / ry : 1.0);\n"
    "  float fog = 1.0 - exp(-max(f, 0.0));\n"
    "  vec3 fc = mix(horizonCol(r), fogCol, 0.5) + sunCol * 0.5 * pow(max(dot(r, sunDir), 0.0), 8.0);\n"
    "  finalColor = vec4(mix(col, fc, fog), 1.0);\n"
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
    "  vec3 c = texture(texture0, fragTexCoord).rgb * w[0];\n"
    "  for (int i = 1; i < 5; i++) c += (texture(texture0, fragTexCoord + stepv * float(i)).rgb + texture(texture0, fragTexCoord - stepv * float(i)).rgb) * w[i];\n"
    "  finalColor = vec4(c, 1.0);\n"
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

static Shader shEmissive, shAtmos, shBright, shBlur, shFinal;
static int locIntensity;
static int locA[12], locBrightTexel, locBrightThr, locBlurStep, locFinal[5];
static RenderTexture2D scene, hdr, bl[3][2];
static int curW, curH;
static Matrix invVP;
static Vector3 camPos;

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
    UnloadRenderTexture(scene); UnloadRenderTexture(hdr);
    for (int i = 0; i < 3; i++) { UnloadRenderTexture(bl[i][0]); UnloadRenderTexture(bl[i][1]); }
    curW = curH = 0;
}

static void ensure_targets(void) {
    int w = GetRenderWidth(), h = GetRenderHeight();
    if (w == curW && h == curH) return;
    free_targets();
    if (w < 8 || h < 8) return;
    scene = make_rt(w, h, 1);
    hdr = make_rt(w, h, 0);
    for (int i = 0; i < 3; i++) {
        int d = 2 << i;
        bl[i][0] = make_rt(w / d, h / d, 0);
        bl[i][1] = make_rt(w / d, h / d, 0);
    }
    curW = w; curH = h;
}

void post_init(void) {
    rlSetClipPlanes(0.1, 1500.0);
    shEmissive = LoadShaderFromMemory(NULL, EMISSIVE_FS);
    shAtmos = LoadShaderFromMemory(NULL, ATMOS_FS);
    shBright = LoadShaderFromMemory(NULL, BRIGHT_FS);
    shBlur = LoadShaderFromMemory(NULL, BLUR_FS);
    shFinal = LoadShaderFromMemory(NULL, FINAL_FS);
    locIntensity = GetShaderLocation(shEmissive, "intensity");
    const char *an[12] = {"depthTex", "invVP", "camPos", "sunDir", "sunCol", "zenith", "horizon", "horizonAway", "fogCol", "fogDensity", "fogFalloff", "time"};
    for (int i = 0; i < 12; i++) locA[i] = GetShaderLocation(shAtmos, an[i]);
    locBrightTexel = GetShaderLocation(shBright, "texel");
    locBrightThr = GetShaderLocation(shBright, "threshold");
    locBlurStep = GetShaderLocation(shBlur, "stepv");
    const char *fn[5] = {"bloom1", "bloom2", "bloom3", "texel", "exposure"};
    for (int i = 0; i < 5; i++) locFinal[i] = GetShaderLocation(shFinal, fn[i]);
    ensure_targets();
}

void post_begin_scene(void) {
    ensure_targets();
    BeginTextureMode(scene);
    ClearBackground(BLACK);
}

void post_capture_camera(Camera3D cam) {
    Matrix view = rlGetMatrixModelview(), proj = rlGetMatrixProjection();
    invVP = MatrixInvert(MatrixMultiply(view, proj));
    camPos = cam.position;
}

void post_end_scene(void) { EndTextureMode(); }

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

void post_present(const Env *e, float time) {
    if (!curW) return;
    // 1. Atmosphere.
    BeginTextureMode(hdr);
    BeginShaderMode(shAtmos);
    SetShaderValueTexture(shAtmos, locA[0], scene.depth);
    SetShaderValueMatrix(shAtmos, locA[1], invVP);
    SetShaderValue(shAtmos, locA[2], &camPos, SHADER_UNIFORM_VEC3);
    Vector3 sd = Vector3Normalize(e->sunDir);
    SetShaderValue(shAtmos, locA[3], &sd, SHADER_UNIFORM_VEC3);
    SetShaderValue(shAtmos, locA[4], &e->sunCol, SHADER_UNIFORM_VEC3);
    SetShaderValue(shAtmos, locA[5], &e->zenith, SHADER_UNIFORM_VEC3);
    SetShaderValue(shAtmos, locA[6], &e->horizon, SHADER_UNIFORM_VEC3);
    SetShaderValue(shAtmos, locA[7], &e->horizonAway, SHADER_UNIFORM_VEC3);
    SetShaderValue(shAtmos, locA[8], &e->fogCol, SHADER_UNIFORM_VEC3);
    SetShaderValue(shAtmos, locA[9], &e->fogDensity, SHADER_UNIFORM_FLOAT);
    SetShaderValue(shAtmos, locA[10], &e->fogFalloff, SHADER_UNIFORM_FLOAT);
    SetShaderValue(shAtmos, locA[11], &time, SHADER_UNIFORM_FLOAT);
    DrawTexturePro(scene.texture, (Rectangle){0, 0, (float)curW, -(float)curH}, (Rectangle){0, 0, (float)curW, (float)curH}, (Vector2){0, 0}, 0, WHITE);
    EndShaderMode();
    EndTextureMode();

    // 2. Bloom: bright pass, then blur at 1/2, 1/4 and 1/8 resolution.
    Vector2 texel = {1.0f / curW, 1.0f / curH};
    float thr = 1.0f;
    SetShaderValue(shBright, locBrightTexel, &texel, SHADER_UNIFORM_VEC2);
    SetShaderValue(shBright, locBrightThr, &thr, SHADER_UNIFORM_FLOAT);
    blit(bl[0][0], shBright, hdr.texture);
    for (int i = 0; i < 3; i++) {
        if (i > 0) blit(bl[i][0], shBlur, bl[i - 1][0].texture);   // (blur shader with zero step = copy)
        Vector2 tx = {1.0f / bl[i][0].texture.width, 1.0f / bl[i][0].texture.height};
        for (int it = 0; it < 2; it++) {
            Vector2 sh = {tx.x * 1.5f, 0};
            SetShaderValue(shBlur, locBlurStep, &sh, SHADER_UNIFORM_VEC2);
            blit(bl[i][1], shBlur, bl[i][0].texture);
            sh = (Vector2){0, tx.y * 1.5f};
            SetShaderValue(shBlur, locBlurStep, &sh, SHADER_UNIFORM_VEC2);
            blit(bl[i][0], shBlur, bl[i][1].texture);
        }
        Vector2 zero = {0, 0};
        SetShaderValue(shBlur, locBlurStep, &zero, SHADER_UNIFORM_VEC2);
    }

    // 3. Present to the screen.
    BeginShaderMode(shFinal);
    SetShaderValueTexture(shFinal, locFinal[0], bl[0][0].texture);
    SetShaderValueTexture(shFinal, locFinal[1], bl[1][0].texture);
    SetShaderValueTexture(shFinal, locFinal[2], bl[2][0].texture);
    SetShaderValue(shFinal, locFinal[3], &texel, SHADER_UNIFORM_VEC2);
    SetShaderValue(shFinal, locFinal[4], &e->exposure, SHADER_UNIFORM_FLOAT);
    int lb = GetShaderLocation(shFinal, "bloomStrength");
    SetShaderValue(shFinal, lb, &e->bloom, SHADER_UNIFORM_FLOAT);
    DrawTexturePro(hdr.texture, (Rectangle){0, 0, (float)curW, -(float)curH},
                   (Rectangle){0, 0, (float)GetScreenWidth(), (float)GetScreenHeight()}, (Vector2){0, 0}, 0, WHITE);
    EndShaderMode();
}

void post_shutdown(void) {
    free_targets();
    UnloadShader(shEmissive); UnloadShader(shAtmos); UnloadShader(shBright); UnloadShader(shBlur); UnloadShader(shFinal);
}
