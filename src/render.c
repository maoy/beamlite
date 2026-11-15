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
#define FAR_PLANE 9000.0

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

// Animated LED billboards. Original abstract content only: no real brands. Per-vertex data:
// texcoord = position on the screen in metres (x across, y up); colour r = seed, g = style,
// b = width / 64, a = height / 64. Each screen cycles through "spots" every 6-9 seconds.
static const char *BILLBOARD_FS = GLSL
    "uniform float time, intensity;\n"
    "float h1(float n) { return fract(sin(n * 12.9898) * 43758.5453); }\n"
    "float h2(vec2 p) { return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453); }\n"
    "vec3 pal(float t) { return 0.55 + 0.45 * cos(6.2831 * (t + vec3(0.0, 0.33, 0.67))); }\n"
    // A blocky 3x5 glyph, so rows of them read as text from a distance.
    "float glyph(vec2 c, float id) {\n"
    "  if (c.x < 0.12 || c.x > 0.88 || c.y < 0.1 || c.y > 0.9) return 0.0;\n"
    "  vec2 g = floor(vec2((c.x - 0.12) / 0.76 * 3.0, (c.y - 0.1) / 0.8 * 5.0));\n"
    "  return step(0.45, h2(g + id * 7.13));\n"
    "}\n"
    "float textRow(vec2 m, float h, float speed, float seed, float t) {\n"   // a row of glyphs of height h
    "  vec2 q = vec2(m.x + t * speed, m.y) / vec2(h * 0.62, h);\n"
    "  float id = floor(q.x) + seed * 31.0;\n"
    "  if (h2(vec2(id, seed)) < 0.18) return 0.0;\n"            // word gaps
    "  return glyph(fract(q), id);\n"
    "}\n"
    "void main() {\n"
    "  vec2 m = fragTexCoord;\n"
    "  float seed = floor(fragColor.r * 255.0 + 0.5), style = floor(fragColor.g * 255.0 + 0.5);\n"
    "  float W = fragColor.b * 64.0, H = fragColor.a * 64.0;\n"
    "  vec2 uv = m / vec2(W, H);\n"
    "  float t = time + seed * 3.7;\n"
    "  float period = 6.0 + 3.0 * h1(seed);\n"
    "  float spot = floor(t / period), ph = fract(t / period);\n"
    "  float k = h1(seed * 1.7 + spot * 3.1);\n"
    "  vec3 a = pal(k), b = pal(k + 0.3 + 0.2 * h1(spot)), col;\n"
    "  if (style < 0.5) {\n"                                    // brand spot: gradient, big shape, slogan
    "    col = mix(a * 0.35, b, uv.y);\n"
    "    vec2 c = vec2(0.30 + 0.40 * h1(spot + seed), 0.62);\n"
    "    vec2 d = (uv - c) * vec2(W / H, 1.0);\n"
    "    float shape = h1(spot * 5.0 + seed) < 0.5 ? length(d) - 0.24 : max(abs(d.x), abs(d.y)) - 0.2;\n"
    "    col = mix(col, vec3(1.0), smoothstep(0.02, -0.02, shape) * (0.75 + 0.25 * sin(t * 3.0)));\n"
    "    if (uv.y < 0.26) col = mix(vec3(0.03), b * 1.2, textRow(m, H * 0.16, 0.0, seed + spot, 0.0));\n"
    "  } else if (style < 1.5) {\n"                             // ticker: scrolling rows of text
    "    float row = floor(uv.y * 3.0);\n"
    "    vec3 inkc = row < 1.0 ? vec3(1.0, 0.75, 0.2) : row < 2.0 ? vec3(0.4, 1.0, 0.6) : vec3(0.9);\n"
    "    float on = textRow(vec2(m.x, mod(m.y, H / 3.0)), H / 3.0 * 0.7, 6.0 + row * 2.0, seed + row, t);\n"
    "    col = mix(vec3(0.02, 0.02, 0.05), inkc, on);\n"
    "  } else if (style < 2.5) {\n"                             // video wall: tiles of colour cycling and wiping
    "    vec2 tile = floor(m / 1.6);\n"
    "    float n = h2(tile + spot);\n"
    "    col = pal(n + t * 0.05) * (0.4 + 0.6 * step(0.5, fract(uv.x * 0.5 + uv.y * 0.3 - ph * 1.5)));\n"
    "    col = mix(col, a, 0.4);\n"
    "  } else {\n"                                               // product spot: sweeping diagonal bands and a flash
    "    float band = sin((uv.x * W + uv.y * H) * 0.35 - t * 2.5);\n"
    "    col = mix(a, b, 0.5 + 0.5 * band);\n"
    "    col *= 0.6 + 0.4 * smoothstep(0.6, 0.9, abs(band));\n"
    "    col += vec3(1.0) * smoothstep(0.96, 1.0, fract(t / period * 2.0)) * 0.8;\n"
    "    if (uv.y > 0.78) col = mix(col * 0.2, vec3(1.0), textRow(m - vec2(0.0, H * 0.78), H * 0.14, 0.0, seed * 2.0 + spot, 0.0));\n"
    "  }\n"
    "  vec2 px = fract(m * 3.0);\n"                                // LED pixel grid, and a dark bezel
    "  col *= 0.82 + 0.18 * step(0.25, px.x) * step(0.25, px.y);\n"
    "  float bez = step(0.15, m.x) * step(0.15, m.y) * step(m.x, W - 0.15) * step(m.y, H - 0.15);\n"
    "  finalColor = vec4(col * intensity * bez + vec3(0.02) * (1.0 - bez), 1.0);\n"
    "}\n";

// Vehicle paint: per-pixel lighting for everything drawn between post_paint_begin/end. Vertex colour
// rgb is the albedo; its alpha selects the material (the shader always writes alpha 1):
//   255 unlit (colour as given)   250 clear-coated paint   245 flame livery over the paint
//   240 chrome   235 glass   230 satin trim   225 matte (rubber, cloth)   220 brushed stainless
//   215 gloss paint (airliners)
// Diffuse light matches the baked vertex lighting of the world (ambient + sun, display space); on top
// come a Fresnel reflection of the procedural sky and ground and a sun highlight.
static const char *PAINT_VS =
    "#version 330\n"
    "in vec3 vertexPosition; in vec2 vertexTexCoord; in vec3 vertexNormal; in vec4 vertexColor;\n"
    "uniform mat4 mvp;\n"
    "out vec2 fragTexCoord; out vec4 fragColor; out vec3 fragPos; out vec3 fragNormal;\n"
    "void main() { fragTexCoord = vertexTexCoord; fragColor = vertexColor; fragPos = vertexPosition; fragNormal = vertexNormal;\n"
    "  gl_Position = mvp * vec4(vertexPosition, 1.0); }\n";
static const char *PAINT_FS = GLSL SKY_GLSL
    "in vec3 fragPos; in vec3 fragNormal;\n"
    "uniform vec3 bakeAmb, bakeSun, groundCol;\n"
    "uniform float lampGlow;\n"
    "vec3 envAt(vec3 r) {\n"                                    // what a mirror sees: sky above, the ground below
    "  vec3 up = sky(normalize(vec3(r.x, max(r.y, 0.0), r.z)), 0.0);\n"
    "  vec3 dn = mix(groundCol * 0.7, groundCol * 1.25 + horizonCol(r) * 0.22, smoothstep(-0.45, 0.0, r.y));\n"
    "  vec2 gp = r.xz / max(-r.y, 0.04);\n"                     // where the ray meets the ground: mottle it like road and verge
    "  dn *= 0.72 + 0.56 * noise(gp * 0.9 + 3.0) * smoothstep(0.0, -0.08, r.y);\n"
    "  float az = atan(r.z, r.x);\n"                             // a dark skyline (trees, roofs) along the horizon
    "  float skyline = 0.035 + 0.075 * noise(vec2(az * 9.0, 1.7)) + 0.05 * step(0.62, noise(vec2(az * 23.0, 8.3)));\n"
    "  up = mix(groundCol * 0.45, up, smoothstep(skyline - 0.012, skyline + 0.012, r.y));\n"
    "  dn += vec3(1.0, 0.72, 0.42) * lampGlow * (0.10 + 0.25 * smoothstep(0.90, 1.0, sin(r.x * 23.0) * sin(r.z * 19.0))) * smoothstep(-0.30, -0.02, r.y);\n"
    "  return mix(dn, up, smoothstep(-0.02, 0.03, r.y)); }\n"
    "vec3 envRough(vec3 r, float rough) {\n"
    "  vec3 a = envAt(r);\n"
    "  if (rough < 0.12) return a;\n"
    "  vec3 avg = mix(groundCol, 0.5 * (zenith + horizonAway), smoothstep(-0.6, 0.6, r.y));\n"
    "  return mix(a, avg, clamp(rough * 1.6, 0.0, 1.0)); }\n"
    "vec3 flames(vec2 t, vec3 base) {\n"                        // t.x: metres behind the nose; t.y: height around the body
    "  float s = t.x + 0.10 * sin(t.y * 7.0 + t.x * 2.3);\n"
    "  float w = t.y * 2.3 + 0.30 * sin(t.x * 1.3 + t.y * 2.0);\n"
    "  float id = floor(w), lane = fract(w);\n"
    "  float tri = 1.0 - abs(lane * 2.0 - 1.0);\n"
    "  float len = 1.2 + 3.0 * hash(vec2(id, 7.3));\n"
    "  float d = 0.70 + len * pow(tri, 0.75) - s;\n"            // > 0 inside a tongue of flame
    "  vec3 fl = mix(vec3(0.95, 0.36, 0.05), vec3(0.74, 0.04, 0.03), smoothstep(0.0, 1.3, d));\n"
    "  vec3 c = mix(base, fl, smoothstep(0.0, 0.025, d));\n"
    "  float w2 = t.y * 3.1 + 1.7 + 0.25 * sin(t.x * 1.9);\n"   // ghost flames further back, tone on tone
    "  float d2 = 2.2 + (1.0 + 2.5 * hash(vec2(floor(w2), 3.1))) * pow(1.0 - abs(fract(w2) * 2.0 - 1.0), 0.8) - s;\n"
    "  c = mix(c, c * 1.5 + vec3(0.02, 0.03, 0.10), smoothstep(0.0, 0.03, d2) * (1.0 - smoothstep(0.0, 0.025, d)) * 0.55);\n"
    "  return mix(c, vec3(1.0, 0.72, 0.22), smoothstep(0.022, 0.006, abs(d - 0.012)) * 0.85); }\n"
    "void main() {\n"
    "  float m = fragColor.a * 255.0;\n"
    "  if (m > 252.5) { finalColor = vec4(fragColor.rgb, 1.0); return; }\n"
    "  vec3 N = normalize(fragNormal), V = normalize(camPos - fragPos);\n"
    "  if (dot(N, V) < 0.0) N = -N;\n"
    "  vec3 alb = fragColor.rgb;\n"
    "  float metal = 0.0, rough = 0.6, coat = 0.0, f0 = 0.04;\n"
    "  if (m > 247.5) { coat = 1.0; f0 = 0.05; }\n"
    "  else if (m > 242.5) { alb = flames(fragTexCoord, alb); coat = 1.0; f0 = 0.06; }\n"
    "  else if (m > 237.5) { metal = 1.0; rough = 0.04; }\n"
    "  else if (m > 232.5) { coat = 1.0; f0 = 0.09; }\n"
    "  else if (m > 227.5) { coat = 0.30; rough = 0.5; }\n"
    "  else if (m > 222.5) { coat = 0.0; }\n"
    "  else if (m > 217.5) { metal = 1.0; rough = 0.30; }\n"
    "  else { coat = 0.60; }\n"
    "  float ndl = max(dot(N, sunDir), 0.0), ndv = max(dot(N, V), 0.0);\n"
    "  vec3 light = bakeAmb + 0.10 * max(N.y, 0.0) * vec3(1.0, 1.0, 1.2) + bakeSun * ndl;\n"
    "  vec3 lin = pow(alb * light, vec3(2.2)) * (1.0 - metal);\n"
    "  vec3 R = reflect(-V, N);\n"
    "  float fr = pow(1.0 - ndv, 5.0);\n"
    "  vec3 H = normalize(V + sunDir);\n"
    "  float ndh = max(dot(N, H), 0.0);\n"
    "  if (metal > 0.5) {\n"
    "    vec3 tint = mix(pow(alb, vec3(2.2)), vec3(1.0), fr);\n"
    "    float shin = mix(900.0, 40.0, clamp(rough * 3.0, 0.0, 1.0));\n"
    "    lin = envRough(R, rough) * tint * (rough > 0.12 ? 0.80 : 0.92) + sunCol * tint * pow(ndh, shin) * (rough > 0.12 ? 0.30 : 6.0) * step(0.0, sunDir.y + 0.2);\n"
    "  } else {\n"
    "    float F = (f0 + (1.0 - f0) * fr) * coat;\n"
    "    lin = lin * (1.0 - F) + envRough(R, coat > 0.9 ? 0.0 : 0.35) * F;\n"
    "    lin += sunCol * (pow(ndh, 420.0) * 2.2 * coat + pow(ndh, 24.0) * 0.05) * ndl;\n"
    "  }\n"
    "  finalColor = vec4(pow(max(lin, 0.0), vec3(1.0 / 2.2)), 1.0);\n"
    "}\n";

// The bay: a flat sheet whose normal is rippled by scrolling noise. Fresnel blend between deep water
// and the reflected sky, plus sun glitter. Ripples flatten with distance so the far water doesn't shimmer.
static const char *WATER_FS = GLSL SKY_GLSL
    "in vec3 fragPos; in vec3 fragNormal;\n"
    "uniform vec3 waterCol;\n"
    "float wave(vec2 p) { return noise(p * 0.22 + vec2(time * 0.30, time * 0.17)) + 0.5 * noise(p * 0.71 - vec2(time * 0.42, time * 0.26)); }\n"
    "vec3 skyLite(vec3 r) {\n"                                 // the sky without its clouds: cheap enough for a whole sea
    "  float sd = max(dot(r, sunDir), 0.0);\n"
    "  return mix(horizonCol(r), zenith, pow(clamp(r.y, 0.0, 1.0), 0.5)) + sunCol * (pow(sd, 6.0) * 0.30 + pow(sd, 90.0) * 0.9); }\n"
    "void main() {\n"
    "  vec3 V = camPos - fragPos; float dist = length(V); V /= dist;\n"
    "  vec2 p = fragPos.xz;\n"
    "  float e = 0.4, amp = 1.1 / (1.0 + dist * 0.006);\n"
    "  float h0 = 0.75; vec3 N = vec3(0.0, 1.0, 0.0);\n"
    "  if (dist < 1800.0) {\n"                                 // ripples only where they can be seen
    "    h0 = wave(p);\n"
    "    N = normalize(vec3((h0 - wave(p + vec2(e, 0.0))) * amp, e, (h0 - wave(p + vec2(0.0, e))) * amp));\n"
    "    h0 = mix(h0, 0.75, smoothstep(1200.0, 1800.0, dist));\n"
    "  }\n"
    "  vec3 R = reflect(-V, N); R.y = abs(R.y) + 0.02; R = normalize(R);\n"
    "  float F = 0.03 + 0.97 * pow(1.0 - max(dot(N, V), 0.0), 5.0);\n"
    "  vec3 col = mix(waterCol * (0.7 + 0.6 * h0), skyLite(R), F);\n"
    "  col += sunCol * pow(max(dot(R, sunDir), 0.0), 220.0) * 3.0;\n"
    "  finalColor = vec4(pow(max(col, 0.0), vec3(1.0 / 2.2)), 1.0);\n"
    "}\n";

// Shadow-map depth pass: any colour will do.
static const char *DEPTH_FS = "#version 330\nout vec4 finalColor;\nvoid main() { finalColor = vec4(1.0); }\n";

// Ambient occlusion from the depth buffer (half resolution, blurred afterwards).
static const char *AO_FS = GLSL SKY_GLSL
    "float linz(float d) { float z = d * 2.0 - 1.0; return 2.0 * 0.1 * 9000.0 / (9000.1 - z * 8999.9); }\n"
    "void main() {\n"
    "  vec2 uv = fragTexCoord;\n"
    "  float d = texture(depthTex, uv).r;\n"
    "  if (d >= 0.999999) { finalColor = vec4(1.0); return; }\n"
    "  vec3 P = worldAt(uv, d);\n"
    "  float dist = length(P - camPos);\n"
    "  if (dist > 160.0) { finalColor = vec4(1.0); return; }\n"
    "  vec3 N = normalize(cross(dFdx(P), dFdy(P)));\n"
    "  if (dot(N, P - camPos) > 0.0) N = -N;\n"
    "  vec3 T = normalize(abs(N.y) < 0.9 ? cross(N, vec3(0, 1, 0)) : cross(N, vec3(1, 0, 0))), B = cross(N, T);\n"
    "  float ang = 6.2831 * fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));\n"
    "  float ca = cos(ang), sa = sin(ang);\n"
    "  float radius = clamp(dist * 0.06, 0.35, 1.4), occ = 0.0;\n"
    "  for (int i = 0; i < 8; i++) {\n"
    "    float fi = float(i) + 0.5;\n"
    "    float r = sqrt(fi / 8.0), th = fi * 2.399963;\n"      // golden-angle spiral on the disc, lifted into the hemisphere
    "    vec2 k = r * vec2(cos(th), sin(th));\n"
    "    k = vec2(k.x * ca - k.y * sa, k.x * sa + k.y * ca);\n"
    "    float kz = sqrt(max(1.0 - r * r, 0.0));\n"
    "    vec3 S = P + N * 0.04 + (T * k.x + B * k.y + N * kz) * radius * (0.35 + 0.65 * fi / 8.0);\n"
    "    vec4 c = VP * vec4(S, 1.0);\n"
    "    if (c.w <= 0.0) continue;\n"
    "    vec3 n = c.xyz / c.w; vec2 su = n.xy * 0.5 + 0.5;\n"
    "    if (su.x < 0.0 || su.x > 1.0 || su.y < 0.0 || su.y > 1.0) continue;\n"
    "    float sceneZ = linz(texture(depthTex, su).r), sampZ = linz(n.z * 0.5 + 0.5);\n"
    "    float diff = sampZ - sceneZ;\n"
    "    occ += step(0.04, diff) * smoothstep(0.0, 1.0, radius * 1.5 / max(diff, 1e-3));\n"
    "  }\n"
    "  float ao = 1.0 - occ / 8.0;\n"
    "  ao = mix(ao, 1.0, smoothstep(90.0, 160.0, dist));\n"
    "  finalColor = vec4(ao, ao, ao, 1.0);\n"
    "}\n";

static const char *ATMOS_FS = GLSL SKY_GLSL
    "uniform sampler2D shadowMap, aoTex;\n"
    "uniform mat4 lightVP;\n"
    "uniform vec3 shadowAmb, shadowSun;\n"
    "uniform float shadowOn, aoOn, shadowTexel;\n"
    "void main() {\n"
    "  vec2 uv = fragTexCoord;\n"
    "  float d = texture(depthTex, uv).r;\n"
    "  vec3 w = worldAt(uv, d);\n"
    "  vec3 r = normalize(w - camPos);\n"
    "  if (d >= 0.999999) { finalColor = vec4(sky(r, 1.0), 0.0); return; }\n"
    "  vec4 sc = texture(texture0, uv);\n"
    "  bool glow = max(sc.r, max(sc.g, sc.b)) > 1.02;\n"       // emissive things are not shaded
    "  if (shadowOn > 0.5 && !glow) {\n"
    // Sun shadows, deferred: the world's lighting is baked (ambient + sun) into its colours, so where the
    // shadow map says the sun is blocked, scale back to the ambient-only share.
    "    vec3 Ng = normalize(cross(dFdx(w), dFdy(w)));\n"
    "    if (dot(Ng, r) > 0.0) Ng = -Ng;\n"
    "    float ndl = dot(Ng, sunDir);\n"
    "    if (ndl > 0.02) {\n"
    "      vec4 lp = lightVP * vec4(w + Ng * 0.09 + sunDir * 0.05, 1.0);\n"
    "      vec3 q = lp.xyz / lp.w * 0.5 + 0.5;\n"
    "      if (q.x > 0.0 && q.x < 1.0 && q.y > 0.0 && q.y < 1.0 && q.z < 1.0) {\n"
    "        float ang = 6.2831 * fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));\n"
    "        float lit = 0.0;\n"
    "        for (int i = 0; i < 6; i++) {\n"
    "          float fi = float(i) + 0.5, rr = sqrt(fi / 6.0) * 1.6, th = fi * 2.399963 + ang;\n"
    "          lit += step(q.z - 0.00005, texture(shadowMap, q.xy + rr * shadowTexel * vec2(cos(th), sin(th))).r);\n"
    "        }\n"
    "        lit /= 6.0;\n"
    "        vec2 e = smoothstep(0.0, 0.06, q.xy) * (1.0 - smoothstep(0.94, 1.0, q.xy));\n"
    "        lit = mix(1.0, lit, e.x * e.y);\n"
    "        vec3 a = shadowAmb + 0.10 * max(Ng.y, 0.0) * vec3(1.0, 1.0, 1.2);\n"
    "        sc.rgb *= mix(a / (a + shadowSun * ndl), vec3(1.0), lit);\n"
    "      }\n"
    "    }\n"
    "  }\n"
    "  vec3 col = pow(max(sc.rgb, 0.0), vec3(2.2));\n"   // scene colours are authored in sRGB
    "  if (aoOn > 0.5 && !glow) col *= mix(1.0, texture(aoTex, uv).r, 0.85);\n"
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
    "uniform vec4 roads[32];\n"
    "uniform int nRoads, nHighway;\n"
    "uniform vec4 broadway;\n"                                   // x0 at z0, dx/dz, horizontal half-width, z0 ...
    "uniform vec2 broadwayZ;\n"                                  // ... and its z range (empty range = none)
    "float linz(float d) { float z = d * 2.0 - 1.0; return 2.0 * 0.1 * 9000.0 / (9000.1 - z * 8999.9); }\n"
    "float surface(vec2 p) {\n"      // 1 asphalt, 0.6 pavement, 0.22 anything else
    "  float s = 0.10;\n"
    "  if (p.y > broadwayZ.x && p.y < broadwayZ.y && abs(p.x - broadway.x - broadway.y * (p.y - broadway.w)) < broadway.z) return 0.9;\n"
    "  for (int i = 0; i < 32; i++) {\n"
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

static Shader shBillboard;
static int locBbTime, locBbInt;
static Shader shEmissive, shAtmos, shSSR, shComposite, shBright, shBlur, shFinal;
static int locIntensity, locAtmos[U_COUNT], locSSR[U_COUNT], locWet, locRain, locRoads, locNRoads, locNHighway, locRefl;
static int locBrightTexel, locBrightThr, locBlurStep, locFinal[6];
static RenderTexture2D scene, hdr, hdr2, refl[2], bl[3][2], aoRT[2];
static Shader shPaint, shDepth, shAO, shWater;
static int locWater[U_COUNT], locWaterCol;
static int locPaint[U_COUNT], locPaintAmb, locPaintSun, locPaintGround, locPaintLamp, locAO[U_COUNT];
static int locShMap, locShVP, locShAmb, locShSun, locShOn, locShTexel, locAoTex, locAoOn;
#define SHADOW_RES 2048
#define SHADOW_HALF 60.0f                       // the shadow map covers this far either side of its focus
static RenderTexture2D shadowRT;
static Matrix lightVP;
static int shadowReady, shadowDrawn, aoEnabled = 1, shadowsEnabled = 1;
static Vector3 shadowAmb, shadowSun;
static int curW, curH;
static Matrix invVP, VP;
static Vector3 camPos;
static float roadRects[32 * 4];
static float bwParams[4], bwZ[2];
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
    UnloadRenderTexture(aoRT[0]); UnloadRenderTexture(aoRT[1]);
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
    aoRT[0] = make_rt(w / 2, h / 2, 0);
    aoRT[1] = make_rt(w / 2, h / 2, 0);
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
    shBillboard = LoadShaderFromMemory(NULL, BILLBOARD_FS);
    locBbTime = GetShaderLocation(shBillboard, "time");
    locBbInt = GetShaderLocation(shBillboard, "intensity");
    shAtmos = LoadShaderFromMemory(NULL, ATMOS_FS);
    shSSR = LoadShaderFromMemory(NULL, SSR_FS);
    shComposite = LoadShaderFromMemory(NULL, COMPOSITE_FS);
    shBright = LoadShaderFromMemory(NULL, BRIGHT_FS);
    shBlur = LoadShaderFromMemory(NULL, BLUR_FS);
    shFinal = LoadShaderFromMemory(NULL, FINAL_FS);
    locIntensity = GetShaderLocation(shEmissive, "intensity");
    shPaint = LoadShaderFromMemory(PAINT_VS, PAINT_FS);
    shDepth = LoadShaderFromMemory(NULL, DEPTH_FS);
    shAO = LoadShaderFromMemory(NULL, AO_FS);
    shWater = LoadShaderFromMemory(PAINT_VS, WATER_FS);
    locWaterCol = GetShaderLocation(shWater, "waterCol");
    for (int i = 0; i < U_COUNT; i++) locWater[i] = GetShaderLocation(shWater, SKY_UNIFORMS[i]);
    locPaintAmb = GetShaderLocation(shPaint, "bakeAmb");
    locPaintSun = GetShaderLocation(shPaint, "bakeSun");
    locPaintGround = GetShaderLocation(shPaint, "groundCol");
    locPaintLamp = GetShaderLocation(shPaint, "lampGlow");
    locShMap = GetShaderLocation(shAtmos, "shadowMap");
    locShVP = GetShaderLocation(shAtmos, "lightVP");
    locShAmb = GetShaderLocation(shAtmos, "shadowAmb");
    locShSun = GetShaderLocation(shAtmos, "shadowSun");
    locShOn = GetShaderLocation(shAtmos, "shadowOn");
    locShTexel = GetShaderLocation(shAtmos, "shadowTexel");
    locAoTex = GetShaderLocation(shAtmos, "aoTex");
    locAoOn = GetShaderLocation(shAtmos, "aoOn");
    {   // shadow map: a depth texture (plus a token colour attachment so the framebuffer is complete)
        shadowRT.id = rlLoadFramebuffer();
        rlEnableFramebuffer(shadowRT.id);
        shadowRT.texture.id = rlLoadTexture(NULL, SHADOW_RES, SHADOW_RES, PIXELFORMAT_UNCOMPRESSED_GRAYSCALE, 1);
        shadowRT.texture.width = shadowRT.texture.height = SHADOW_RES; shadowRT.texture.mipmaps = 1;
        shadowRT.texture.format = PIXELFORMAT_UNCOMPRESSED_GRAYSCALE;
        rlFramebufferAttach(shadowRT.id, shadowRT.texture.id, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
        shadowRT.depth.id = rlLoadTextureDepth(SHADOW_RES, SHADOW_RES, false);
        shadowRT.depth.width = shadowRT.depth.height = SHADOW_RES; shadowRT.depth.mipmaps = 1;
        rlFramebufferAttach(shadowRT.id, shadowRT.depth.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
        shadowReady = rlFramebufferComplete(shadowRT.id);
        if (!shadowReady) TraceLog(LOG_WARNING, "post: shadow framebuffer incomplete");
        rlDisableFramebuffer();
        SetTextureWrap(shadowRT.depth, TEXTURE_WRAP_CLAMP);
    }
    for (int i = 0; i < U_COUNT; i++) {
        locPaint[i] = GetShaderLocation(shPaint, SKY_UNIFORMS[i]);
        locAO[i] = GetShaderLocation(shAO, SKY_UNIFORMS[i]);
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

void post_set_broadway(float x0, float dxdz, float halfX, float z0, float zmin, float zmax) {
    bwParams[0] = x0; bwParams[1] = dxdz; bwParams[2] = halfX; bwParams[3] = z0; bwZ[0] = zmin; bwZ[1] = zmax;
}

void post_set_roads(const float (*rects)[4], int n, int nHighway) {
    if (n > 32) n = 32;
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
Shader post_billboard_shader(void) { return shBillboard; }
void post_set_billboards(float time, float intensity) {
    SetShaderValue(shBillboard, locBbTime, &time, SHADER_UNIFORM_FLOAT);
    SetShaderValue(shBillboard, locBbInt, &intensity, SHADER_UNIFORM_FLOAT);
}

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
    if (loc[U_DEPTH] >= 0) SetShaderValueTexture(sh, loc[U_DEPTH], scene.depth);
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

// Vehicle paint shader: wrap immediate-mode drawing whose vertices carry normals and a material alpha.
void post_paint_begin(const Env *e, Vector3 bakeAmb, Vector3 bakeSun, float time) {
    BeginShaderMode(shPaint);
    set_sky_uniforms(shPaint, locPaint, e, time);
    SetShaderValue(shPaint, locPaintAmb, &bakeAmb, SHADER_UNIFORM_VEC3);
    SetShaderValue(shPaint, locPaintSun, &bakeSun, SHADER_UNIFORM_VEC3);
    Vector3 sd = Vector3Normalize(e->sunDir);
    float up = sd.y > 0 ? sd.y : 0;
    // The ground as a mirror sees it: mid-grey asphalt/grass under the baked light, linear.
    Vector3 g = {powf(0.44f * (bakeAmb.x + bakeSun.x * up), 2.2f), powf(0.45f * (bakeAmb.y + bakeSun.y * up), 2.2f), powf(0.43f * (bakeAmb.z + bakeSun.z * up), 2.2f)};
    SetShaderValue(shPaint, locPaintGround, &g, SHADER_UNIFORM_VEC3);
    float lamp = e->lights * 0.25f;
    SetShaderValue(shPaint, locPaintLamp, &lamp, SHADER_UNIFORM_FLOAT);
}
void post_paint_end(void) { EndShaderMode(); }

// Water: draw the sea surface (world-space vertices) between these.
void post_water_begin(const Env *e, Vector3 bakeAmb, Vector3 bakeSun, float time) {
    BeginShaderMode(shWater);
    set_sky_uniforms(shWater, locWater, e, time);
    Vector3 sd = Vector3Normalize(e->sunDir);
    float up = sd.y > 0 ? sd.y : 0;
    float k = bakeAmb.y + bakeSun.y * up;
    Vector3 wc = {0.010f * k, 0.060f * k, 0.085f * k};
    SetShaderValue(shWater, locWaterCol, &wc, SHADER_UNIFORM_VEC3);
}
void post_water_end(void) { EndShaderMode(); }

// Sun shadow map: draw the shadow casters between these two. The map is an orthographic view down the
// sun's rays, SHADOW_HALF metres either side of `focus`, snapped to whole texels so it doesn't shimmer.
int post_shadow_begin(Vector3 focus, const Env *e, Vector3 bakeAmb, Vector3 bakeSun) {
    static int tick;
    static const Env *lastEnv;
    shadowAmb = bakeAmb; shadowSun = bakeSun;
    if (!shadowReady || !shadowsEnabled) { shadowDrawn = 0; return 0; }
    // Refresh the map every other frame; in between, the previous map (and its matrix) still holds.
    if (shadowDrawn && lastEnv == e && (tick++ & 1)) return 0;
    lastEnv = e;
    Vector3 L = Vector3Normalize(e->sunDir);
    Vector3 up = fabsf(L.y) > 0.95f ? (Vector3){1, 0, 0} : (Vector3){0, 1, 0};
    Vector3 rx = Vector3Normalize(Vector3CrossProduct(up, L)), ry = Vector3CrossProduct(L, rx);
    float texel = 2.0f * SHADOW_HALF / SHADOW_RES;
    float fx = floorf(Vector3DotProduct(focus, rx) / texel) * texel, fy = floorf(Vector3DotProduct(focus, ry) / texel) * texel;
    float fz = Vector3DotProduct(focus, L);
    Vector3 f = Vector3Add(Vector3Add(Vector3Scale(rx, fx), Vector3Scale(ry, fy)), Vector3Scale(L, fz));
    Camera3D lc = {0};
    lc.position = Vector3Add(f, Vector3Scale(L, 600.0f));
    lc.target = f;
    lc.up = up;
    lc.fovy = 2.0f * SHADOW_HALF;
    lc.projection = CAMERA_ORTHOGRAPHIC;
    BeginTextureMode(shadowRT);
    ClearBackground(WHITE);
    rlSetClipPlanes(1.0, 1500.0);                       // a short depth range keeps the shadow map precise
    BeginMode3D(lc);
    lightVP = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
    rlDisableBackfaceCulling();
    BeginShaderMode(shDepth);
    shadowDrawn = 1;
    return 1;
}
void post_shadow_end(void) {
    EndShaderMode();
    rlEnableBackfaceCulling();
    EndMode3D();
    EndTextureMode();
    rlSetClipPlanes(NEAR_PLANE, FAR_PLANE);
}
int post_shadow_active(void) { return shadowDrawn; }
Shader post_depth_shader(void) { return shDepth; }
void post_set_quality(int shadows, int ao) { shadowsEnabled = shadows; aoEnabled = ao; }

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

    // 0. Ambient occlusion at half resolution, blurred.
    if (aoEnabled) {
        BeginTextureMode(aoRT[0]);
        BeginShaderMode(shAO);
        set_sky_uniforms(shAO, locAO, e, time);
        DrawTexturePro(scene.texture, full, (Rectangle){0, 0, (float)aoRT[0].texture.width, (float)aoRT[0].texture.height}, (Vector2){0, 0}, 0, WHITE);
        EndShaderMode();
        EndTextureMode();
        blur_pass(aoRT[0], aoRT[1], 1.0f, 1.0f);
    }

    // 1. Atmosphere (alpha written as-is: it carries the ground mask), with sun shadows and AO.
    BeginTextureMode(hdr);
    rlSetBlendFactors(1, 0, 0x8006);
    rlSetBlendMode(BLEND_CUSTOM);
    BeginShaderMode(shAtmos);
    set_sky_uniforms(shAtmos, locAtmos, e, time);
    float shOn = shadowDrawn ? 1.0f : 0.0f, aoOn = aoEnabled ? 1.0f : 0.0f, shTexel = 1.0f / SHADOW_RES;
    SetShaderValueTexture(shAtmos, locShMap, shadowRT.depth);
    SetShaderValueTexture(shAtmos, locAoTex, aoRT[0].texture);
    SetShaderValueMatrix(shAtmos, locShVP, lightVP);
    SetShaderValue(shAtmos, locShAmb, &shadowAmb, SHADER_UNIFORM_VEC3);
    SetShaderValue(shAtmos, locShSun, &shadowSun, SHADER_UNIFORM_VEC3);
    SetShaderValue(shAtmos, locShOn, &shOn, SHADER_UNIFORM_FLOAT);
    SetShaderValue(shAtmos, locShTexel, &shTexel, SHADER_UNIFORM_FLOAT);
    SetShaderValue(shAtmos, locAoOn, &aoOn, SHADER_UNIFORM_FLOAT);
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
        SetShaderValue(shSSR, GetShaderLocation(shSSR, "broadway"), bwParams, SHADER_UNIFORM_VEC4);
        SetShaderValue(shSSR, GetShaderLocation(shSSR, "broadwayZ"), bwZ, SHADER_UNIFORM_VEC2);
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
        blur_pass(bl[i][0], bl[i][1], 2.1f, 2.1f);       // one pass at sqrt(2) the step = two passes at 1.5, half the draw calls
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
    UnloadShader(shEmissive); UnloadShader(shBillboard); UnloadShader(shAtmos); UnloadShader(shSSR); UnloadShader(shComposite);
    UnloadShader(shBright); UnloadShader(shBlur); UnloadShader(shFinal);
    UnloadShader(shPaint); UnloadShader(shDepth); UnloadShader(shAO); UnloadShader(shWater);
    if (shadowReady) UnloadRenderTexture(shadowRT);
}
