# BeamLite

A small soft-body driving sandbox in the spirit of BeamNG.drive, written from scratch in C.
Two vehicles, picked with **V**: an angular stainless-steel electric pickup (a Cybertruck-style
wedge) and the Sparrow, a modern compact sport hatchback (unbadged, proportioned after a current
C-segment five-door).
Runs natively on macOS (Apple Silicon or Intel).

## Build & run (macOS)

```bash
xcode-select --install        # once, if you don't have the command line tools
./build.sh                    # uses Homebrew raylib if present, else builds it from source
./beamlite
```

(Optional: `brew install raylib` first and the build takes about 2 seconds.)

## Controls

| Key | Action |
|---|---|
| W / S or ↑ / ↓ | throttle / brake, reverse when stopped |
| A / D | steer |
| Space | handbrake (drift) |
| Mouse drag / wheel | orbit / zoom camera |
| C | dash cam: driver's seat (drag the mouse to look around) |
| B | beam view: the node/beam skeleton, colored by stress and permanent deformation |
| N | spawn a parked truck 20 m ahead (side-on, to T-bone) |
| X | clear spawned trucks |
| G | drop the tailgate |
| V | choose vehicle (←/→, Enter) |
| L | lighting: dusk / day |
| O | weather: dry / wet / rain / snow |
| Q | recover (fresh car, upright, in place) |
| R | reset to start |
| T / P | slow-mo / pause |
| K | clear skid marks |

The map has a launch ramp straight ahead, a crash-test wall behind you, concrete pillars,
parked trucks and a dirt washboard section on the cross road, and rolling hills past ~100 m.
Up the cross road to the left (+z) are two small towns with avenues, side streets and pavements:
houses, shops, apartment blocks, a couple of towers and warehouses, with saplings along the kerbs.
There's a warehouse district off a service road to the right, and forest covers the hills.
Saplings and young trees snap and fall when you hit them; mature trunks stop you dead.

## How it works

- **Vehicles** are entries in a definition table (`VehicleDef` in `physics.h`). Each entry sets the
  body's stations (up to 24) (position, floor, sill, beltline and roof heights, roof width, wheel arches,
  bulkheads, door seams, and which gaps are glass), plus widths, mass layout, structural zones and
  stiffness, wheels and suspension, engine, brakes, drive split, rear steer and lights. The same
  builder turns any entry into nodes, beams and skin.
  - *Stainless pickup*: 5.68 m, 3.1 t, ~600 hp dual-motor AWD, rear steer, detachable tailgate,
    0–100 km/h in 4.6 s.
  - *Sparrow hatchback*: 4.37 m long, 1.79 m wide, 1.45 m tall, 2.64 m wheelbase, 1.35 t,
    ~127 hp 2.0 L front-wheel drive, on 18" wheels. 21 stations: a sloping hood into a raked
    windshield, a roof that peaks over the front seats and falls to a spoiler over a raked hatch,
    a window line that kicks up into a pointed rear quarter window, round arches and flared rear
    haunches. Drawn with smooth-shaded clear-coat paint (no panel outlines, just shut lines),
    door mirrors, a roof spoiler, a black grille, intake and diffuser, and dark ten-spoke alloys.
    0–100 km/h in 7.8 s, 100–0 in 37 m.
- **Body**: 18 cross-section rings along a 5.68 m body, 9 nodes each (floor centre, floor
  edges, sills, beltline crease, roof/sail edge), shaped by the signature profile: one straight
  line from the nose up to the roof apex, another from the apex down to the tail. The rings form a
  monocoque shell (longitudinal members plus shear-panel diagonals) with full bulkheads at the
  nose, axles, cabin rear and tail, and lighter ones at the dash and B-pillar. Long rails run along
  the floor, beltline and roof. 168 nodes, about 1250 beams, 326 flat-shaded facets.
- **Zones**: soft crumple zones at both ends, a stiff stainless safety cell around the cabin,
  softer door skins, a stiff frame and battery floor, and reinforced suspension towers.
  Mass sits low: about 3.1 t, with the structural battery pack and drive units carried by the
  floor nodes.
- **Deformation**: past its yield point a beam's rest length follows the deformation, so dents
  stay. Beams stretched past 130% tear.
- **Glass**: the windshield, glass roof, rear glass and four side windows are separate panels.
  Each one shatters into shards when its frame is distorted by more than 3%.
- **Tailgate**: a separate rigid part. Hinge joints hold the bottom corners and latch joints hold
  the top; each snaps at its own strain limit. Check-strap cables catch it level when it drops,
  and rubber stops keep it from swinging into the bed. It can pop open in a rear hit or come off
  entirely.
- **Chassis**: raycast struts (spring + damper + bump stop) on 35" tyres with a friction-circle
  tyre model. Dual-motor AWD (~600 hp, 0–100 km/h in about 4.6 s), rear-wheel steering
  (up to ~10°, counter-steering at low speed) and a handbrake for drifting.
- **Collisions**: nodes collide with the terrain (heightfield) and boxes with friction.
  Truck-to-truck contact uses node spheres, checking only nodes inside the overlapping bounding boxes.
- **World**: the towns are generated from a fixed seed: lots are laid out along each block's
  street edges, then yards and the forest are filled in. Areas the headless test drives
  through are kept clear. Buildings collide as boxes; trees collide as trunk cylinders. A tree
  snaps once the impulse it takes in a short window passes a limit set by its trunk radius, then
  falls like a toppling rod in the direction it was hit. A uniform 6 m grid makes the collision
  checks cheap (about 70 buildings, about 1000 trees). Standing trees are baked into one mesh per
  50 m patch, which is rebuilt when a tree in it falls. The renderer bakes the buildings into one
  flat-shaded mesh, with windows, doors, shopfronts, gable roofs, chimneys and rooftop plant.
  The chase camera pulls in so buildings never block the view.
- **Rendering** (`src/render.c`): the world is drawn into a half-float HDR buffer with a depth
  texture. A post pass rebuilds each pixel's world position from depth, then draws the sky
  (gradient, sun, procedural clouds) behind everything and blends exponential height fog over it.
  Bright areas are blurred at three resolutions for bloom. Last comes edge-directed
  anti-aliasing, ACES tone mapping, a cool-shadow/warm-highlight colour grade, a vignette and
  dithering. Lighting is baked into vertex colours from the active preset's sun and ambient
  colours, so switching presets rebuilds those meshes. Lit windows, lamp lenses and the
  truck's light bars draw brighter than white, so they bloom. Street lamps, wall lamps over doors
  and the headlights add additive pools of light on walls and the road.
- **Wet roads**: the ground writes 1 into the scene buffer's alpha and everything else writes 0,
  so only real ground can be wet. Up-facing ground is classed as asphalt, pavement or other from
  the road layout, with procedural puddles. A screen-space reflection ray is marched through the
  depth buffer at 1/3 resolution, falling back to the sky on a miss. The result is blurred mostly
  vertically, the way wet asphalt streaks lights, and composited after darkening the wet surface.
  Damp asphalt gets a sheen; puddles are nearly mirrors. Rain adds falling streaks and fine
  ripples. Any wet road throws spray off the tyres, and costs about 15–20% of tyre grip.
- **Snow**: snow cover is baked into the terrain, roofs and tree tops, wherever a surface faces
  up. Ploughed roads keep less of it and steep rock sheds it. Snowflakes drift as soft sprites
  around the camera. The sky turns overcast with thicker haze, every rolling wheel leaves a track,
  and tyre grip halves (the Sparrow's 100–0 km/h stop goes from 37 m to 68 m).
- **Integration**: semi-implicit Euler at 1200 Hz fixed substeps. Each beam's stiffness and
  damping is limited by the masses it connects, which keeps the explicit integrator stable.

`tests/test_physics.c` is a headless test that runs acceleration, braking, cornering,
a tailgate drop, a 60 km/h wall crash, a ramp jump, a T-bone and hitting a sapling and a mature tree, then writes wireframe snapshots:
`cc -O2 tests/test_physics.c src/physics.c -lm -o tp && ./tp`

## Ideas for next steps

Real-time shadow maps; per-pixel lighting from nearby
lamps; node-beam wheels and tyres; opening doors and frunk; self-collision so crushed panels
stack up instead of passing through each other; slip-angle tyre model; Metal renderer; sound.
