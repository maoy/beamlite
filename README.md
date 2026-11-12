# BeamLite

A small soft-body driving sandbox in the spirit of BeamNG.drive, written from scratch in C.
You drive an angular stainless-steel electric pickup (a Cybertruck-style wedge).
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
| C | hood camera |
| B | beam view: the node/beam skeleton, colored by stress and permanent deformation |
| N | spawn a parked truck 20 m ahead (side-on, to T-bone) |
| X | clear spawned trucks |
| G | drop the tailgate |
| Q | recover (fresh car, upright, in place) |
| R | reset to start |
| T / P | slow-mo / pause |
| K | clear skid marks |

The map has a launch ramp straight ahead, a crash-test wall behind you, concrete pillars,
parked trucks and a dirt washboard section on the cross road, and rolling hills past ~100 m.

## How it works

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
- **Integration**: semi-implicit Euler at 1200 Hz fixed substeps. Each beam's stiffness and
  damping is limited by the masses it connects, which keeps the explicit integrator stable.

`tests/test_physics.c` is a headless test that runs acceleration, braking, cornering,
a tailgate drop, a 60 km/h wall crash, a ramp jump and a T-bone, then writes wireframe snapshots:
`cc -O2 tests/test_physics.c src/physics.c -lm -o tp && ./tp`

## Ideas for next steps

Node-beam wheels and tyres; opening doors and frunk; self-collision so crushed panels
stack up instead of passing through each other; slip-angle tyre model; Metal renderer; sound.
