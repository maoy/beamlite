# BeamLite

A small soft-body driving sandbox in the spirit of BeamNG.drive, written from scratch in C.
Three vehicles, picked with **V**: an angular stainless-steel electric pickup (a Cybertruck-style
wedge), the Sparrow, a modern compact sport hatchback (unbadged, proportioned after a current
C-segment five-door), and a long-nose semi tractor in a red-and-blue flame livery (unbadged).
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
| C | dash cam from the driver's seat: dashboard, wheel, mirror, seats, pillars (drag to look around) |
| B | beam view: the node/beam skeleton, colored by stress and permanent deformation |
| N | spawn a parked truck 20 m ahead (side-on, to T-bone) |
| X | clear spawned trucks |
| G | drop the tailgate |
| V | choose vehicle (←/→, Enter) |
| M | map: Default / New York / Airport |
| J | traffic on / off |
| L | lighting: dusk / day |
| F2 | sun shadows + ambient occlusion on / off |
| Tab | Airport map: fly the airliner on the runway / back to the car |

Flying (after **Tab** on the Airport map):

| Key | Action |
|---|---|
| Shift / Ctrl | throttle up / down |
| S / W (or ↓ / ↑) | pitch up / down: pull back to climb |
| A / D (or ← / →) | roll left / right |
| Q / E | rudder, and nose-wheel steering on the ground |
| F | flaps: up / take-off / full |
| Space | wheel brakes |
| C | nose camera |
| R | put the airliner back on the runway |

To take off: flaps to take-off, full throttle, keep it on the centreline with Q/E, and pull back
gently at about 135 kt. It lifts off after roughly 580 m. Let go of the keys and it holds wings
level and its pitch attitude. To land: flaps full, about 140 kt, throttle to idle, brake.
| O | weather: dry / wet / rain / snow |
| Q | recover (fresh car, upright, in place) |
| R | reset to start |
| T / P | slow-mo / pause |
| K | clear skid marks |

Three maps, switched with **M**. All share the proving ground: a launch ramp straight ahead, a
crash-test wall behind you, concrete pillars, a warehouse district off a service road to the right,
and forest on the hills past ~100 m. Saplings and young trees snap when you hit them; mature
trunks stop you dead.

- **Default**: two small towns up the cross road to the left (+z), with avenues, side streets and
  pavements: houses, shops, apartment blocks, a couple of towers and warehouses, with saplings
  along the kerbs, and a dirt washboard section on the cross road.
- **New York**: Midtown, with the Times Square "bowtie" where Broadway crosses 7th Avenue on a
  diagonal (you start facing up 7th Avenue toward it). Towers of 55–140 m fill the blocks wall to
  wall. A screen-clad wedge tower stands in the south gore and red grandstand steps on the
  pedestrian plaza. Animated LED billboards cover the facades around the square: original abstract
  content (tickers, video walls, colour-cycling spots), no real brands. They spill coloured light
  onto the street and smear across the wet road. There are zebra crossings, lane lines, street
  lamps and yellow cabs parked on 7th Avenue. The washboard moves south of the highway.
- **Airport**: you start on the access road heading north toward a glass terminal, with a
  departures forecourt under a canopy, taxis at the kerb, and two car parks. Past the terminal,
  airliners sit nose-in at the gates with jet bridges, a control tower glows over the apron, and
  floodlight masts light it. Further north are two hangars (one with an airliner outside) and
  runway 09/27. Like the runways at SFO, it runs 1.5 km out into a bay on a causeway, with piano keys,
  touchdown bars, numbers, edge and threshold lights and approach lights. A blue-lit taxiway runs
  alongside. The bay lies north and east of the field. To the south and west the land rises into
  coastal hills. The map reaches 6 km from the field in every direction, so there is room to fly.
  An airliner waits lined up on runway 09: press **Tab** to fly it (see below). The parked airliners
  are soft bodies and the buildings are solid. The field is graded flat, and this map has no washboard.

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
  - *Long-nose semi*: a classic conventional tractor with a sleeper, bobtail. 8.25 m long, 3.0 m
    tall, 8.2 t, ~600 hp, three axles with tandem drive: six wheels in the physics, duals on the
    rear. The node/beam body is the narrow hood, cab, sleeper and bare frame. The chrome bumper,
    barred grille, twin headlamps, fenders, air cleaners, exhaust stacks, fuel tanks, visor, mirrors
    and fifth wheel are drawn on those nodes, so they move with the damage but don't collide
    themselves. The flames are a procedural pattern in the paint shader: red-orange tongues with a
    pinstripe, licking back from the nose over deep blue, with tone-on-tone ghost flames behind.
    0–100 km/h in 8.2 s, 100–0 in 36 m.
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
- **Traffic** (`src/traffic.c`): each map's streets form a grid of junctions. Signals alternate
  between the two axes, with 2.5 s of all-red clearance and offsets staggered between junctions.
  About 8 AI cars drive the grid on the same soft-body physics as the player. They stay in the
  right-hand lane, follow curved paths through junctions (swinging wide on right turns, and turning
  left only where nothing else goes), and slow for corners. They stop for red, and for yellow when
  they can stop comfortably, and keep a gap to whatever is ahead, the player included. Wrecked,
  flipped, off-route or deadlocked cars respawn out of sight. At the airport the landside roads
  are unsignalised, and cars just loop around the terminal. The highway past the ramp and
  through the crash wall is left out of the routes.
- **Airliners**: each parked airliner is a ~40 t node/beam airframe of 246 nodes and 1560 beams,
  resting on its landing gear. The fuselage is a tube of 10-node frames joined by stringers and
  shear diagonals. Wings and stabilisers are two-layer boxes, and the engines hang on pylons. Every
  beam runs at the integrator's stiffness limit for its node masses, so the airframe carries its own
  weight (the tail droops a few centimetres) but dents, bends and yields where you hit it.
  Car nodes collide with the airframe's skin triangles as well as its nodes, so nothing slips
  through the gaps of the coarse structure. An airframe settles when the map loads, sleeps at zero
  cost, and wakes when a car comes within a metre. The beam view (**B**) shows its structure.
- **Flight**: the airliner flies the way BeamNG's aircraft do. There is no rigid-body flight model,
  only forces on nodes. Each lifting surface (three panels per wing, the centre section, both
  stabilisers and the fin) is a quad of nodes. Every substep it reads its own velocity through the
  air and its own angle of attack, and pushes along its normal at its quarter-chord. Lift grows
  with the angle up to about 15 degrees, then the panel stalls. Ailerons, flaps, elevator and rudder
  change a panel's effective angle. Each engine pushes 80 kN on its own nodes, so thrust, lift and
  weight all reach the fuselage through the beams. The wheels roll along the fuselage axis, grip
  sideways, steer at the nose and brake on the mains. Two assists make it flyable on a keyboard:
  control throws shrink as speed rises, and with the keys released it levels the wings and trims
  out pitch rate.
- **Crashes**: an airframe beam that is permanently crushed or stretched past a limit snaps, so a
  hard impact breaks the airliner into pieces. Past a damage threshold it is wrecked: engines and
  controls die and the buckled wings stop lifting. Skin whose frame has pulled apart is no longer
  drawn, so breaks show as gaps. Wreckage scraping the ground throws dust and sparks. Breaking
  up on land, or violently on water, starts a fuel fire with smoke that burns down over about a minute.
  In the bay, drag rises with speed, so water stops a fast node almost dead: a dive tears the
  airframe apart in spray, while a gentle ditching leaves it whole and afloat. Flames go out
  under water.
- **The bay and far terrain**: outside the detailed 600 m square the airport map adds fine terrain
  along the causeway and one coarse sheet out to 6 km. The sea is a flat sheet shaded with rippling
  normals, a Fresnel reflection of the sky and sun glitter.
- **Maps**: `world_init(world, map)` builds the shared proving ground plus that map's own streets,
  buildings and landmarks. Road tables, the dirt washboard's location, Broadway and the city's
  extent all belong to the map, and the terrain asks the current map for them.
- **World**: the towns are generated from a fixed seed: lots are laid out along each block's
  street edges, then yards and the forest are filled in. Areas the headless test drives
  through are kept clear. Buildings collide as boxes; trees collide as trunk cylinders. A tree
  snaps once the impulse it takes in a short window passes a limit set by its trunk radius, then
  falls like a toppling rod in the direction it was hit. A uniform 6 m grid makes the collision
  checks cheap (about 70 buildings, about 1000 trees). Standing trees are baked into one mesh per
  50 m patch, which is rebuilt when a tree in it falls. The renderer bakes the buildings into one
  flat-shaded mesh, with windows, doors, shopfronts, gable roofs, chimneys and rooftop plant.
  The chase camera pulls in so buildings never block the view.
- **Vehicle paint** (`src/render.c`): vehicles and airliners are shaded per pixel. Each vertex
  carries a normal and a material: clear-coated paint, flame livery, chrome, glass, satin trim,
  rubber or brushed stainless. Diffuse light matches the world's baked lighting. On top come a
  Fresnel reflection of the procedural sky (with clouds), the ground and a dark skyline, and a sun
  highlight. Chrome and stainless are mirrors of that environment, sharp or blurred.
- **Sun shadows**: a 2048² shadow map, an orthographic view down the sun's rays covering 60 m
  either side of your car, refreshed every other frame. Buildings, trees, vehicles and airliners
  cast. The post pass looks each pixel up in it. Because the world's sunlight is baked into its
  colours, a shadowed pixel is scaled back to its ambient-only share, using a normal rebuilt from
  depth. Soft edges come from six rotated samples.
- **Ambient occlusion**: computed from the depth buffer at half resolution (eight samples in the
  hemisphere over each pixel), blurred, and multiplied in. It darkens creases, wheel arches and
  where things meet the ground. **F2** turns shadows and occlusion off together.
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
  and tyre grip halves (the Sparrow's 100–0 km/h stop goes from 36 m to 66 m).
- **Integration**: semi-implicit Euler at 1200 Hz fixed substeps. Each beam's stiffness and
  damping is limited by the masses it connects, which keeps the explicit integrator stable.

`tests/test_physics.c` is a headless test that runs acceleration, braking, cornering,
a tailgate drop, a 60 km/h wall crash, a ramp jump, a T-bone and hitting a sapling and a mature tree, then writes wireframe snapshots:
`cc -O2 tests/test_physics.c src/physics.c -lm -o tp && ./tp`

## Ideas for next steps

Per-pixel lighting from nearby lamps; screen-space reflections on chrome and paint; node-beam wheels and tyres; opening doors and frunk; self-collision so crushed panels
stack up instead of passing through each other; slip-angle tyre model; Metal renderer; sound.
