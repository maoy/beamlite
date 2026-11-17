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
| M | back to the map menu (Esc there returns to your drive) |
| J | traffic on / off |
| L | lighting: dusk / day |
| F2 | sun shadows + ambient occlusion on / off |
| O | weather: dry / wet / rain / snow |
| Q | recover (fresh car, upright, in place) |
| R | reset to start |
| T / P | slow-mo / pause |
| K | clear skid marks |
| Y | collect the rubber duck in reach |
| Z | sound on / off |
| U | rebuild every knocked-down building (clears the rubble; anything parked on it is moved out) |
| Tab | Airport map: fly the airliner on the runway, then the helicopter on the pad, then back to the car |

Flying the airliner (first **Tab** on the Airport map):

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

Flying the helicopter (second **Tab**):

| Key | Action |
|---|---|
| Shift / Ctrl | collective up / down (it hovers at about 50%) |
| W / S (or ↑ / ↓) | cyclic: nose down to fly forward / nose up to slow down |
| A / D (or ← / →) | cyclic: bank left / right |
| Q / E | pedals: turn left / right |
| C | cockpit camera |
| R | put the helicopter back on the pad |

The rotor takes about six seconds to spool up. Raise the collective until it lifts off, then ease
the stick forward. Let go of the stick and it returns to level; level it out and it slows down. Hop
out in the air (**Tab**) and it slows to a hover and lets itself down. Keep the blades clear: a tip
that touches the ground, a wall, a tree or a vehicle shatters the rotor, and losing the tail rotor
sends it spinning.

The game opens on a two-second title card (any key skips it), then asks you to pick a map: **The
Town**, **New York City** or **The Airport**, each shown with an aerial view. Choose with ←/→ and
Enter, the number keys, or a click. **M** brings the menu back at any time. The pictures are frames
from the game itself and live in `assets/`; without them the screens fall back to a plain backdrop.

All three maps share the proving ground: a launch ramp straight ahead, a
crash-test wall behind you, concrete pillars, a warehouse district off a service road to the right,
and forest on the hills past ~100 m. Saplings and young trees snap when you hit them; mature
trunks stop you dead. Street lamps and traffic-signal posts are solid too: a slow car is stopped by one, a fast or heavy one
shears it off, and it goes dark and falls where it was hit. Every building on every map can be knocked down: hard hits punch holes in
the walls and throw out masonry, and once a building has taken enough it collapses into a rubble
pile you can drive over. A fast car takes out a house, a semi gets through warehouses and
apartment blocks, and the towers, hangars and terminal take several big hits (or the airliner).
Press **U** to stand them all back up.

Each map hides 24 small rubber ducks: tucked behind buildings, at the foot of trees, out in the
fields and forest, and a few on low flat roofs (knock the building down and the duck comes down
with it, or land the helicopter on the roof). At the airport two are afloat in the bay. Get within
a few metres of one in whatever you are driving or flying and a prompt comes up: press **Y** to
collect it. The count shows at the top of the screen, and ducks you have collected stay collected
when you switch maps.

- **The Town**: two small towns up the cross road to the left (+z), with avenues, side streets and
  pavements: houses, shops, apartment blocks, a couple of towers and warehouses, with saplings
  along the kerbs, and a dirt washboard section on the cross road.
- **New York City**: Midtown, with the Times Square "bowtie" where Broadway crosses 7th Avenue on a
  diagonal (you start facing up 7th Avenue toward it). Towers of 55–140 m fill the blocks wall to
  wall. A screen-clad wedge tower stands in the south gore and red grandstand steps on the
  pedestrian plaza. Animated LED billboards cover the facades around the square: original abstract
  content (tickers, video walls, colour-cycling spots), no real brands. They spill coloured light
  onto the street and smear across the wet road. There are zebra crossings, lane lines, street
  lamps and yellow cabs parked on 7th Avenue. The washboard moves south of the highway.
  Around that core the same grid of avenues and cross streets carries on to the edge of the map,
  about 290 buildings in all with towers up to 190 m, so every street ends in more city instead of
  forest. Tall buildings step back as they rise and carry cornices, mechanical floors, masts with
  warning lights or timber water tanks. Pavements are 3 m wide with kerbs, street trees, hydrants,
  litter bins, newspaper boxes, mailboxes, parking signs and subway entrances with globe lamps.
  Orange-and-white steam stacks and manholes breathe steam. The furniture is scenery: it doesn't
  collide. South of the highway is still the proving ground.
- **The Airport**: you start on the access road heading north toward a glass terminal, with a
  departures forecourt under a canopy, taxis at the kerb, and two car parks. Past the terminal,
  airliners sit nose-in at five gates with jet bridges, a control tower glows over the apron, and
  floodlight masts light it. Beside the tower is a helipad with a helicopter you can fly, a windsock
  and green edge lights. East of it, three maintenance hangars line their own apron, and an airliner
  stands outside the big one. Two more hangars sit west of the terminal (one with an airliner
  outside), and north of it all is runway 09/27. Like the runways at SFO, it runs 1.5 km out into a bay on a causeway, with piano keys,
  touchdown bars, numbers, edge and threshold lights and approach lights. A blue-lit taxiway runs
  alongside. Across the runway from the terminal is the north apron. Four more hangars and a row
  of service buildings face the runway from it: two cargo sheds, the fire station and an office
  block. Three taxiway crossings over the runway connect it to the south side. The bay lies north and east of the field. To the south and west the land rises into
  coastal hills. The map reaches 6 km from the field in every direction, so there is room to fly.
  The land stands clear of sea level everywhere, so the only water is the bay. Out past the field
  the hills are wooded with oaks and umbrella pines, and winding roads lead to hill villages with
  tiled roofs. That outlying scenery (woods, villages, roads) is for the view: it is not solid.
  An airliner waits lined up on runway 09: press **Tab** to fly it, and **Tab** again to fly the
  helicopter (see above). The parked aircraft are soft bodies and the buildings are solid. The field is graded flat, and this map has no washboard.

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
- **Helicopter**: a ~2 t light utility helicopter (after the H125 class) built the same way: 72
  nodes and 474 beams forming a cabin of five frames, an engine deck, a tapering tail boom with fin
  and stabiliser, a mast placed over the centre of mass, and skids on cross tubes that yield before
  the cabin does. It also flies by forces on nodes. The rotor's thrust acts on the rotor head along
  the mast and grows with the square of rotor speed and with collective. It falls off as the
  helicopter climbs through its own downwash, which is what turns a fixed collective into a steady
  climb or descent, and it gains a little in ground effect and with forward speed. The cyclic and
  pedals run through a stability-augmentation loop like a modern helicopter's: the stick commands a
  pitch and bank angle, the pedals a yaw rate, and in forward flight the tail weathervanes it into
  coordinated turns. The loop's output is an angular acceleration applied to every node as
  m (α × r), so it turns the airframe without pushing it anywhere. Its authority is the rotor's: none
  at rest, and full only once the rotor carries the weight. Blade tips are tested against the
  ground, buildings, trees, vehicles and other aircraft every substep, and a strike breaks the rotor
  in a burst of fragments and sparks. Low over the ground the downwash blows dust (or spray over
  the bay) out from under it.
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
  through are kept clear. Buildings collide as boxes; trees collide as trunk cylinders. A node
  hitting a building wall faster than 2.5 m/s adds its impulse to the building's damage; strength
  scales with volume^(2/3) by building type (`BLD_STRENGTH_K` in `physics.c`). Past it the box stops
  colliding and a rubble mound rises into the terrain height, so wheels and nodes ride on it. The
  renderer blanks the building's vertex range in the baked city mesh in place, sinks a copy of it
  into the dust, and scatters chunks that settle on the ground. A tree
  snaps once the impulse it takes in a short window passes a limit set by its trunk radius, then
  falls like a toppling rod in the direction it was hit. A uniform 6 m grid makes the collision
  checks cheap (about 70 buildings, about 1000 trees). Standing trees are baked into one mesh per
  50 m patch, which is rebuilt when a tree in it falls. The renderer bakes the buildings into one
  flat-shaded mesh, with windows, doors, shopfronts, gable roofs, chimneys and rooftop plant.
  The chase camera pulls in so buildings never block the view.
- **City facades** (`src/render.c`): New York's buildings are plain boxes, a few dozen triangles
  each, and a shader draws the facade on them per pixel. A wall carries one of six styles in its
  vertex alpha: blue or green curtain wall, brick, limestone, dark piers, concrete ribbons. Street
  level gets shopfronts with displays and coloured sign bands. Above that are bays of windows that
  reflect the sky by day and light up at dusk: offices a floor at a time in cool white, flats one
  by one in warm light, some with blinds half down. Far away, where a bay is smaller than a pixel,
  the wall fades to its average so it doesn't shimmer. Position along the wall comes from the
  object-space position and a normal rebuilt from derivatives, so the pattern stays put on a
  building while it collapses.
- **Vehicle paint** (`src/render.c`): vehicles and airliners are shaded per pixel. Each vertex
  carries a normal and a material: clear-coated paint, flame livery, chrome, glass, satin trim,
  rubber or brushed stainless. Diffuse light matches the world's baked lighting. On top come a
  Fresnel reflection of the procedural sky (with clouds), the ground and a dark skyline, and a sun
  highlight. Chrome and stainless are mirrors of that environment, sharp or blurred.
- **Ground**: terrain keeps its baked vertex lighting but a shader redraws the surface per pixel:
  lush and dry grass in patches, grain up close, bare rock where the slope steepens, dark canopy
  where woodland is marked, and grain and staining on tarmac. The bay has turquoise shallows and
  surf along the shore.
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
- **Sound** (`audio.c`): everything is synthesised as it plays; there are no sample files. One
  stereo stream mixes continuous voices, which the game steers every frame, with one-shot voices
  for events. Sources are placed by distance and panned by where they are relative to the camera.
  - Engines: the hatch is a four-cylinder (harmonics of its firing rate, brighter under load), the
    semi a diesel six with injector clatter. The pickup is electric, so it has no engine sound:
    silent at rest, then a thin motor whine that climbs with speed. The physics has no
    gearbox, so engine speed comes from road speed through made-up gears, and wheelspin lets it
    rev. Your car and the three loudest others each get a voice.
  - Helicopter: blade slap three times per rotor turn, a turbine whine and the tail rotor's buzz,
    all following rotor speed and collective. Airliner: exhaust roar, the whine of two fans
    slightly out of tune, and a spool-up that takes a couple of seconds.
  - Around you: wind that rises with speed, tyre roar, squeal when a tyre slides (quieter in the
    wet, on snow and off the road), rain, metal scraping along the ground, burning wreckage.
  - Events: a crash is a sudden change in a vehicle's velocity or beams snapping, and bigger hits
    sound lower and longer. Also glass, explosions, walls breached, buildings collapsing, trees
    snapping, splashes, rotor strikes, and the ducks.
  - Slow motion drops everything an octave. **Z** turns sound off.
- **Integration**: semi-implicit Euler at 1200 Hz fixed substeps. Each beam's stiffness and
  damping is limited by the masses it connects, which keeps the explicit integrator stable.

`tests/test_physics.c` is a headless test that runs acceleration, braking, cornering,
a tailgate drop, a 60 km/h wall crash, a ramp jump, a T-bone and hitting a sapling and a mature tree, then writes wireframe snapshots:
`cc -O2 tests/test_physics.c src/physics.c src/traffic.c -lm -o tp && ./tp`

`tests/test_audio.c` renders every sound through the synthesiser without opening an audio device,
checks each one's level, and can write them all to a WAV to listen to:
`cc -O2 -DAUDIO_HEADLESS tests/test_audio.c src/audio.c -lm -o ta && ./ta sounds.wav`

## Ideas for next steps

Per-pixel lighting from nearby lamps; screen-space reflections on chrome and paint; node-beam wheels and tyres; opening doors and frunk; self-collision so crushed panels
stack up instead of passing through each other; slip-angle tyre model; Metal renderer; Doppler shift and echoes off buildings.
