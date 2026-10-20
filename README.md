# HairSim

**HairSim is a MATLAB-based simulation platform for robotic haircutting
research.** It integrates head and hair modeling, directional clipper cutting,
toolpath planning, and robot execution in an interactive environment. The
platform supports automated haircutting, keyboard teleoperation, and session
replay, and provides interfaces for researchers to develop custom planners,
integrate robot models, and explore haircutting strategies.

A robot arm (UR5e by default, Kinova Gen3 included) holds an electric clipper
next to a scanned head. Hair is modelled as guide strands rooted on the scalp.
The arm can be mounted on a fixed base, on the ceiling, or on a circular floor
rail around the head.

## Demo

<!-- TODO: add media under docs/media/ and link it here -->

| Automated haircut | Keyboard teleoperation | Before / after |
|---|---|---|
| *(GIF: `runFullDemo`, all five regions)* | *(GIF: `runTeleop`)* | *(screenshots)* |

## Features

- **Scanned head and scalp regions.** A head mesh with hair roots sampled on
  the scalp, each root assigned to one of five regions (`scalp_top_left`,
  `scalp_top_right`, `scalp_left`, `scalp_right`, `scalp_back`).
- **Guide-hair model with live display.** Strands follow a hair-flow field,
  respond to gravity and contact, and fall as clippings when cut.
- **Directional clipper cutting.** A strand is cut to the target (guard-comb)
  length only when its root passes under the clipper comb, the blade is close
  enough to the scalp, and the feed runs against the hair flow.
- **Automatic toolpath planning.** CAM-style clipper passes per region,
  pushed against the local hair flow, with rest machining and edge cuts where
  the blade does not fit.
- **Robot constraint checking.** Every sample of every stroke is checked for
  IK feasibility, robot-vs-head collision, self-collision and clipper
  clearance. Strokes are linked into one chain with safe transits and rail
  slides.
- **Recording and replay.** Runs are recorded (joint samples, clipper path,
  cut events) and a full-head session can be saved as a bundle and replayed
  without re-planning.

## Installation and quick start

### Requirements

- MATLAB R2025b (developed and tested there)
- Robotics System Toolbox

### Launch

From the app folder:

```matlab
runAuto       % plan a region, preview, run, record
runTeleop     % drive the clipper with the keyboard
```

Each launcher first calls `setupPath`, which puts this copy of the `+hairsim`
package, and no other copy, on the MATLAB path. Don't run the launchers from
inside another copy's folder: the current folder takes precedence over the
path.

### Your first haircut

1. Run `runAuto`.
2. Keep the default robot setup, region and target length.
3. Press **Plan**. Planning a region takes several minutes; the plan is then
   cached in `config/plans` and reused next time.
4. Press **Preview** to see the strokes, then **Run** to execute them on the
   robot and watch the hair being cut.

## Usage

### Auto

Interactive planning and execution of one scalp region.

1. Choose a **Robot Setup**: fixed, ceiling, or a rail station.
2. Choose a **region** and a **target length** (the guard-comb length left
   on the head).
3. **Plan**, then **Preview** and **Run**.

When the rail is used, the station is chosen per stroke, so the base moves
during a run when it has to.

`runAuto` also takes name–value options, e.g.
`runAuto('AdapterName', "gen3", 'RegionName', "scalp_back", 'TargetLengthM', 0.01)`.

### Teleop

Click the 3D view first. The clipper rides on the scalp.

| Key | Action |
|---|---|
| W / ↑ | push forward along the feed (the only move that cuts) |
| S / ↓ | pull back |
| A D / ← → | slide sideways |
| Q / E | turn the feed left / right (15°) |
| G | turn the feed against the hair flow under the clipper |
| R / F | lift / lower 10 mm (lowest = touching the scalp) |
| Z / X | roll the blade for edge cuts (up to ±30°) |
| hold Space | cut |

Poses are built and checked with the same code the planner uses. A move that
would hit the head or self-collide is refused, and the readout shows why. The
readout also shows the height above the scalp, whether the blade is touching,
and the feed angle against the flow. Recording is optional: cutting works
without it.

### Full-head task

Plans and runs all five regions in one continuous session; hair cut in one
region stays cut when the next region is planned.

```matlab
result = runFullDemo('BasePlacementId', "rail180", 'TargetLengthM', 0.005, ...
    'SaveBundlePath', "outputs/fullhead_bundle.mat");
```

`BasePlacementId` is `"fixed"` (default), `"ceiling"`, `"rail<deg>"` (the
rail, starting at that station, e.g. `"rail180"`), or `"railPerRegion"` (the
rail, starting at a preset station for the first region). On the rail the
planner picks the station of every stroke itself and the rail slides between
stations when needed, so the two rail options only set where the session
starts. See `help hairsim.app.runFullDemoSession` for all options.

### Replay

```matlab
result = replayBundle("outputs/fullhead_bundle.mat");   % no planning
```

### Main parameters

| Parameter | Where | Meaning |
|---|---|---|
| Robot setup / `BasePlacementId` | Auto, full demo | fixed, ceiling, or rail station |
| `AdapterName` | all entries | `"ur5e"` (default) or `"gen3"` |
| Region / `RegionName` | Auto | one of the five scalp regions |
| Target length / `TargetLengthM` | all entries | guard-comb length in metres |
| `PlannerName` | all entries | any registered planner (built in: `"camClipperAgainstFlow"`) |
| `DataDir` | all entries | folder with your own head and hair data |

## Extending HairSim for research

The `hairsim.api` package exposes the whole pipeline without the GUI:

```matlab
setupPath
scene   = hairsim.api.loadScene();                      % head, hair, regions
context = hairsim.api.createRobotContext("ur5e", scene); % robot, IK, collision
plan    = hairsim.api.planRegion(scene.haircutState, "scalp_back", 0.005, ...
              context, "camClipperAgainstFlow");
hairsim.api.executePlan(plan, context);                 % headless replay
```

### Plug in your own planner

Write a function with the signature

```matlab
regionPlan = myPlanner(haircutState, regionName, targetLengthM, context, options)
```

that returns the same `regionPlan` fields as
`hairsim.planning.camClipperPlanner` (strokes, transits and their joint
trajectories). Register it once per session:

```matlab
hairsim.planning.registerRegionPlanner("myPlanner", @myPlanner);
```

It then appears in the Auto planner list and can be passed as `PlannerName`
to every entry or to `hairsim.api.planRegion`. You can reuse the building
blocks of the built-in planner: toolpath generation in `+hairsim/+cam`,
robot post-processing (IK, collision, transits) in `+hairsim/+post`, and the
coverage evaluation in `+hairsim/+eval`.

### Add a robot

Write a zero-argument builder that returns an adapter spec (`name`,
`loadRobotName` for `loadrobot`, `endEffector`, IK `weights`, optional
pre-tuned Home poses `qHomeConstant` / `qHomeCeiling`) and register it:

```matlab
hairsim.robot.registerRobotAdapter("myRobot", @myRobotAdapter);
runAuto('AdapterName', "myRobot")
```

`+hairsim/+robot/+adapters/ur5e.m` is the reference implementation; see
`help hairsim.robot.registerRobotAdapter` for the full contract. If no
`qHomeConstant` is given, a safe Home is solved by IK on first use; without a
`qHomeCeiling`, the ceiling mount starts from the robot's own home
configuration.

### Use your own head and hair roots

Put a folder with the same three files as `data/` and pass it as `DataDir`:

| File | Contents |
|---|---|
| `HeadScan.stl` | head mesh |
| `hair_roots.json` | one entry per root: `id`, `region`, `triangle_id`, `barycentric`, `position`, `normal`, `density_weight` |
| `HairClipper.stl` | clipper mesh used for display and collision |

```matlab
runAuto('DataDir', "/path/to/my_head")
scene = hairsim.api.loadScene('DataDir', "/path/to/my_head");
```

### Change the cut model

The cut model is a struct created by
`hairsim.hair.cutting.createCombLiftCutModel` and dispatched on its `type`
in `hairsim.hair.cutting.applyClipperHaircutStep`. To try a different cutting
rule, add a new `type` with its own apply function there. The clipper
geometry is built by `hairsim.tool.createClipperTool`; the built-in planner
sets its 45° feed-angle limit (`MaxFeedAngleRad`) in
`hairsim.planning.camClipperTool`. Keep the planner's coverage predicate
(`hairsim.eval.sweepCombLiftCoverage`) consistent with the cut model if you
want planned coverage and executed cuts to agree.

## Project structure

| Folder | Contents |
|---|---|
| `+hairsim/+api` | library entry points: load scene, build robot context, plan, execute |
| `+hairsim/+hair` | hair model, dynamics, rendering, clippings, cut models |
| `+hairsim/+cam` | toolpaths: machining surface, hair flow, passes, segments, rest machining |
| `+hairsim/+planning` | planner registry and the built-in camClipper planner |
| `+hairsim/+post` | robot post-processor: stations, IK, transits, rail slides, park pose |
| `+hairsim/+tool` | clipper model, signed distance to the head mesh, clearance |
| `+hairsim/+robot` | robot adapters (UR5e, Gen3), adapter registry, rail |
| `+hairsim/+execution` | plan execution engine shared by the GUI, the full demo and tests |
| `+hairsim/+eval` | coverage rule, placement map, ideal-vs-robot comparison |
| `+hairsim/+app` | interactive Auto and Teleop, full demo, replay, recording, view |
| `data/` | head scan, hair roots, clipper mesh |
| `tests/` | release tests |

### Tests

```matlab
runReleaseTests                  % about 15 min
runReleaseTests('Quick', true)   % about 3 min, skips the full region plan
```

| Test | Checks |
|---|---|
| `testCamClipperFoundations` | clipper model, exact head distance, coverage rule and cut on analytic and real geometry |
| `testTeleopClipper` | headless Teleop: clearance ≥ 1 mm; strands cut are exactly those the coverage rule predicts; pulling back or pushing with the flow cuts nothing |
| `testLaunchers` | `runAuto` and `runTeleop` open and close |
| `testAutoRegion` | plans `scalp_back`, executes it headless, and checks the strands cut match the evaluation |

## Model assumptions and current limitations

- **Feed-direction threshold.** A strand is cut only when the feed is within
  45° of against the hair flow. This is a modelling assumption, not a
  measured value.
- **Guard comb as logic.** The comb is not simulated geometrically: when the
  rule above holds, the strand is stood up and cut at the target length.
  Strands already at or below the target are never touched.
- **Guide-hair model.** Strands are guide hairs, not a full hair volume. The
  hair physics (gravity, bending, length constraints, contact with the exact
  head mesh) only shapes the displayed hair; coverage and cutting are decided
  from the roots and the strand nodes, so they do not depend on it.
- **Collision scope.** Robot links are checked against a convex hull of the
  head, the clipper against the exact head mesh, plus self-collision. On the
  rail the links are also checked against a simplified floor, torso and
  shoulder model (a cylinder and a box, not anatomy); the fixed and ceiling
  mounts have no body model.
- **Uniform target length** per run (one guard-comb length for the area
  being cut).

## Citation, acknowledgements and license

**Citation.** A paper describing HairSim is in preparation. Until then,
please cite this repository:

```bibtex
@misc{hairsim,
  title  = {HairSim: A Simulation Platform for Robotic Haircutting},
  note   = {https://github.com/<org>/HairSim},
  year   = {2026}
}
```

**Acknowledgements.** *(to be added)*

**License.** *(to be decided.)* The code license and the terms for the head
scan and hair-root data in `data/` will be stated here.
