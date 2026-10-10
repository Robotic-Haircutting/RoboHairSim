# HairSimApp

A MATLAB simulation of a robot cutting hair with an electric clipper. A UR5e
arm (a Kinova Gen3 adapter is included) on a circular floor rail holds a hair
clipper. Strands are modelled as guide hairs on a scanned head. The robot can
cut by itself (**Auto**) or be driven by keyboard (**Teleop**).

The planner works the way CNC machining software does:

1. **CAM toolpaths** (`+hairsim/+cam`). For each scalp region, the clipper
   passes are generated from scratch. They are parallel passes, each pushed
   *against* the local hair flow, cut into 4 cm segments. Where the blade
   does not fit, rest machining and edge cuts fill in (an offset or rolled
   blade).
2. **Robot post-processing** (`+hairsim/+post`). Every segment becomes a joint
   trajectory. This step searches rail stations, splits segments that are
   only partly reachable, and checks IK, collisions and clipper clearance on
   every sample. It then links the strokes into one chain with safe transits
   and rail slides.
3. **Evaluation** (`+hairsim/+eval`). One coverage rule decides whether a
   strand is cut. A strand counts as covered when its root lies under the
   clipper's comb, the blade is close enough to the scalp, and the feed runs
   within 45° of against the hair flow. Coverage is reported twice: **A**
   uses the ideal CAM poses and **B** uses the poses the robot actually
   reaches. The loss between A and B is broken down by cause.
4. **Cutting.** When a plan runs, or in Teleop, the hair is cut by the same
   rule (the comb-lift cut model, `+hairsim/+hair/+cutting`). A strand is
   cut to the target length exactly when the evaluation says it is covered.

## Requirements

- MATLAB R2025b (developed and tested there)
- Robotics System Toolbox

## Quick start

From the app folder:

```matlab
runAuto            % plan one region, preview, run, record
runTeleop          % drive the clipper with the keyboard
result = runFullDemo('BasePlacementId', "rail180", 'TargetLengthM', 0.005, ...
    'SaveBundlePath', "outputs/fullhead_bundle.mat");   % all five regions, one session
result = replayBundle("outputs/fullhead_bundle.mat");   % replay a saved demo, no planning
```

Each launcher first calls `setupPath`. `setupPath` puts this copy of the
`+hairsim` package, and no other copy, on the MATLAB path. The current folder
takes precedence over the path, so don't run the launchers from inside
another copy's folder.

### Auto

1. Choose a **Robot Setup**: fixed, ceiling, or a rail station.
2. Choose a region and a target length, which is the guard-comb length.
   Planning a region takes several minutes. Plans are cached in
   `config/plans`.
3. Press **Plan**. When it finishes, **Preview** draws the strokes and
   **Run** executes them on the robot.

During planning the rail station is chosen for each stroke, so the base
moves during a run when it has to.

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
would hit the head or self-collide is refused, and the readout shows why.
The readout also shows the height above the scalp, whether the blade is
touching, and the feed angle against the flow. Recording is optional: cutting
works without it.

## Tests

```matlab
runReleaseTests              % about 15 min
runReleaseTests('Quick', true)   % about 3 min, skips the full region plan
```

| Test | Checks |
|---|---|
| `testCamClipperFoundations` | the clipper model, exact head distance, the coverage rule and the cut on analytic and real geometry |
| `testTeleopClipper` | headless Teleop on the scalp: clearance ≥ 1 mm; the strands cut are exactly those the coverage rule predicts; pulling back or pushing with the flow cuts nothing; clippings fall while the arm stands still |
| `testLaunchers` | `runAuto` and `runTeleop` open and close |
| `testAutoRegion` | plans `scalp_back`, executes the plan headless, and checks that the strands cut are exactly the ones evaluation B predicted |

## Layout

| Folder | Contents |
|---|---|
| `+hairsim/+cam` | toolpaths: machining surface, hair flow, raster, segments, rest machining |
| `+hairsim/+post` | robot post-processor: stations, IK, transits, rail slides, park pose |
| `+hairsim/+tool` | clipper model, exact signed distance to the head mesh, contact lift |
| `+hairsim/+eval` | coverage rule, placement map, ideal-vs-robot comparison |
| `+hairsim/+planning` | planner registry (`registerRegionPlanner`, `listRegionPlanners`), the camClipper planner |
| `+hairsim/+robot` | robot adapters (UR5e, Gen3) |
| `+hairsim/+execution` | plan execution engine, used by the GUI, the full demo and tests |
| `+hairsim/+hair` | hair model, physics, clippings, cut models |
| `+hairsim/+app` | the two interactive entries, the full demo, bundle replay, Teleop, view |
| `data/` | head scan, hair roots, clipper mesh |
| `tests/` | the release tests above |

## Extending

- **Another planner.** Write a function with the signature
  `regionPlan = f(haircutState, regionName, targetLengthM, context, options)`
  that returns the same `regionPlan` fields as
  `hairsim.planning.camClipperPlanner`. Register it with
  `hairsim.planning.registerRegionPlanner("name", @f)`. It then appears in
  the Auto planner list.
- **Another robot.** Add an adapter next to `+hairsim/+robot/+adapters`.
- **Your own head.** Pass `'DataDir'` to `hairsim.api.loadScene` (and to the
  entries) pointing at a folder with the same three files as `data/`.

## Known limitations

- The 45° feed-angle rule is a modelling assumption, not a measured value.
- Hair physics can lift strands at the back and sides off the scalp. Coverage
  uses the roots, so it is not affected, but the rendered shape is.
- Front and side rail stations are not checked against the shoulders or
  torso. There is no body model.
