NuRec reconstructed environments
===============================

Fallen cartons and tool props
----------------------------

Galileo loads 12 added fallen cartons (roughly 0.62–0.70m wide), with tape
wrapped over the top and sides, shipping labels and barcode stripes. Several
cartons rest on their sides. Three orange fallen rack beams, eight debris
fragments, and separate hammer, screwdriver and open-ended wrench props sit
along the northern storage edge. These are editable polygonal USD objects in
`/World/DisasterProps`, defined in `disaster_props.py`. Collision meshes feed
into the NuRec navigation map; thin details below the map's floor-height
threshold remain visual floor clutter. Source reconstructed assets are unchanged.
The existing short task demo paths near y=0 remain clear; all three tasks
completed with these props in the verification run.

Run the demo with the complete fleet, task markers and added props in a fixed
full-room overhead view:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_nurec_tasks.ps1 -ViewProps
```

The launcher now uses this overview with or without `-ViewProps`; that switch
is retained for existing commands. The camera is initialized once above the
room; its near plane clips the captured roof and upper walls to expose the
interior without removing source objects. It does not track robots during
simulation. Racks can still
occlude nearby objects, and very small tools need a closer manual view for detail.
This is the four-robot symbolic task demo, separate from the five Ridgeback–Franka
ROS 2 launcher described in `../ROS2_PIPELINE.md`.
`--no-disaster-props` on the Python launcher omits these additions.
Use paired `--camera-eye X Y Z` and
`--camera-target X Y Z` arguments for another manual view.

Run and watch all three tasks complete
-------------------------------------

From the workspace root in PowerShell:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_nurec_tasks.ps1
```

This starts the actual Galileo neural-volume scene with four robot assets and
the reachable three-task demo in `isaac_sim/config/tasks_demo.json`. The task
window shows assigned robot, task status and remaining execution time. The
console prints each transition through assigned, in_progress and completed.
Demo execution uses a 4-second symbolic timer after reaching each location;
this is not physical pickup, real inspection or victim detection. The original
`tasks.json` is unchanged, including its unreachable delivery coordinate.

The GUI remains open after tasks finish. For an automatic verification run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_nurec_tasks.ps1 -Headless
```

Headless mode exits when all tasks complete or after 1800 updates. The verified
run completed all three tasks after 399 updates. Final results are saved in
`isaac_sim/scenes/nurec_task_demo.state.json`. `-IsaacRoot` selects another
Isaac installation and `-Steps` sets a finite update limit.

Combined Galileo disaster-response environment
----------------------------------------------

User-specified three-task list
------------------------------

The supplied list is saved in `isaac_sim/config/tasks.json`. Load and allocate it with:

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/nurec_scene.py --navigation --fleet-mode independent --camera-view group --tasks-file isaac_sim/config/tasks.json --explore-first 0
```

The generic capabilities map to the existing fleet: `camera` means area search,
`inspection` means ground inspection, and `delivery` means supply delivery.
Each task receives an estimated execution cost of 15 seconds unless supplied
explicitly, and all six visual scores default to zero (no image evidence).
`assigned_robot` is set on allocation and retained after symbolic completion.
Task coordinates are fixed and never snapped onto another location. Unreachable
tasks remain pending with a `blocked_reason`. In the current Galileo collision
mesh, `(0,-3)` lies outside the navigation bounds, so that delivery request
cannot be dispatched. Edit the JSON to choose a reachable point if desired.
The warehouse launcher also accepts `--tasks-file` for its catalog and markers,
but its patrol behavior does not execute the independent task allocator.
Convoy mode keeps the list visible but pauses individual task assignments.


Low aisle-height view matching the reference
-------------------------------------------

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/nurec_scene.py --navigation --camera-view chase
```

Chase view follows Jackal at 0.45m camera height and 1.2m behind its travel
heading, using a perspective lens. The ground robot stays ahead of the camera
with warehouse aisles in the background. Robot scale and shared-route behavior
remain unchanged. Camera position adjusts to free map cells near walls.
For a closer/lower shot, use `--camera-height .35 --camera-distance .8`.
To follow Forklift C instead, use `--follow-robot ugv_3`.
The image reference contains two shots; this mode supplies one moving chase
viewport, without a split-screen layout. For an all-fleet overhead view, use
`--camera-view group`.

Whole-fleet view and shared route
------------------------------------------

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/nurec_scene.py --navigation
```

The navigation default is now `--fleet-mode convoy --camera-view chase`.
All four official robot assets use 15% display scale. A common route is planned
against the union of their altitude maps using the largest robot footprint.
They move along the same XY centreline at their own UAV/ground heights, with
spacing. At route ends, visited-cell counts prefer unvisited common-map areas.
The occupied path prefix is retained when extending the route, preserving all
robot positions. The formation can reverse its order to explore from its other
end without vehicles passing each other. Candidate extensions are checked for
clearance around bends. If none fits, the fleet backtracks and retries. Coverage
is limited to connected areas that fit the largest robot at every fleet altitude;
unreachable rooms and obstructed regions cannot be visited. Visited cells and
route-leg counts are recorded in navigation metadata. Small colored annotation rings identify
tiny models without changing the referenced asset geometry. The wide overhead
view remains available with `--camera-view group`. The optional elevated rear camera (`--camera-view rear`)
tracks the group from behind and slightly above. It prefers clear viewpoints
behind the current travel heading, eases short camera motion in free space,
and adjusts framing for the formation's extent. Viewpoints can change near
walls; scene geometry may still occlude a model. Use `--exploration-speed .2`
to slow the convoy or `.4` to speed it up.

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/nurec_scene.py --navigation --camera-view rear
```

The three task types and risk data remain in the scene, but separate task
assignments pause in convoy mode so robots stay together. This is kinematic
movement; it does not implement a physical vehicle or flight controller.
Use `--robot-scale .1` for smaller models. If a map cannot fit the fleet with
clearance, the launcher reports an error instead of shrinking clearance.

For the previous independent task behavior and camera tour:

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/nurec_scene.py --navigation --fleet-mode independent --camera-view tour
```

The following section describes that independent mode.

From the workspace root, select independent mode explicitly:

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/nurec_scene.py --navigation --fleet-mode independent --camera-view tour
```

Independent mode selects the verified local Galileo `3dgrt/real2sim_galileo.usd` asset,
adds all four official robot assets at 25% scale, creates the three task types,
and starts exploration automatically. GUI mode runs until closed. Use
`--robot-scale .15` for smaller models or `--steps 500` for a finite run.
`--navigation` enables the disaster fleet and omits the extra Carter sample.

Robots explore for the first 15 seconds before task allocation. Unvisited
cells are preferred for new exploration goals, within reachable mesh space.
Use `--explore-first 30` for a longer initial survey or `--explore-first 0`
to allocate tasks immediately.

With `--camera-view tour`, the automatic camera cycles through four
robot-follow shots and up to four viewpoints spread across clear room areas,
with eight seconds per shot. Camera eyes use conservative free-map positions;
shots cut between locations instead of crossing walls. Within each shot,
camera motion eases toward the current robot/viewpoint. Reconstructed geometry
may still occlude robots, and the tour does not guarantee every room location
is visible. Use `--camera-dwell 5` for a faster tour, or
`--camera-view manual` to control the viewport yourself. Pause the simulation
timeline to pause the tour and robots together.

Collision mesh triangles are transformed into world coordinates and rasterized
into conservative 0.2m maps at each robot's height. Floor triangles below 10cm
are excluded; map bounds come from the reconstructed mesh. Robots are placed
in free space with clearance, and synthetic task areas move to reachable
points. A* uses the scaled robot footprint plus a 12cm margin. Peer robot
positions block paths too; robots stop and retry when blocked. Map geometry
is static during the run. Scene edits require restarting the combined mode.
Unknown geometry, overhangs, holes, or reconstruction errors can invalidate
the approximation. This uses scene knowledge, not live robot sensors.

A capability/priority/distance baseline allocates unique tasks, verifies a
mesh-map route and battery reserve, and drives robots to the task area.
After arrival, task execution is a symbolic timer using estimated task cost,
then status becomes completed and estimated energy is deducted. Search and
inspection produce no real measurements; delivery involves no actual payload
pickup/drop-off. Idle robots continue exploring reachable points. All image
scores are synthetic; Human Damage is an area search clue. No QMIX policy,
trained classifier, Nav2, wheel/flight controller, or lift controller is loaded.

To export the combined stage, initial catalog, final state and preview:

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/nurec_scene.py --navigation --headless --steps 180 --output isaac_sim/scenes/galileo_response.usda --preview isaac_sim/scenes/galileo_response.png
```

The USD wrapper and `.tasks.json` describe the initial environment. The
`.state.json` sidecar records final robot positions, tasks, battery, and masks.
Live task data is stored on `/DisasterResponse` in the open stage. Route-safety
fields refer only to the conservative mesh map, not certified physical safety.
The original static-view commands below remain available without `--navigation`.

Verified on the installed Isaac Sim 5.1
-------------------------------------

Add the disaster-response fleet and task markers:

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/nurec_scene.py --stage assets/nurec/nova_carter-galileo/3dgrt/real2sim_galileo.usd --scene-only --disaster-robots --steps 0 --output isaac_sim/scenes/nurec_disaster.usda
```

This adds official Crazyflie and Ingenuity aerial assets for area search,
Clearpath Jackal for hazard inspection, and Forklift C for supply delivery.
Ground robots can also search. Colored area markers carry synthetic tasks.
Human Damage raises area-search priority and does not localize a person.
The standard Jackal USD does not include the arm shown in the user's photo.
Crazyflie's 5.1 path is `Bitcraze/Crazyflie/cf2x.usd`. Asset paths and task-role
mapping are in `asset_robots.py`; the catalog is on `/DisasterResponse` and in
an exported `.tasks.json` sidecar. Roles are scenario capability assumptions.
Supply delivery remains masked until a planner confirms route safety.

Use `--robot-assets-root <folder-or-url>` to override the Isaac asset root
(the folder must contain `Isaac/Robots`). Models use 25% display scale by default (`--robot-scale .25`) with
robot physics disabled by scene overrides; flight, driving, lifting and QMIX
controllers are not implemented. Native-scale Forklift C needs more clearance
than this small captured room may provide; the procedural warehouse is a better
layout for viewing the full fleet. Fleet placement is relative to
`--fleet-origin X Y Z` (Galileo default -2, 0, 0), without collision-clearance
validation. No source robot USD is modified.
`--scene-only` omits the separate Carter navigation sample while keeping this
fleet when `--disaster-robots` is supplied.

The downloaded `v0.1` Galileo `3dgrt` scene renders successfully locally:

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/nurec_scene.py --stage assets/nurec/nova_carter-galileo/3dgrt/real2sim_galileo.usd --scene-only --steps 0
```

To download only the files for this working variant:

```powershell
hf download nvidia/PhysicalAI-Robotics-NuRec --type dataset --revision v0.1 --include "nova_carter-galileo/3dgrt/*" --local-dir assets/nurec
```

The particle variant was also downloaded, but its render test failed with a
missing `ParticleFieldEmissive` shader and a black preview. The volume variant
failed to create its neural renderer. The older `3dgrt` variant above is the
verified option for this local installation. Commands below describe the
newer example's particle workflow and require a compatible runtime.

`nurec_scene.py` implements the scene-loading and Nova Carter navigation flow
from NVIDIA's [6.0 example](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/assets/usd_assets_nurec.html).
It is a separate entry point from the procedural warehouse/disaster demo.
It loads a real reconstructed asset, optionally adds the Nova Carter navigation
sample, sets its spawn and relative target, and runs the timeline. The `--disaster-robots` option adds a synthetic task catalog; no QMIX
policy or disaster classifier runs in this entry point.

Dataset access
--------------

Accept the access conditions on the
[NVIDIA dataset page](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-NuRec)
in your browser. Then authenticate in your terminal (never paste tokens into chat):

```powershell
hf auth login
```

The checked local installation at `C:\isaac-sim` is 5.1. NVIDIA identifies
dataset revision `v0.1` as targeting 5.1; current `main` targets 6.1 and changes
asset filenames. Match the runtime, dataset revision, and asset variant. See
the dataset card's release table and the
[5.1 NuRec documentation](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_nurec.html).
Rendering of the actual gated dataset has not been verified locally.

Download the Galileo particle-stage package and small support files. Skip
`raw_images.zip` (8.4 GB) and the volume variant when running particle rendering. The short
`assets/nurec` directory keeps Hugging Face temporary paths below the legacy
Windows path-length limit in this workspace:

```powershell
hf download nvidia/PhysicalAI-Robotics-NuRec --type dataset --revision v0.1 --include "nova_carter-galileo/stage_particle.usdz" --include "nova_carter-galileo/occupancy_map.*" --include "nova_carter-galileo/training_trajectory*" --local-dir assets/nurec --dry-run
hf download nvidia/PhysicalAI-Robotics-NuRec --type dataset --revision v0.1 --include "nova_carter-galileo/stage_particle.usdz" --include "nova_carter-galileo/occupancy_map.*" --include "nova_carter-galileo/training_trajectory*" --local-dir assets/nurec
Get-ChildItem assets/nurec/nova_carter-galileo
```

Start with the particle-stage variant used in the linked 6.0 example:

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/nurec_scene.py --scene galileo --asset-file stage_particle.usdz --scene-only --steps 0
```

If your chosen revision uses another filename, pass its exact name with
`--asset-file`, or pass a local USD/USDZ using `--stage`. Legacy neural-volume
variants may need the rendering settings in that revision's instructions.
The latest `particle_sh_optimized.usdz`, `particle_spg-runtime.usdz`, and
`volume.usdz` variants are not drop-in replacements for the old 6.0 example;
the SPG variants require the additional setup described in the dataset card.

Add Carter and run the documented 500-update navigation example:

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/nurec_scene.py --scene galileo --asset-file stage_particle.usdz --steps 500 --output isaac_sim/scenes/nurec_galileo.usda
```

Supported presets: `galileo`, `cafe`, `andoria`, `wormhole`. Override spawn
with `--start X Y Z` and the target relative to the Carter navigation root
with `--target X Y Z`. `--carter-usd <path-or-url>` selects a local navigation
sample instead of downloading from the Isaac asset server. `--preview <png>`
captures the initial viewport; `--headless --steps 500` runs without a window.

The loader creates an editable USD wrapper with the scene as a sublayer,
avoiding the documented problem with adding assets to a USDZ root stage.
Exported wrappers reference the local dataset by absolute path; retain that
dataset at the same location. Source assets are not modified.

Gaussian appearance does not provide physical collision. The loader checks
for authored collision APIs before adding Carter. This is an existence check,
not validation of collision coverage or a safe route. `--collision-ground`
explicitly adds only a 20m square invisible floor at spawn height; it does not
model walls or obstacles. The supplied presets assume Z-up, metre-based scenes.
For disaster tasks, choose locations in the reconstructed scene's coordinate
frame rather than reusing warehouse coordinates or synthetic visual scores.
