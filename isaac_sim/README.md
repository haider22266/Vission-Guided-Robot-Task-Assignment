For the current CDD classifier, five robot camera feeds, task visualization and
continuous warehouse cleanup, run from the workspace root:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_environment_cdd.ps1
```

Close an existing warehouse instance first. Omit `-Headless` for the 3D window
and the **Live robot cameras and CDD predictions** panel. See
[ENVIRONMENT_CDD_PROJECT.md](../ENVIRONMENT_CDD_PROJECT.md) for details.

For the earlier five-robot pick-and-place workflow with a fixed camera and an
open-front environment, see [FIVE_ROBOT_PICK_PLACE.md](FIVE_ROBOT_PICK_PLACE.md).

For the separate five-Ridgeback–Franka ROS 2 camera/allocator/controller system,
see [ROS2_PIPELINE.md](../ROS2_PIPELINE.md).

Run the earlier exploration scene from the workspace root in PowerShell:

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/disaster_scene.py --environment warehouse
```

The warehouse is the default environment: a 26 by 32 metre floor, three rack
rows, two main aisles, cargo, fallen boxes, floor markings and overhead lights.
The front and roof are open for the overview camera. Edit `warehouse.py` to
change the layout. The reference image guides the layout; its assets are not included.

The fleet uses official Isaac Sim USD robot assets at 25% size by default:

| Robot ID | Asset | Task role |
| --- | --- | --- |
| uav_1 | Bitcraze Crazyflie CF2X | area_search |
| uav_2 | NASA Ingenuity | area_search |
| ugv_1 | Clearpath Jackal | hazard_inspection and area_search |
| ugv_3 | IsaacSim Forklift C | supply_delivery and area_search |

See `asset_robots.py` for paths and role mapping. The Jackal is the standard
base asset, without the separately mounted manipulator shown in the reference
photo. Crazyflie's 5.1 catalog path is `Bitcraze/Crazyflie/cf2x.usd`.
Task capabilities are configured scenario assumptions. Asset geometry alone
is not evidence of fire/water resistance or an integrated inspection sensor.

Assets load from the configured Isaac server. Missing assets raise an error.
For a local Isaac asset pack, pass `--robot-assets-root <folder>` where the
folder contains `Isaac/Robots/`. Referenced asset files remain unmodified.
The scene overrides rigid bodies, joints and collision to disable robot physics
for a parked preview. Warehouse exploration follows patrol goals at 0.3?0.7 m/s using clearance-aware
A* detours around scene objects. Headings follow the direction of motion. Playback starts automatically; pause the timeline to pause motion.
Use `--parked` for a stationary scene. No wheel, flight, lifting or task-execution
controller runs. Motion remains kinematic; it uses USD object bounds rather
than onboard sensor perception or a drivetrain controller.
Use `--robot-scale 0.15` for smaller models, or `--robot-scale 1` for native size.
Source asset scale is preserved inside a scaled wrapper; the tiny Crazyflie may be difficult
to see in the wide warehouse overview. Select its prim and frame it in the viewport.

Build, export and capture without opening a window:

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/disaster_scene.py --headless --build-only --preview isaac_sim/scenes/warehouse_preview.png
```

Output: `scenes/disaster_scene.usda` and `scenes/disaster_scene.tasks.json`.
Use `--output <path.usda>` to choose another location and `--environment disaster`
for the outdoor marker layout. Warehouse structures/cargo have static collisions.
Disaster scores are synthetic. Human Damage is an area search clue.
Delivery remains masked until route safety is confirmed by a planner.

For NuRec environments, see [NUREC.md](NUREC.md).

Smaller exploring robots:

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/disaster_scene.py --environment warehouse --robot-scale 0.15
```

Warehouse patrol waypoints are in `exploration.py`. Ground routes pass around
the front debris and follow the rack aisles. UAVs use different heights and
cross between aisles at the front/back of the rack rows. Motion updates the
in-memory robot positions; exported task data describes the initial state.

Obstacle avoidance is implemented in `navigation.py`. Robot footprints come
from scaled asset bounds plus a 0.12m margin. A* uses a 0.3m grid and checks
full segments against inflated object bounds. The robot holds position if
no route exists, then retries. Scene bounds refresh each simulated second;
other robot positions are checked each update. Detection is limited to
authored collision objects and explicit disaster props, at each robot
altitude. Objects moved between refreshes can temporarily be absent from
the obstacle snapshot. This is not a physical or sensor-based safety guarantee.
The NuRec launcher uses smaller models but remains a stationary fleet display.
