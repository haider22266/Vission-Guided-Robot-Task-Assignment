Run the five-robot manipulation demo from the workspace root:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_five_robot_pick_place.ps1
```

The new entry point is `five_robot_pick_place.py`. It builds a separate,
editable warehouse with no front or side walls and no roof. The rear wall
remains. The previous NuRec and task-demo scripts are unchanged.

Five official Isaac Sim Franka USD arms sit on geometric mobile display bases:

| Robot | Environment objects |
| --- | --- |
| robot_1 | Rear cartons with tape bands |
| robot_2 | Rubble pieces |
| robot_3 | Screwdrivers |
| robot_4 | Rear rack beams |
| robot_5 | Wrenches |

Objects are grasp-sized models, not the full-size cartons/beams from the other
scene. Two objects per type are initially on feed tables. Each robot approaches
its feed area, uses Lula IK/FK to align its real arm and gripper, closes both
fingers, lifts, carries, lowers to a reserved rack slot, opens, and retreats.
The live panel shows current object, phase, rack destination and completed count.

`pick_place_tasks.py` reserves objects and slots atomically in the single-threaded
simulation. An object cannot be assigned to two robots. Once initial objects
are racked, robots continuously pick racked items and relocate them into other
free slots. Objects are not teleported back to feeders or silently deleted.

The camera is configured exactly once before the simulation loop. There are no
follow, tour, tracking, or per-frame camera pose updates. An AST test checks the
script, and each run reports `camera_unchanged` in its results.

Grasp verification and limits
------------------------------

This is a kinematic demonstration: source robot rigid bodies and joints are
disabled and the visible official meshes follow Lula joint kinematics. It does
not simulate contact forces, friction, grasp stability, mobile-base dynamics,
or robot/environment collision avoidance. The mobile bases use independent lanes.

No object pose is updated before a verified grasp. Verification measures the
actual world-space inner faces of the two visible rubber pads added to the
Franka fingers. Both must be within 2mm of the object's specified grasp surfaces,
the TCP within 1.5mm of the object centre, and orientation error below 0.005 for
five successive frames. Failure leaves the object stationary and unacquired.
On success the exact relative pose is preserved while carrying, avoiding a snap
or a visible gripper/object gap. The scene contains simple known grasp regions;
this is not generic perception-based grasp planning for arbitrary geometry.

Release occurs only at the rack destination, within 4mm position tolerance,
after both fingers clear the item by at least 3mm. The item stays on the shelf
at its measured placement pose. Completion records retain the measured gaps,
placement error, robot, object and destination slot.

Validation run
---------------

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_five_robot_pick_place.ps1 -Headless -Steps 3600 -StopAfter 15
```

Direct Python invocation with an optional fixed-view screenshot:

```powershell
& "C:\isaac-sim\python.bat" isaac_sim/five_robot_pick_place.py --headless --steps 3600 --stop-after 15 --preview isaac_sim/scenes/five_robot_pick_place.png
```

Outputs default to `isaac_sim/scenes/five_robot_pick_place.usda` and
`five_robot_pick_place.results.json`. `--output` chooses a different path;
`--robot-assets-root` accepts a local Isaac asset-pack root containing
`Isaac/Robots`. The PowerShell launcher supports `-IsaacRoot` for another
Isaac installation. Closing the window stops the otherwise continuous run.
