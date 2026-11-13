Run the existing warehouse with continuous cleanup:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_environment_cdd.ps1
```

The default is now `-TasksPerRobot 0`: remove the old two-object limit and use
a shared queue of all registered movable disaster props. The inspected scene
contains 88 logical objects: compound cartons/tape and multi-part tools count
as one object each. Warehouse stock and rack structure are not cleanup targets.
The same stage, five robots, trained camera classifier and fixed viewport remain.

Each robot first tries its original object family, then other available floor
objects. Reservations cover both object identity and the object's projected
world-space rack footprint. A robot must find a feasible approach/grasp, a free
position on an original low rack pallet, and a reachable placement/retreat pose
before claiming the job. The scene contains 22 candidate low pallet supports;
occupied stock and already reserved/stored footprints block candidate positions.
Physics raycasts measure the actual pallet support surface. Objects are not
duplicated, removed from the simulation or reported as stored to make progress.

Oversized beams are screened before pickup: unsupported depth is limited and
the beam footprint must fit the pallet's usable length. In particular,
`Fallen_Rack_Beam_02` caused a placement IK/tracking failure in the earlier run.
It now stays pending with a "needs longer storage support" reason instead of
being carried into an unusable rack position. Robots can select other work;
this does not claim that the oversized beam has been cleaned. It needs a
suitable long-load storage destination to complete that job.

Before placing an eligible load, the controller recomputes the target from the
measured carried-object bounds and gripper pose, checking sampled approach,
lowering and retreat IK. Small alternative placement offsets and docking moves
are considered with object/rack reservations and navigation clearance retained.
Objects move through the physical robot and verified attachment, never direct
object-pose edits to simulate a successful placement. The placement regression
stored a normal carton with 3.8 mm error; the oversized-beam regression correctly
made no pickup. See `reports/stuck_robot_fix_validation.json` for scope and limits.

The existing controller approaches, aligns, verifies contact, performs its
assisted grasp/lift, carries, places and releases the object. It checks support
and settling, then backs out and requests another job. Completed objects stay
on the racks. Queue progress causes previously blocked pending jobs to be
reconsidered. Robots now work **concurrently by default**. Independent routes
can run at the same time, and one robot can manipulate an object while others
travel. A spatial reservation encloses each navigation path, robot and carried
load. A conflicting path yields until space becomes available; reaching a goal
releases that navigation reservation, without waiting for the whole task to end.
Stationary peers, shelves, floor clutter and stored objects remain planning
obstacles. The live per-frame peer/load overlap check and local avoidance remain
active. The viewport camera stays fixed.

This is a conservative spatial route coordinator, not Nav2 or a proven
collision-free controller. Conflicting routes may wait even if they could be
used at different times. Real Isaac physics tests are still necessary for
complete cleanup, narrow passages and changing carried-load geometry.
`-Serial` explicitly enables the former one-trip-at-a-time debugging mode;
omit it for normal concurrent operation.
The camera classifier only ranks worker scheduling; USD object registration
supplies target locations. Whole-image classification does not locate objects.

The GUI has no default simulated-time deadline. For a bounded headless run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_environment_cdd.ps1 -Headless -Duration 3600
```

`-Headless` without a duration uses 3600 simulated seconds. To reproduce the
older two-object-per-role experiment, pass `-TasksPerRobot 2 -Duration 600`.
`-PerceptionBackend mock` explicitly tests cleanup without learned inference.

Progress is displayed in the task window and saved every five simulated
seconds to `isaac_sim/scenes/environment_cdd_report.cleanup.json`. The full
final report remains `environment_cdd_report.json`, with a `cleanup` section:
`floor_clean`, total/pending/assigned/stored/failed counts, remaining object
paths/reasons, and reserved rack footprints. Vision predictions retain their
separate `.vision.jsonl` log.

"Floor clean" means every registered object has a verified rack placement and
the final stored-object check still passes. Unreachable objects, insufficient
rack space, unsupported components or manipulation failures keep this flag
false. When no active robot can make further progress, the controller stops
with the remaining work listed. It does not claim full cleanup because workers
are idle. A manipulated object that fails verification is quarantined rather
than being assigned again through an obsolete source path. This mode remains
an assisted-grasp simulation controller and a priority baseline, not QMIX.

The earlier seven-placement result in `reports/` belongs to the bounded role
demo. It is not evidence that all 88 objects were cleaned by continuous mode.

Validation: automated tests cover exclusive object/footprint reservations,
more than two jobs, retry-after-progress, independent concurrent routes,
crossing/head-on path conflicts, stationary peers and carried-load clearance.
An Isaac single-robot run verified three successive stored objects; a later
pickup encountered an arm-tracking failure. A five-robot run with learned
camera inference verified three stored objects before its 400-second simulated
time limit. Its fixed camera remained unchanged. Neither run cleaned all 88
objects. See `reports/continuous_cleanup_validation.json` for the measured
results and incomplete-work counts. Those earlier tests used serial travel.
Concurrent navigation measurements are saved in the newer run's `navigation`
section: `peak_moving_robots`, `overlap_frames`, `peak_reserved_routes`, and
time-stamped reservation events. In the 110-second concurrent test, all five
robots started navigation; peak simultaneous moving robots was three, with
5,604 overlapping-motion physics frames. Three objects remained verified on
racks. This motion test used explicit mock perception; the learned classifier
connection was verified in the earlier separate run. All 64 automated tests
pass. Full cleanup remains unverified.
