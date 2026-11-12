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

The existing controller approaches, aligns, verifies contact, performs its
assisted grasp/lift, carries, places and releases the object. It checks support
and settling, then backs out and requests another job. Completed objects stay
on the racks. Queue progress causes previously blocked pending jobs to be
reconsidered. A FIFO gate reserves the shared travel aisles for one complete
pickup/storage/return trip at a time. Other robots wait at their separate home
positions, avoiding opposing carried loads in narrow aisles. This favors
reliable cleanup over simultaneous travel; the older bounded demo retains its
concurrent workers.
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

Validation: 58 automated tests pass, including exclusive object/footprint
reservations, more than two jobs, retry-after-progress and FIFO aisle turns.
An Isaac single-robot run verified three successive stored objects; a later
pickup encountered an arm-tracking failure. A five-robot run with learned
camera inference verified three stored objects before its 400-second simulated
time limit. Its fixed camera remained unchanged. Neither run cleaned all 88
objects. See `reports/continuous_cleanup_validation.json` for the measured
results and incomplete-work counts. Full cleanup remains unverified.
