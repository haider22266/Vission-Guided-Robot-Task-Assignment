To show all 20 cartons together from the start, run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_small_cleanup.ps1 -TaskCount 20 -Layout grouped
```

This is the default layout: five visible groups of four brown cartons with
tape bands, rather than staged task releases. All 20 are immediately pending,
visible and colliding. No later batches are introduced. Use `-Layout pile`
for one shared group. The groups have 0.90 m centre spacing and 24 cm cartons;
their gaps provide approach options without overlapping source geometry.
The grouped layout completed a headless Isaac Sim 5.1 test in 556.217 simulated
seconds: 20 stored, zero pending, assigned, waiting, blocked or failed. All 20
remained on their racks at the final recheck. Peak simultaneously moving robots
was five, with 21,192 overlapping-motion frames. Task counts per robot were
6, 4, 3, 2 and 5 because reassignment can cross groups. See
[reports/grouped_cleanup_20_validation.json](reports/grouped_cleanup_20_validation.json).
This is one measured grouped-layout run, not a validation of the optional
single-pile layout or the original dense 88-prop scene. All 95 logic tests pass.
Each robot first considers its nearby group, then other available tasks if
its own group is infeasible or already assigned. Exclusive reservations still
prevent duplicate work. Traffic-only floor recoveries do not count against the
three-attempt manipulation-failure limit; verified grasp/settling and safe
recovery requirements are unchanged.

For the previously validated batched version, run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_small_cleanup.ps1 -TaskCount 20 -Layout batched
```

Close any other warehouse simulation first, or add `-Restart` to gracefully
close an existing warehouse window and start a
fresh run. If shutdown has not finished after 30 seconds, the launcher stops
instead of starting a duplicate. Restarting resets the current run's progress.
This opens the existing warehouse,
five Ridgeback–Franka robots and original racks with 20 coloured-box tasks,
released in four batches of five 24 cm boxes. Each robot can claim a box, approach, verify surface contact,
lift, carry to a nearby feasible rack, release and verify support. Reservations
prevent duplicate assignments. Robot cameras are off and the viewport stays
fixed. The GUI stays open with `FLOOR CLEAN` after all tasks finish. The default
for this batched option is 20 tasks, with only five boxes on the floor at a time; `-TaskCount 5 -Layout batched` reproduces the original
five-box demo. Counts 10 and 15 are also available. The robots draw from a
shared exclusive queue, so each can collect multiple boxes. Future batches
remain hidden and non-colliding in `waiting` state until the preceding batch
is no longer pending or assigned and all robots have released their loads and
returned to their starting areas.
Introducing a new box is task initialization, not a successful pickup or
placement. Completed boxes stay on the racks; the ledger must reach 20 stored
and zero waiting/pending/failed objects before reporting completion.

The dense original disaster-floor props are hidden and their collisions disabled
before simulation starts, only in the session layer. They are excluded from this
version's queue. Original warehouse stock and racks remain. The source USD and
the original 88-task launcher are unchanged. The new boxes are not teleported
to their destinations: carrying still uses contact-verified assisted grasps and
physical robot motion. Contact probes allow a motion-arrival tolerance against
the surface, while pickup still requires a visible gap below 3 mm and measured
contact force above 0.02 N. Source-pose, lift, settling and final-storage checks
remain enabled. Robot/prop collision conversion reads the session layer because
these demo boxes are authored there.

The 20-task Isaac Sim 5.1 headless validation completed all 20 tasks in 473.017
simulated seconds. Each robot stored four boxes. All 20 remained on their racks
at final recheck; stored 20, pending 0, assigned 0, waiting 0, blocked 0, failed 0.
Peak simultaneously moving robots: five; 13,440 physics frames had overlapping
motion. See [reports/small_cleanup_20_validation.json](reports/small_cleanup_20_validation.json).
The small-demo traffic escape uses the physical base footprint plus clearance
when leaving inflated planning margins, with peer-load obstacles retained.
These results apply to four batches of five, not
20 simultaneous floor props or the original 88-prop scene.

The earlier five-task validation completed all five tasks in 60.267
simulated seconds. Every robot stored one box and all five remained supported
on their assigned racks at final recheck. Final counts: stored 5, pending 0,
assigned 0, blocked 0, failed 0. Peak simultaneously moving robots: 5; 1,555
physics frames had overlapping motion. Placement errors were 3.4–4.6 mm.
See [reports/small_cleanup_validation.json](reports/small_cleanup_validation.json).
These five-box results apply to `-TaskCount 5`, not the 20-box version or the
original 88 props.
GUI wall-clock duration can differ from the headless test.
The small-demo launcher caps Isaac's CPU worker pool at eight threads and uses
a 1280×720 starting window, following a GUI LLVM out-of-memory failure on this
machine. The larger-scene application's defaults are unchanged.

For a bounded automated run:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_small_cleanup.ps1 -Headless -Duration 600
```

Use `-IsaacRoot 'C:\isaac-sim'` if that is your installation. Use
`-ExitOnComplete` to close the GUI after the tasks finish. Reports are written
to `isaac_sim/scenes/small_cleanup_report.json` and its `.cleanup.json` sibling.
The entry point is `isaac_sim/environment_small_tasks.py`; the reduced scene
setup is `isaac_sim/small_cleanup_scene.py`. The common controller provides the
same navigation and placement checks as the larger project.
