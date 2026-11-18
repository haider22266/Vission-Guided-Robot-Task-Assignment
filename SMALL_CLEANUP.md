Run the small-prop version with 20 tasks from the workspace root:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_small_cleanup.ps1 -TaskCount 20
```

Close any other warehouse simulation first. This opens the existing warehouse,
five Ridgeback–Franka robots and original racks with 20 coloured-box tasks,
released in four batches of five 24 cm boxes. Each robot can claim a box, approach, verify surface contact,
lift, carry to a nearby feasible rack, release and verify support. Reservations
prevent duplicate assignments. Robot cameras are off and the viewport stays
fixed. The GUI stays open with `FLOOR CLEAN` after all tasks finish. The default
is 20 tasks, with only five boxes on the floor at a time; `-TaskCount 5` reproduces the original
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
All 92 automated tests pass. These results apply to four batches of five, not
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
