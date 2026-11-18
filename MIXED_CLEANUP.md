Run the reduced mixed-prop layout:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_small_cleanup.ps1 -TaskCount 20 -Layout mixed -Restart
```

All 20 props are visible and collidable from the start in five areas:

| Initial robot preference | Task family | Count | Area |
| --- | --- | --- | --- |
| Robot 01 | Rear-left carton | 4 | Rear-left aisle |
| Robot 02 | Aisle rubble | 4 | Left/front aisle |
| Robot 03 | Screwdriver | 4 | Front work area |
| Robot 04 | Rear beam | 4 | Rear/right aisle |
| Robot 05 | Rear-right carton | 4 | Rear-right aisle |

The controller dispatches one task from each family per round, giving five
parallel assignments. The next round is dispatched after current loads are
released and the robots return to their starting areas. Queued props remain
visible and collidable in their original floor locations throughout; no props
are hidden, spawned or repositioned to introduce a round. `waiting` in the HUD
means queued assignment, not an invisible object. Each robot first considers
its family, then other feasible currently dispatched tasks.
Exclusive object and rack-footprint reservations remain active. Existing racks,
warehouse stock, five Ridgeback–Franka robots and fixed camera view are retained.
The original dense floor clutter is hidden and non-colliding only in the
session layer. The source USD is never saved. No props are introduced in hidden batches.

Cartons have tape bands, rubble uses gray blocks, screwdrivers have red handles
and connected metal shafts, and rear beams have orange bodies and visible metal
flanges. These are simplified procedural props, not detailed scanned assets.
The four screwdriver shafts are components of their handles' tasks: 24 physical
components form 20 logical tasks. Beams are deliberately shortened to 62 cm to
fit the existing rack supports. The mixed-layout short beams use reachable
front-edge support positions rather than the long-beam placement offset.
The beams and screwdriver shafts lie along X in their source areas. Their
pickup headings are restricted to the Y directions so that the long dimension
stays crosswise while carrying and approaching a rack, instead of projecting
into shelves ahead of the base. Full route/load checks remain enabled.
Rack reservations include an additional 4 cm around mixed-layout payloads to
leave room for small settling changes; final footprint-conflict checks remain
unchanged. This reduces close packing rather than accepting overlapping loads.

The controller still requires actual visible contact and measured force,
pose-preserving payload conversion, a verified lift, physical carrying, and
checked release/support. Neither props nor task results are moved to racks to
force success. Camera inference is off; this is a motion/scene demo, not QMIX
or a classifier evaluation. The carton-only completion results do not validate
this new mixture. Mixed-layout validation is recorded separately.

The final Isaac Sim 5.1 headless test completed 20/20 mixed tasks in 478.5
simulated seconds, with four tasks completed by each robot. Final recheck:
20 still stored, zero pending, assigned, waiting, blocked or failed. Each of
the five families has four completed tasks. Peak simultaneously moving robots
was five; 14,915 physics frames had overlapping motion. See
[reports/mixed_cleanup_20_validation.json](reports/mixed_cleanup_20_validation.json).
All 101 automated logic tests pass. This is one reduced assisted-grasp demo
run, not validation of the original 88-prop scene or arbitrary disturbances.

The screwdriver group is placed farther left in the front aisle, outside the
central rack's blocked approach region. Rack faces are checked in order of
distance from the prop; right-side beams use a feasible right-side rack face
instead of always being sent around to the left. Slow clearance exits use
measured rotated base footprints plus clearance for both robots, retain
carried-load obstacles, and refuse unsafe geometry. Mixed-layout carrying
waits are longer to avoid dropping a load for short-lived traffic contention.

For a bounded automated run, use `-Headless -Duration 1200`. The small-task
launcher uses that duration by default for headless mixed layouts. GUI mode
stays open after completion. Use `-TaskCount 5` for one prop of each family;
the earlier five-task integration test is archived separately and predates
the final tool-area and rack-face adjustments.
