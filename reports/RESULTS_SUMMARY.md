Recorded results snapshot: 2026-11-12T16:02:48+09:00 (Asia/Tokyo).

Classifier: EfficientNetB0 trained on CDD. Held-out test images: 2000.
Accuracy: 95.95%; balanced accuracy: 91.65%; macro F1: 0.8891.
Split: 9,336 training / 2,000 validation / 2,000 test images. Excluded:
2 unreadable files, 141 duplicate copies and 78 conflicting-label images.
Exact decoded-content duplicates are disjoint. Related events/near-duplicates
are not known to be disjoint. These are CDD metrics, not Isaac camera accuracy.

Isaac Sim recorded duration: 338.667 simulated seconds.
Fixed viewport camera unchanged: True.
Successful robot routines: 5/5.
Completed object placements: 7; verified still stored: 7.
Each routine requests up to 2 objects; a routine can stop when no reachable candidate remains.

| Robot | Task | Completed placements | Still stored | Routine success |
| --- | --- | ---: | ---: | --- |
| Robot_01 | Recover rear-left carton | 1 | 1 | True |
| Robot_02 | Clear aisle rubble | 2 | 2 | True |
| Robot_03 | Recover screwdriver | 1 | 1 | True |
| Robot_04 | Recover rear-right beam | 1 | 1 | True |
| Robot_05 | Recover rear-right carton | 2 | 2 | True |

Logged live-camera predictions: 1540. See camera_record_counts.csv
for captured-versus-logged counts; a pending final inference batch need not
appear in the prediction log. The camera predictions have no ground-truth
labels here, so simulation detection accuracy cannot be calculated.

Execution uses the original planar-drive physics controller and contact-verified
assisted grasp/placement. Perception changes role-constrained worker scheduling
priority. QMIX, cross-role task reassignment and hardware execution are not
claimed for this run. Human Damage is a whole-image area clue, not a localized
person detection. Raw JSON reports retain routes, transitions and grasp checks.
