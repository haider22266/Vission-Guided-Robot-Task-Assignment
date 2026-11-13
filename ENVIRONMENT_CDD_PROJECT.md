The integration entry point is `isaac_sim/environment_cdd_project.py`, copied
from the supplied `E:\ICAST\Research Project\PhysicalAiRoleSwitchin_IsaacSim\environment_prop_rack_isaacsim51.py`.
It opens the original `warehouse_verified.usd` once and retains the original
five Ridgeback–Franka robots, racks, props, planar drives, navigation, exclusive
object reservations and contact-verified assisted rack placement. All changes
use the source controller's session layer. The source Python and USD files
are unchanged. The scene and helper scripts are also copied into
`isaac_sim/warehouse_source`, which is now used by default so the project can
be shared. Set `WAREHOUSE_SOURCE_DIR` to override that directory. NVIDIA asset
references remain external; see `results/cdd/scene_asset_references.json`.

Run from this workspace in PowerShell:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_environment_cdd.ps1
```

The default now uses continuous floor cleanup (`-TasksPerRobot 0`) rather than
stopping after two objects per robot. See [CONTINUOUS_CLEANUP.md](CONTINUOUS_CLEANUP.md)
for queue behavior, rack reservations and remaining-work reports. Pass
`-TasksPerRobot 2 -Duration 600` to reproduce the earlier bounded role demo.
All five workers run concurrently by default. Navigation reservations only
delay conflicting routes; local obstacle and peer/load avoidance remain active.
The optional `-Serial` flag is for one-at-a-time debugging.

The GUI shows the fixed warehouse viewport, the task window, and a docked
**Live robot cameras and CDD predictions** panel with five RGB feeds, last
predicted class/confidence, risk scores, scheduling priority and current task.
Camera and prediction timestamps are shown separately, because inference can
lag behind live frames. Live images continue refreshing while inference is
busy. `-Headless` intentionally hides all visualization.

Close an existing simulation before restarting after code changes. The
launcher checks for an already running warehouse instance: two full instances
can exceed GPU memory. The original window can be restored from the Windows
taskbar (title: Isaac Sim Python 5.1.0). A full GUI validation of the new panels
was blocked by GPU memory exhaustion while another warehouse instance was
running; that test instance was stopped. Python logic tests cover live-frame
refresh during pending inference and timestamp/mock labeling.

The default is **trained EfficientNet perception**, using
`models/cdd/efficientnet_cdd.keras` and the isolated `.venv-cdd-wsl` TensorFlow
environment. The model was trained from the supplied CDD images; the dataset
folder itself is not a checkpoint. Five
mounted RGB cameras render at 224×224. They do not control the fixed viewport.
Each rendered batch is sampled about once per simulated second. Inference
allows only one outstanding batch so frames cannot accumulate indefinitely.
Vision records are written beside `isaac_sim/scenes/environment_cdd_report.json`
in `environment_cdd_report.vision.jsonl`.

The default command starts the classifier in a separate WSL process. Windows
Application Control blocked native TensorFlow DLL loading on this machine.
For a different checkpoint and a working native Windows TensorFlow environment:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_environment_cdd.ps1 `
  -PerceptionBackend efficientnet `
  -Model 'D:\path\to\trained_cdd_model.keras' `
  -ClassifierBackend windows `
  -ClassifierPython 'D:\path\to\training_env\python.exe'
```

The model must output six class probabilities in the order documented in
`CDD_DATASET.md`. Preprocessing uses 224×224 RGB with values 0–255, matching
the ROS adapter's default; models trained with different preprocessing need
that adapter adjusted. The classifier runs in a separate process through
`scripts/cdd_inference_worker.py` (through `scripts/run_cdd_worker.sh` for WSL).
Checkpoint metadata is checked for class order and preprocessing compatibility.
There is no automatic fallback to mock predictions. For an explicit transport
test, pass `-PerceptionBackend mock`.

The selected EfficientNetB0 model achieved 95.95% accuracy, 91.65% balanced
accuracy and 0.889 macro F1 on 2,000 held-out CDD images. These are CDD image
metrics, not measured Isaac camera accuracy. The split excludes exact decoded
duplicates but is not known to separate related events or near-duplicates.
Training commands, exclusions, methodology and metric files are documented in
`TRAIN_CDD.md`.

The adapter associates each camera's whole image with its robot's current work
area. Hazard or Human Damage scores increase its worker scheduling priority
from 0.5 toward 1.0. Higher-priority workers issue commands first within each
shared physics frame, while the original controller retains its corridor,
object and rack reservations. This is a **priority baseline**, not QMIX.
Continuous mode first prefers each robot's original task family, then lets
idle robots take other available objects from the shared cleanup queue.
It is separate from the ROS 2 nodes in `ROS2_PIPELINE.md`; this entry point
does not yet expose ROS control, learned role switching or a failure monitor.
Human Damage is an area clue, not a localized person detection, and visual
scores do not constitute object-specific evidence or a verified safe route.

The original source controller remains responsible for successful task
completion. Camera inference cannot bypass its contact, lift or placement
checks. The normal GUI remains open after tasks finish; `-ExitOnComplete`
closes it. `-Headless -Duration 2` is a short camera/motion smoke test and
normally exits with unfinished-task status because two seconds is insufficient
for rack placement. It must not be reported as a successful task-completion run.

The integration test on the supplied F: Isaac Sim 5.1 installation read all
five camera streams and produced learned EfficientNet area clues in the
original stage. The inference-worker protocol was also verified with six real
held-out images, one from each class. The limited-duration camera test does
not establish full rack-placement success or prediction correctness on the
simulated images.
