# Disaster-response robotics with CDD and Isaac Sim 5.1

Five Ridgeback–Franka robots operate in an existing warehouse with props and
racks. Mounted RGB cameras send live images to a separately running EfficientNetB0
classifier trained on the Comprehensive Disaster Dataset (CDD). Predicted risks
adjust the existing role-constrained worker scheduling priority. The controller
retains object reservations, navigation, contact checks and assisted rack placement.

The current warehouse integration uses a priority baseline. It does not run
QMIX or change robot roles. The separate ROS 2 implementation and policy
interface are described in [ROS2_PIPELINE.md](ROS2_PIPELINE.md).

## Recorded results

- Held-out CDD test set: 2,000 images; 95.95% accuracy, 91.65% balanced accuracy,
  and 0.889 macro F1. Exact decoded-image duplicates are separated, but related
  events and near-duplicates are not known to be disjoint.
- Latest recorded warehouse run: five successful robot routines, seven objects
  verified on racks, and 338.667 simulated seconds. Camera predictions are
  unlabelled, so these classifier test metrics are not Isaac camera accuracy.
- Human Damage is a whole-image area-search clue, not person localization.

See [the result snapshot](reports/RESULTS_SUMMARY.md) and
[classifier metrics](reports/classifier_test_metrics.json).

## Run the warehouse integration

Requires Isaac Sim 5.1, Windows with WSL2 Ubuntu-22.04, and TensorFlow in a
separate environment. The warehouse scene and helper scripts are included in
`isaac_sim/warehouse_source`; its NVIDIA assets remain external references.

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_environment_cdd.ps1 -IsaacRoot 'C:\isaac-sim'
```

The default expects `models/cdd/efficientnet_cdd.keras` plus its metadata and
`.venv-cdd-wsl`. The trained checkpoint, Python environments, raw CDD images,
NuRec data and generated scene files are excluded from Git. Train the model
using [TRAIN_CDD.md](TRAIN_CDD.md), or restore it from the separately shared
research ZIP. [ENVIRONMENT_CDD_PROJECT.md](ENVIRONMENT_CDD_PROJECT.md) explains
the camera pipeline and configuration.

For an explicit mock-perception integration run without trained weights:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_environment_cdd.ps1 -IsaacRoot 'C:\isaac-sim' -PerceptionBackend mock
```

## Code and data guide

- `isaac_sim/environment_cdd_project.py`: existing warehouse controller with integration hooks.
- `isaac_sim/environment_cdd_runtime.py`: mounted cameras, inference transport and priority scheduling.
- `scripts/train_cdd_classifier.py`: CDD auditing, splitting, training and evaluation.
- `scripts/cdd_inference_worker.py`: separate-process whole-image inference.
- `src/disaster_mrta/`: task rules, ROS 2 nodes, QMIX inference interface and tests.
- `reports/`: selected recorded results; full generated data is distributed separately.

Dataset layout and class order: [CDD_DATASET.md](CDD_DATASET.md).
Earlier procedural and NuRec scenes: [isaac_sim/README.md](isaac_sim/README.md).
To build the full code/model/results archive locally, run
`py -3.11 scripts/export_research_package.py` after generating its required results.
