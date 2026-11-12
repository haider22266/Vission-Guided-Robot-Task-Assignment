The classifier source dataset is
`D:\Phd\2nd Journal\Disaster Image Dataset\archive\Comprehensive Disaster Dataset(CDD)`.
The configured path in `src/disaster_mrta/config/disaster_classes.yaml` is
relative to the workspace. It also resolves from this workspace under WSL.

Prepare the image inventory from PowerShell:

```powershell
py -3.11 scripts/prepare_cdd.py
```

This writes `results/cdd/manifest.jsonl` and `results/cdd/summary.json` without
modifying source images. Each manifest row contains a path relative to the
dataset root, a class index and a whole-image ground-truth label.

The model output order matches the ROS perception configuration:

| Index | Folder | Model label | Images |
| --- | --- | --- | ---: |
| 0 | Damaged_Infrastructure | Damaged Infrastructure | 1454 |
| 1 | Fire_Disaster | Fire Disaster | 933 |
| 2 | Human_Damage | Human Damage | 241 |
| 3 | Land_Disaster | Land Disaster | 657 |
| 4 | Non_Damage | Non-Damage | 9237 |
| 5 | Water_Disaster | Water Disaster | 1035 |

Total: 13,557 images. The classes are imbalanced; most images are Non-Damage.
The inventory itself does not train a model. The training pipeline in
`TRAIN_CDD.md` creates duplicate-aware stratified splits, trains EfficientNet
and reports per-class metrics as well as overall accuracy.

CDD supplies classifier images; NuRec Galileo supplies the 3D environment.
The current NuRec task launcher uses synthetic risk scores. The trained CDD
checkpoint is `models/cdd/efficientnet_cdd.keras`; the existing-warehouse
integration uses it for live camera inference (see `ENVIRONMENT_CDD_PROJECT.md`).
The separate ROS perception node can also load that checkpoint with matching
class order and preprocessing; see `ROS2_PIPELINE.md`.
The dataset folder itself is not a model checkpoint. Human_Damage remains a
whole-image area-search clue, without victim localization.
