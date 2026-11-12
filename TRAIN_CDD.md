Train CDD from the workspace root:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\train_cdd.ps1 -Setup
```

After setup, omit `-Setup` for subsequent runs. Training uses the workspace's
isolated `.venv-cdd-wsl` environment in Ubuntu-22.04 and its NVIDIA GPU. The
setup script is configured for this machine's Python/pip bootstrap path.
Dependencies are recorded in `results/cdd/training_requirements.txt`.

The classifier is EfficientNetB0 initialized with ImageNet weights, followed
by dropout and a six-class softmax head. Frozen features train the head for
up to 30 epochs with early stopping; the last 20 backbone layers are then
eligible for three fine-tuning epochs, with batch normalization frozen.
Class weights use inverse training frequencies. Checkpoint selection uses
validation macro F1; the held-out test set is scored after selection.

CDD's 13,557 source files are decoded to RGB and hashed before splitting.
The audit excludes 2 unreadable images, 141 duplicate copies and 78 images
whose identical decoded content has conflicting class labels. The remaining
13,336 images are stratified into 9,336 training, 2,000 validation and 2,000
test images with seed 42. Source images are unchanged. Split membership and
exclusions are saved under `results/cdd/training/`.

Exact decoded-content duplicates cannot cross these splits. Near-duplicates
and images from the same event may still cross because event IDs are not
provided. The reported metrics measure this held-out CDD image split;
they do not measure generalization to new events or Isaac camera accuracy.

Training writes:

- `models/cdd/efficientnet_cdd.keras`: deployable model.
- `models/cdd/efficientnet_cdd.metadata.json`: class order, preprocessing,
  checkpoint checksum and training details.
- `results/cdd/training/test_metrics.json`: accuracy, balanced accuracy,
  macro F1, per-class metrics and confusion matrix.
- `results/cdd/training/test_predictions.jsonl`: individual held-out results.

The checkpoint accepts RGB values 0–255 at 224×224. EfficientNet's rescaling
is inside the model; do not divide the input by 255 again. The canonical class
order is the order in `CDD_DATASET.md`. See the official
[EfficientNet documentation](https://keras.io/api/applications/efficientnet/)
and [TensorFlow installation guide](https://www.tensorflow.org/install/pip).

Run the existing warehouse with the trained checkpoint:

```powershell
powershell.exe -ExecutionPolicy Bypass -File .\run_environment_cdd.ps1
```

The launcher defaults to learned EfficientNet perception and the local
checkpoint above. It starts inference in a separate WSL process, because
Windows Application Control blocked this machine's native TensorFlow DLLs.
It never falls back to mock predictions. To explicitly run a transport demo,
pass `-PerceptionBackend mock`.

The five mounted cameras stay in the original Isaac stage and send live RGB
frames to the model. Predictions are whole-image work-area clues. Human Damage
does not localize a person. The current priority worker scheduler
uses the risk scores, while existing object/rack reservations, navigation,
grasp and placement verification remain in control. This does not enable
QMIX or learned role switching. See `CONTINUOUS_CLEANUP.md` for the shared
object queue used by the default continuous-cleanup mode.
