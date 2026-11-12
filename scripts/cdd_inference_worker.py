"""JSON-lines whole-image inference, isolated from the Isaac Sim process."""
import argparse
import base64
import contextlib
import json
from pathlib import Path
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',required=True)
    args=parser.parse_args()
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/disaster_mrta'))
    # Reserve stdout for protocol messages, including framework initialization.
    with contextlib.redirect_stdout(sys.stderr):
        import numpy as np
        import tensorflow as tf
        tf.config.threading.set_intra_op_parallelism_threads(4)
        tf.config.threading.set_inter_op_parallelism_threads(2)
        for gpu in tf.config.list_physical_devices('GPU'):
            tf.config.experimental.set_memory_growth(gpu,True)
        from disaster_mrta.perception_model import EfficientNetModel,CLASSES,vision_result
        model=EfficientNetModel(args.model,list(CLASSES))
    for line in sys.stdin:
        try:
            request=json.loads(line)
            rgb=np.frombuffer(base64.b64decode(request['rgb']),dtype=np.uint8).reshape(request['shape'])
            with contextlib.redirect_stdout(sys.stderr):
                probabilities=model.predict(rgb)
            result=vision_result(probabilities,list(CLASSES),'robot_work_area',request['robot_id'])
        except Exception as exc:
            result={'error':str(exc)}
        print(json.dumps(result),flush=True)


if __name__=='__main__':
    main()
