"""Send real held-out RGB images through the same worker used by Isaac cameras."""
import base64
import json
from pathlib import Path
import subprocess
import sys
from PIL import Image
import numpy as np

root=Path(__file__).resolve().parents[1]
metadata=json.loads((root/'models/cdd/efficientnet_cdd.metadata.json').read_text())
splits=json.loads((root/'results/cdd/training/splits.json').read_text())
dataset=Path(metadata['split_summary']['dataset_root'])
process=subprocess.Popen([sys.executable,str(root/'scripts/cdd_inference_worker.py'),'--model',str(root/'models/cdd/efficientnet_cdd.keras')],
                         stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
results=[]
try:
    for label in range(6):
        row=next(row for row in splits['test'] if row['label']==label)
        with Image.open(dataset/row['image']) as image:rgb=np.array(image.convert('RGB'))
        request={'robot_id':label+1,'shape':list(rgb.shape),'rgb':base64.b64encode(rgb.tobytes()).decode('ascii')}
        process.stdin.write(json.dumps(request)+'\n')
        process.stdin.flush()
        result=json.loads(process.stdout.readline())
        assert 'error' not in result,result
        assert list(result['class_scores'])==metadata['class_names']
        assert abs(sum(result['class_scores'].values())-1)<1e-5
        assert result['source']=='efficientnet'
        results.append({'image':row['image'],'ground_truth':metadata['class_names'][label],'result':result})
    (root/'results/cdd/protocol_check.json').write_text(json.dumps(results,indent=2)+'\n')
    print('PASS: six real held-out images -> deployed worker -> six normalized probabilities in canonical order',flush=True)
finally:
    process.stdin.close()
    try:process.wait(timeout=10)
    except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=10)
