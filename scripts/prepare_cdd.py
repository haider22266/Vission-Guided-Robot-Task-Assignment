"""Inventory CDD without modifying images; preserve the classifier class order.

Relative dataset paths in disaster_classes.yaml are resolved from the workspace.
This creates labeled metadata, not a trained model or an evaluation split.
"""
import argparse
import json
from pathlib import Path


def main():
    workspace=Path(__file__).resolve().parents[1]
    config=json.loads((workspace/'src/disaster_mrta/config/disaster_classes.yaml').read_text(encoding='utf-8'))
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-root',type=Path,default=workspace/config['dataset_path'])
    parser.add_argument('--output',type=Path,default=workspace/'results/cdd')
    args=parser.parse_args()
    root=args.dataset_root.resolve()
    rows=[]
    counts={}
    for index,label in enumerate(config['classes']):
        folder=root/config['dataset_folders'][label]
        if not folder.is_dir():
            parser.error(f'Missing class folder: {folder}')
        images=sorted(p for p in folder.rglob('*') if p.is_file() and p.suffix.lower() in ('.png','.jpg','.jpeg','.bmp','.webp'))
        if not images:
            parser.error(f'No images in {folder}')
        counts[label]=len(images)
        rows.extend({'image':p.relative_to(root).as_posix(),'class_index':index,'ground_truth':label} for p in images)
    args.output.mkdir(parents=True,exist_ok=True)
    summary={'dataset_root':str(root),'class_names':config['classes'],'counts':counts,'total_images':len(rows),
             'manifest':'manifest.jsonl','path_base':'dataset_root',
             'note':'Whole-image labels only. Human Damage is an area-search clue, not a localized person detection. No train/test split or model training performed.'}
    (args.output/'manifest.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows),encoding='utf-8')
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
