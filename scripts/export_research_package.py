"""Export code, model and recorded results; omit environments and raw datasets."""
import csv
from datetime import datetime, timedelta, timezone
import hashlib
import io
import json
from pathlib import Path
import zipfile


ROOT=Path(__file__).resolve().parents[1]


def csv_bytes(rows,fields):
    stream=io.StringIO(newline='')
    writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore')
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode('utf-8-sig')


def main():
    timestamp=datetime.now(timezone(timedelta(hours=9)))
    name='cdd_research_package_'+timestamp.strftime('%Y%m%d_%H%M%S')
    output=ROOT/'deliverables'
    tables=output/name/'exported_results'
    tables.mkdir(parents=True,exist_ok=True)
    primary_paths=['isaac_sim/scenes/environment_cdd_report.json',
                   'isaac_sim/scenes/environment_cdd_report.vision.jsonl',
                   'results/cdd/training/test_metrics.json']
    # Snapshot the primary reports once, so summary and ZIP agree.
    primary={path:(ROOT/path).read_bytes() for path in primary_paths}
    report=json.loads(primary[primary_paths[0]])
    vision=[json.loads(line) for line in primary[primary_paths[1]].decode('utf-8').splitlines() if line.strip()]
    metrics=json.loads(primary[primary_paths[2]])
    metadata=json.loads((ROOT/'models/cdd/efficientnet_cdd.metadata.json').read_text())
    classes=metadata['class_names']
    robot_rows=[]
    placement_rows=[]
    for robot in report['robots']:
        jobs=robot.get('completed_jobs',[])
        completions=[event['time'] for event in robot['transitions'] if event['state']=='completed_original_prop_on_rack']
        robot_rows.append(dict(robot=robot['robot'],task=robot['task'],success=robot['success'],
            completed_jobs=len(jobs),stored_jobs=sum(bool(job.get('still_stored')) for job in jobs),
            started_at_s=robot.get('started_at_s'),first_motion_at_s=robot.get('first_motion_at_s'),
            last_placement_at_s=max(completions) if completions else None,
            travel_distance_m=robot.get('travel_distance_m'),no_more_reachable_props=robot.get('no_more_reachable_props'),
            failure=robot.get('failure')))
        for index,job in enumerate(jobs):
            contact=job.get('contact_check',{})
            placement_rows.append(dict(robot=robot['robot'],task=robot['task'],job_index=index,
                source_props=';'.join(job['source_props']),rack_path=job['rack_path'],still_stored=job.get('still_stored'),
                completed_at_sim_s=completions[index] if index<len(completions) else None,
                contact_verified=contact.get('verified'),max_visible_gap_m=max(contact.get('visible_gaps_m',[0.])),
                min_contact_force_N=min(contact.get('forces_N',[0.])),
                lift_rise_m=job.get('lift_check',{}).get('rise_m'),placement_error_m=job.get('placement_error_m'),
                support_gap_m=job.get('support_gap_m'),final_speed_m_s=job.get('final_speed_m_s')))
    files={}
    def table(filename,rows,fields):files['exported_results/'+filename]=csv_bytes(rows,fields)
    table('robot_task_results.csv',robot_rows,list(robot_rows[0]))
    table('object_placements.csv',placement_rows,list(placement_rows[0]))
    table('classifier_per_class_metrics.csv',[dict(class_name=name,**metrics['classification_report'][name]) for name in classes],
          ['class_name','precision','recall','f1-score','support'])
    table('classifier_confusion_matrix.csv',[dict(true_class=name,**dict(zip(classes,row))) for name,row in zip(classes,metrics['confusion_matrix'])],
          ['true_class']+classes)
    live=[]
    for row in vision:
        live.append(dict(robot_id=row['robot_id'],simulation_time=row.get('simulation_time'),
            predicted_class=row['predicted_class'],confidence=row['confidence'],hazard_risk=row['hazard_risk'],
            rescue_signal=row['rescue_signal'],source=row['source'],**row['class_scores']))
    table('live_camera_predictions.csv',live,['robot_id','simulation_time','predicted_class','confidence','hazard_risk','rescue_signal','source']+classes)
    camera_frames=report.get('research_pipeline',{}).get('camera_frames',{})
    table('camera_record_counts.csv',[{'robot_id':rid,'camera_samples':count,
        'logged_predictions':sum(str(row['robot_id'])==str(rid) for row in vision)} for rid,count in camera_frames.items()],
        ['robot_id','camera_samples','logged_predictions'])
    predictions=[]
    for line in (ROOT/'results/cdd/training/test_predictions.jsonl').read_text().splitlines():
        row=json.loads(line)
        predictions.append(dict(image=row['image'],ground_truth=classes[row['label']],predicted_class=row['predicted_class'],
                                **dict(zip(classes,row['probabilities']))))
    table('classifier_test_predictions.csv',predictions,['image','ground_truth','predicted_class']+classes)
    task_table='\n'.join(f"| {r['robot']} | {r['task']} | {r['completed_jobs']} | {r['stored_jobs']} | {r['success']} |" for r in robot_rows)
    summary=f'''Recorded results snapshot: {timestamp.isoformat(timespec='seconds')} (Asia/Tokyo).

Classifier: EfficientNetB0 trained on CDD. Held-out test images: {metrics['test_images']}.
Accuracy: {metrics['accuracy']:.2%}; balanced accuracy: {metrics['balanced_accuracy']:.2%}; macro F1: {metrics['macro_f1']:.4f}.
Split: 9,336 training / 2,000 validation / 2,000 test images. Excluded:
2 unreadable files, 141 duplicate copies and 78 conflicting-label images.
Exact decoded-content duplicates are disjoint. Related events/near-duplicates
are not known to be disjoint. These are CDD metrics, not Isaac camera accuracy.

Isaac Sim recorded duration: {report['simulated_seconds']} simulated seconds.
Fixed viewport camera unchanged: {report['camera_unchanged']}.
Successful robot routines: {sum(bool(r['success']) for r in robot_rows)}/{len(robot_rows)}.
Completed object placements: {len(placement_rows)}; verified still stored: {sum(bool(j['still_stored']) for j in placement_rows)}.
Each routine requests up to {report['requested_max_tasks_per_robot']} objects; a routine can stop when no reachable candidate remains.

| Robot | Task | Completed placements | Still stored | Routine success |
| --- | --- | ---: | ---: | --- |
{task_table}

Logged live-camera predictions: {len(vision)}. See camera_record_counts.csv
for captured-versus-logged counts; a pending final inference batch need not
appear in the prediction log. The camera predictions have no ground-truth
labels here, so simulation detection accuracy cannot be calculated.

Execution uses the original planar-drive physics controller and contact-verified
assisted grasp/placement. Perception changes role-constrained worker scheduling
priority. QMIX, cross-role task reassignment and hardware execution are not
claimed for this run. Human Damage is a whole-image area clue, not a localized
person detection. Raw JSON reports retain routes, transitions and grasp checks.
'''
    files['RESULTS_SUMMARY.md']=summary.encode('utf-8')
    guide='''This package contains the project code, trained classifier and recorded results.

Start with RESULTS_SUMMARY.md and exported_results/*.csv (Excel-compatible).
The authoritative records are isaac_sim/scenes/environment_cdd_report.json,
its .vision.jsonl file, and results/cdd/training/test_metrics.json.
Other scene reports and ROS results are retained as separate legacy/integration
experiments; do not combine them into the current learned-camera run.

Code: isaac_sim/environment_cdd_project.py, environment_cdd_runtime.py,
scripts/train_cdd_classifier.py, scripts/cdd_inference_worker.py and src/.
Model: models/cdd/efficientnet_cdd.keras, with metadata and training weights.
Splits/exclusions/predictions: results/cdd/training/.
Original warehouse code/helpers and scene: isaac_sim/warehouse_source/.

Extract the archive and open PowerShell in disaster_mrta_cdd (the folder
containing run_environment_cdd.ps1). Install Isaac Sim 5.1 and Ubuntu-22.04
under WSL2. GPU training needs a compatible NVIDIA driver. Virtual environments
and simulator installations are not included. With Python3 venv/pip available
in WSL, create the classifier environment:

    wsl -d Ubuntu-22.04 -- bash scripts/setup_cdd_wsl.sh

That script includes a pip-wheel fallback configured for the original machine;
on another machine install python3-venv/pip or set CDD_PIP_WHEEL as documented
by its error. Then run (replace the Isaac installation path as necessary):

    powershell.exe -ExecutionPolicy Bypass -File .\\run_environment_cdd.ps1 -IsaacRoot 'C:\\isaac-sim'

The trained checkpoint is selected by default; inference runs in a separate WSL
process. -PerceptionBackend mock explicitly selects a transport test instead.
TRAIN_CDD.md documents retraining; it requires the original CDD image dataset,
which is not copied into this archive. Change --dataset-root when its location
differs. Dataset manifests preserve relative image paths and class labels.

The included warehouse USD references NVIDIA assets at external URLs listed
in results/cdd/scene_asset_references.json. Those asset files and the NuRec
dataset are not bundled; access to them is required for the relevant scenes.
No raw camera images/video were saved in the recorded run; the package contains
the prediction records and robot/task telemetry that were actually collected.

FILE_MANIFEST.json contains SHA-256 checksums for every packaged data/code file
except the manifest itself. Documentation describes current limitations.
'''
    files['PACKAGE_README.md']=guide.encode('utf-8')
    code_suffixes={'.py','.ps1','.sh','.md','.yaml','.yml','.json','.xml','.cfg','.txt','.toml'}
    selected=set()
    for path in ROOT.iterdir():
        if path.is_file() and (path.suffix in code_suffixes or path.name=='.gitignore'):selected.add(path)
    for folder in ('src','scripts','isaac_sim'):
        for path in (ROOT/folder).rglob('*'):
            if not path.is_file() or any(part in ('__pycache__','.pytest_cache') for part in path.parts):continue
            relative=path.relative_to(ROOT)
            if folder=='isaac_sim' and 'scenes' in relative.parts:
                if path.suffix in ('.json','.jsonl'):selected.add(path)
            elif path.suffix in code_suffixes or path.parent.name=='resource' or relative.as_posix()=='isaac_sim/warehouse_source/warehouse_verified.usd':
                selected.add(path)
    for folder in ('results/cdd/training','results/ros2','results/ros2_transport_test','models/cdd'):
        for path in (ROOT/folder).rglob('*'):
            if path.is_file() and path.suffix in ('.json','.jsonl','.keras','.h5','.log'):selected.add(path)
    for filename in ('manifest.jsonl','summary.json','protocol_check.json','training_requirements.txt',
                     'scene_asset_references.json','training.log','protocol_check.log','package_scene_smoke.json'):
        path=ROOT/'results/cdd'/filename
        if path.is_file():selected.add(path)
    for path in sorted(selected):
        relative=path.relative_to(ROOT).as_posix()
        files[relative]=primary.get(relative,path.read_bytes())
    model='models/cdd/efficientnet_cdd.keras'
    if hashlib.sha256(files[model]).hexdigest()!=metadata['checkpoint_sha256']:
        raise ValueError('Checkpoint does not match training metadata')
    manifest={'created_at':timestamp.isoformat(),'files':[{'path':name,'bytes':len(data),
              'sha256':hashlib.sha256(data).hexdigest()} for name,data in sorted(files.items())]}
    files['FILE_MANIFEST.json']=(json.dumps(manifest,indent=2)+'\n').encode('utf-8')
    for name_in_package,data in files.items():
        if name_in_package.startswith('exported_results/') or name_in_package in ('RESULTS_SUMMARY.md','PACKAGE_README.md','FILE_MANIFEST.json'):
            target=output/name/name_in_package
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(data)
    archive=output/(name+'.zip')
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as bundle:
        for relative,data in sorted(files.items()):bundle.writestr('disaster_mrta_cdd/'+relative,data)
    with zipfile.ZipFile(archive) as bundle:
        if bundle.testzip() is not None:raise ValueError('ZIP integrity check failed')
        for entry in manifest['files']:
            data=bundle.read('disaster_mrta_cdd/'+entry['path'])
            if hashlib.sha256(data).hexdigest()!=entry['sha256']:raise ValueError('Checksum mismatch: '+entry['path'])
    print(json.dumps({'archive':str(archive),'bytes':archive.stat().st_size,'files':len(files),
                      'summary':str(output/name/'RESULTS_SUMMARY.md'),'csv_directory':str(tables)},indent=2))


if __name__=='__main__':main()
