"""Summarize actual cleanup validation runs without claiming full floor cleanup."""
import json
from pathlib import Path

root=Path(__file__).resolve().parents[1]
rows=[]
for name in ('continuous_cleanup_single_test','continuous_cleanup_fleet_test'):
    path=root/'results/cdd'/(name+'.json')
    report=json.loads(path.read_text())
    rows.append({'test':name,'simulated_seconds':report['simulated_seconds'],
        'perception_backend':report['research_pipeline']['perception_backend'],
        'camera_unchanged':report['camera_unchanged'],'registered_objects':report['cleanup']['total_objects'],
        'floor_clean':report['cleanup']['floor_clean'],'counts':report['cleanup']['counts'],
        'verified_still_stored':sum(bool(job.get('still_stored')) for robot in report['robots'] for job in robot['completed_jobs']),
        'robots':[{'robot':robot['robot'],'completed_jobs':len(robot['completed_jobs']),'failure':robot['failure']} for robot in report['robots']],
        'stop_reason':report['cleanup']['stop_reason'],'camera_samples':report['research_pipeline']['camera_frames']})
output=root/'reports/continuous_cleanup_validation.json'
output.write_text(json.dumps({'tests':rows,'interpretation':
    'The single-robot run verified three successive stored objects, then encountered a lift/arm tracking failure on a later object. '
    'The five-robot learned-camera run was limited to 400 simulated seconds and did not finish the floor. '
    'These tests verify repeated work and safe incomplete reporting; they do not demonstrate full cleanup of all 88 objects.'},indent=2)+'\n')
print(output)
