"""Export measured negative/positive placement regressions for the rack fix."""
import json
from pathlib import Path

root=Path(__file__).resolve().parents[1]
rows=[]
for name in ('stuck_beam_regression','placement_reach_regression'):
    report=json.loads((root/'results/cdd'/(name+'.json')).read_text())
    focus=report['cleanup']['diagnostic_object_filter']
    rows.append({'test':name,'object':focus,'perception_backend':report['research_pipeline']['perception_backend'],
                 'simulated_seconds':report['simulated_seconds'],'floor_clean':report['cleanup']['floor_clean'],
                 'reserved_source_paths':report['reserved_original_paths'],
                 'remaining_target':[item for item in report['cleanup']['remaining_objects'] if item['id'].rsplit('/',1)[-1]==focus],
                 'verified_placements':[job for robot in report['robots'] for job in robot['completed_jobs']],
                 'camera_unchanged':report['camera_unchanged']})
(root/'reports/stuck_robot_fix_validation.json').write_text(json.dumps({'tests':rows,'interpretation':
    'The oversized beam is rejected before pickup with no object reservation or false successful placement. '
    'The carton test verifies one supported placement after actual-grasp IK preflight. '
    'Both tests use explicit mock perception to isolate manipulation. Neither is a full floor-cleanup test. '
    'Small docking-adjustment branches are implemented but were not exercised by the successful zero-offset carton case.'},indent=2)+'\n')
