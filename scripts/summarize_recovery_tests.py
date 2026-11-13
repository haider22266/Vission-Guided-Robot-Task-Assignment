"""Export measured recovery tests without publishing large per-frame logs."""
import json
from pathlib import Path

root=Path(__file__).resolve().parents[1]
tests=[]
for name in ('reassignment_recovery_test','recovery_fleet_test'):
    source=root/'results/cdd'/f'{name}.json'
    if not source.exists():
        continue
    report=json.loads(source.read_text(encoding='utf-8'))
    tests.append(dict(test=name,perception_backend=report['research_pipeline']['perception_backend'],
        simulated_seconds=report['simulated_seconds'],counts=report['cleanup']['counts'],
        floor_clean=report['cleanup']['floor_clean'],camera_unchanged=report['camera_unchanged'],
        navigation={k:v for k,v in report['navigation'].items() if k!='route_events'},
        robots=[dict(robot=r['robot'],failure=r['failure'],recoveries=r.get('recoveries',[]),
                     verified_placements=r['completed_jobs']) for r in report['robots']]))
output=dict(tests=tests,interpretation=(
    'Explicit mock perception isolates motion and recovery; these are not classifier-accuracy tests. '
    'The injected-failure test verifies physical floor recovery, reassignment to another robot, '
    'and checked rack placement of the same source object. Its single-object diagnostic does not clean the scene. '
    'Only existing racks are used; oversized beams remain blocked. '
    'A time-limited fleet run does not establish full cleanup; inspect counts and failures.'))
(root/'reports/recovery_controller_validation.json').write_text(json.dumps(output,indent=2)+'\n',encoding='utf-8')
