"""Export measured reduced-prop completion tests without per-frame route logs."""
import argparse
import json
from pathlib import Path

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--count',type=int,choices=(5,20),default=20)
args=parser.parse_args()
name='small_cleanup_validation' if args.count==5 else 'small_cleanup_20_validation'
source=root/'results/cdd'/(name+'.json')
report=json.loads(source.read_text(encoding='utf-8'))
summary=dict(scenario=report['scenario'],simulated_seconds=report['simulated_seconds'],
    total_tasks=args.count,active_floor_boxes_per_batch=5,
    release_policy='Four batches of five' if args.count==20 else 'One batch of five',
    counts=report['cleanup']['counts'],all_tasks_stored=report['cleanup']['floor_clean'],
    camera_unchanged=report['camera_unchanged'],perception=report['research_pipeline']['perception_backend'],
    navigation={k:v for k,v in report['navigation'].items() if k!='route_events'},
    robots=[dict(robot=r['robot'],success=r['success'],failure=r['failure'],
                 travel_distance_m=r['travel_distance_m'],placements=r['completed_jobs']) for r in report['robots']],
    limitations='%d newly prepared 24 cm box tasks, at most five active floor boxes; original racks and five robots; session-only scene changes. '%args.count+
                'This is a reduced assisted-grasp completion demo, not completion of the original 88-prop scene, '
                'not QMIX, and not an evaluation of the image classifier.')
(root/'reports'/(name+'.json')).write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
