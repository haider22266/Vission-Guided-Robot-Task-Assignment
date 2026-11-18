"""Export the measured five-prop completion test without per-frame route logs."""
import json
from pathlib import Path

root=Path(__file__).resolve().parents[1]
source=root/'results/cdd/small_cleanup_validation.json'
report=json.loads(source.read_text(encoding='utf-8'))
summary=dict(scenario=report['scenario'],simulated_seconds=report['simulated_seconds'],
    counts=report['cleanup']['counts'],all_five_stored=report['cleanup']['floor_clean'],
    camera_unchanged=report['camera_unchanged'],perception=report['research_pipeline']['perception_backend'],
    navigation={k:v for k,v in report['navigation'].items() if k!='route_events'},
    robots=[dict(robot=r['robot'],success=r['success'],failure=r['failure'],
                 travel_distance_m=r['travel_distance_m'],placements=r['completed_jobs']) for r in report['robots']],
    limitations='Five newly prepared isolated 24 cm boxes, original racks and five robots; session-only scene changes. '
                'This is a reduced assisted-grasp completion demo, not completion of the original 88-prop scene, '
                'not QMIX, and not an evaluation of the image classifier.')
(root/'reports/small_cleanup_validation.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
