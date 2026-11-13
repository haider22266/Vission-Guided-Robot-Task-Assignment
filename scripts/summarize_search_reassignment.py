"""Publish measured search/reassignment regression outcomes, including failure."""
import json
from pathlib import Path

root=Path(__file__).resolve().parents[1]
report=json.loads((root/'results/cdd/bounded_reassignment_validation.json').read_text(encoding='utf-8'))
summary=dict(test='injected drop with bounded task search',
    perception_backend=report['research_pipeline']['perception_backend'],
    target=report['cleanup']['diagnostic_object_filter'],
    simulated_seconds=report['simulated_seconds'],counts=report['cleanup']['counts'],
    floor_clean=report['cleanup']['floor_clean'],
    robots=[dict(robot=r['robot'],failure=r['failure'],recoveries=r.get('recoveries',[]),
                 verified_placements=r['completed_jobs']) for r in report['robots']],
    interpretation='Negative completion result: physical floor recovery alone did not produce a successful subsequent placement. '
                   'The later multi-direction escape refinement was not loaded in this recorded run. '
                   'Do not use this diagnostic as proof of full cleanup or successful dropped-object completion.')
(root/'reports/search_reassignment_validation.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
