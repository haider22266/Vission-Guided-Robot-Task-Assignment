"""Area-level tasks and feasibility masks for a future QMIX policy.

Visual scores are whole-image class probabilities, not victim locations or
route-safety certificates. Costs are execution seconds; energy is battery %.
"""
from dataclasses import asdict, dataclass, field
import math
from typing import Mapping
import json
from pathlib import Path
import re


TASK_TYPES = ('area_search', 'hazard_inspection', 'supply_delivery')
RISK_CLASSES = ('Damaged Infrastructure', 'Fire Disaster', 'Human Damage',
                'Land Disaster', 'Non-Damage', 'Water Disaster')
HAZARDS = {
    'Damaged Infrastructure': 'infrastructure_inspection',
    'Fire Disaster': 'fire_inspection',
    'Land Disaster': 'terrain_inspection',
    'Water Disaster': 'flood_monitoring',
}
CAPABILITY_ALIASES = {'camera': 'area_search', 'inspection': 'ground_inspection',
                      'delivery': 'supply_delivery'}


def supports_capability(robot, required):
    return required in robot['capabilities'] or CAPABILITY_ALIASES.get(required) in robot['capabilities']


def visual_scores(scores: Mapping[str, float]) -> dict[str, float]:
    """Accept display labels or dataset labels, preserving all six scores."""
    result = dict.fromkeys(RISK_CLASSES, 0.0)
    for label, value in scores.items():
        label = label.replace('_', ' ').replace('Non Damage', 'Non-Damage')
        if label not in result:
            raise ValueError(f'Unknown visual class: {label}')
        if not math.isfinite(value) or not 0 <= value <= 1:
            raise ValueError('Visual scores must be finite probabilities in [0, 1]')
        result[label] = float(value)
    return result


@dataclass
class Task:
    task_id: str
    task_type: str
    location: tuple[float, float]
    priority: float
    required_capability: str
    estimated_cost: float
    visual_risk_scores: dict[str, float] = field(default_factory=dict)
    status: str = 'pending'
    evidence: str = 'area-level observation; no person localized'
    assigned_robot: str | None = None
    location_policy: str = 'relocate'
    blocked_reason: str | None = None

    def __post_init__(self):
        if self.task_type not in TASK_TYPES:
            raise ValueError('Unsupported task type')
        if (len(self.location) != 2 or not all(math.isfinite(v) for v in self.location)
                or not math.isfinite(self.priority) or not 0 <= self.priority <= 1
                or not math.isfinite(self.estimated_cost) or self.estimated_cost <= 0
                or not self.required_capability):
            raise ValueError('Invalid task location, priority, capability, or cost')
        if self.status not in ('pending', 'assigned', 'in_progress', 'completed', 'cancelled'):
            raise ValueError('Invalid task status')
        if self.location_policy not in ('fixed', 'relocate'):
            raise ValueError('Invalid location policy')
        self.visual_risk_scores = visual_scores(self.visual_risk_scores)

    def to_dict(self):
        return asdict(self)


def load_tasks(path):
    """Load the user's id/type/x/y task format without moving its coordinates."""
    rows=json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(rows,list):
        raise ValueError('Task file must contain a JSON list')
    tasks=[]
    seen=set()
    for row in rows:
        task_id=str(row['id'])
        if not re.fullmatch(r'[A-Za-z0-9_]+',task_id):
            raise ValueError('Task IDs must contain only letters, digits and underscores')
        if task_id in seen:
            raise ValueError(f'Duplicate task ID: {task_id}')
        seen.add(task_id)
        if row.get('status','pending')!='pending' or row.get('assigned_robot') is not None:
            raise ValueError('Input scenario tasks must start pending and unassigned')
        tasks.append(Task(task_id,row['type'],(row['x'],row['y']),row['priority'],
                          row['required_capability'],row.get('estimated_cost',15.),
                          row.get('visual_risk_scores',{}),row.get('status','pending'),
                          evidence='Explicit scenario task; no classifier evidence supplied',
                          assigned_robot=row.get('assigned_robot'),location_policy='fixed'))
    return tasks


def area_tasks(area_id, location, scores, threshold=0.6, execution_seconds=15.0):
    """Create a search and any indicated inspections; never infer delivery demand."""
    scores = visual_scores(scores)
    if not 0 <= threshold <= 1:
        raise ValueError('Invalid confidence threshold')
    tasks = [Task(f'{area_id}_search', 'area_search', location,
                  max(0.3, scores['Human Damage']), 'area_search', execution_seconds, scores)]
    for label, capability in HAZARDS.items():
        if scores[label] >= threshold:
            tasks.append(Task(f'{area_id}_{capability}', 'hazard_inspection', location,
                              max(0.5, scores[label]), capability, execution_seconds, scores))
    return tasks


def assignment_features(robot, task, route_safe=None, battery_reserve=10.0):
    """Expose distance, priority and energy together with a hard feasibility gate.

    route_safe is an independently established per-robot/per-task decision.
    Unknown routes block delivery. False blocks any task. This function does not
    plan paths; Euclidean distance is only an allocation estimate.
    """
    distance = math.hypot(robot['x'] - task.location[0], robot['y'] - task.location[1])
    energy = (distance * robot['energy_per_meter']
              + task.estimated_cost * robot['execution_energy_per_second'])
    compatible_type = (robot['robot_type'] in ('UAV', 'UGV') if task.task_type == 'area_search'
                       else robot['robot_type'] == 'UGV')
    feasible = (task.status == 'pending' and compatible_type
                and supports_capability(robot,task.required_capability)
                and robot['available'] and robot['healthy']
                and robot['communication_available'] and robot['current_task'] is None
                and robot['battery'] >= energy + battery_reserve
                and route_safe is not False
                and (task.task_type != 'supply_delivery' or route_safe is True))
    return {'feasible': bool(feasible), 'priority': task.priority,
            'distance_m': distance, 'estimated_energy': energy,
            'estimated_seconds': distance / robot['speed_mps'] + task.estimated_cost,
            'route_safe': route_safe}


def action_mask(robot, tasks, route_safety=None, battery_reserve=10.0):
    """QMIX action order: idle, then tasks in supplied order. Idle is always valid."""
    route_safety = route_safety or {}
    return [True] + [assignment_features(robot, task, route_safety.get(task.task_id),
                                        battery_reserve)['feasible'] for task in tasks]
