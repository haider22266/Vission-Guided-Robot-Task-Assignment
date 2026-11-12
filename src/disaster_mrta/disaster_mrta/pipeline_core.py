"""ROS-independent state, feasibility, assignment leases and failure recovery."""
from dataclasses import dataclass, field, asdict
import math
import uuid


@dataclass
class AreaTask:
    task_id: str
    task_type: str
    x: float
    y: float
    priority: float
    required_capability: str
    estimated_cost: float = 3.
    route_safe: bool = False
    status: str = 'pending'
    assigned_robot: str | None = None
    assignment_id: str | None = None
    generation: int = 0
    hazard_risk: float = 0.
    rescue_signal: float = 0.
    confidence: float = 0.

    def __post_init__(self):
        if self.task_type not in ('area_search','hazard_inspection','supply_delivery'):
            raise ValueError('Unsupported task type')
        if (not all(math.isfinite(v) for v in (self.x,self.y,self.priority,self.estimated_cost))
                or not 0<=self.priority<=1 or self.estimated_cost<=0 or not self.required_capability):
            raise ValueError('Invalid task coordinates, priority, capability or cost')


@dataclass
class RobotState:
    robot_id: str
    x: float = 0.
    y: float = 0.
    battery: float = 0.
    capabilities: list = field(default_factory=list)
    healthy: bool = False
    communication_available: bool = False
    online: bool = False
    current_task: str | None = None
    sequence: int = -1
    boot_id: str = ''
    retired_boot_ids: list = field(default_factory=list)


def feasible(robot, task, min_battery=20., max_distance=50.):
    aliases={'camera':'area_search','inspection':'ground_inspection','delivery':'supply_delivery'}
    capable=task.required_capability in robot.capabilities or aliases.get(task.required_capability) in robot.capabilities
    return (robot.online and robot.healthy and robot.communication_available and robot.current_task is None
            and robot.battery>=min_battery and task.status=='pending' and capable
            and math.hypot(robot.x-task.x,robot.y-task.y)<=max_distance
            and (task.task_type!='supply_delivery' or task.route_safe))


class FleetState:
    def __init__(self, robot_ids, tasks, min_battery=20., max_distance=50.):
        self.robots={rid:RobotState(rid) for rid in robot_ids}
        self.tasks={str(t['task_id']):AreaTask(**dict(t,task_id=str(t['task_id']))) for t in tasks}
        if len(self.tasks)!=len(tasks):
            raise ValueError('Duplicate task IDs')
        self.min_battery,self.max_distance=min_battery,max_distance
        self.failures=[]

    def status(self, data):
        robot=self.robots[data['robot_id']]
        boot_id=str(data.get('boot_id','default'))
        sequence=int(data['sequence'])
        x,y,battery=float(data['x']),float(data['y']),float(data['battery'])
        if not all(math.isfinite(v) for v in (x,y,battery)) or not 0<=battery<=100 or sequence<0:
            raise ValueError('Nonfinite robot state, invalid battery or sequence')
        if not isinstance(data['healthy'],bool) or not isinstance(data['communication_available'],bool):
            raise ValueError('Health and communication flags must be JSON booleans')
        if not isinstance(data['capabilities'],list) or not all(isinstance(c,str) for c in data['capabilities']):
            raise ValueError('Capabilities must be a string list')
        if boot_id in robot.retired_boot_ids:
            return False
        if boot_id!=robot.boot_id:
            if robot.boot_id:
                robot.retired_boot_ids.append(robot.boot_id)
            robot.boot_id=boot_id
            robot.sequence=-1
        if sequence<=robot.sequence:
            return False
        robot.x,robot.y,robot.battery=x,y,battery
        robot.capabilities=list(data['capabilities'])
        robot.healthy=bool(data['healthy'])
        robot.communication_available=bool(data['communication_available'])
        robot.online=robot.healthy and robot.communication_available
        robot.sequence=sequence
        return True

    def mask(self):
        tasks=list(self.tasks.values())
        return [[True]+[feasible(r,t,self.min_battery,self.max_distance) for t in tasks] for r in self.robots.values()]

    def commit(self, robot_id, task_id):
        r,t=self.robots[robot_id],self.tasks[str(task_id)]
        if not feasible(r,t,self.min_battery,self.max_distance):
            return None
        token=uuid.uuid4().hex
        t.generation+=1
        t.status,t.assigned_robot,t.assignment_id='assigned',robot_id,token
        r.current_task=t.task_id
        return dict(asdict(t),robot_id=robot_id)

    def release_robot(self,robot_id,reason,now):
        r=self.robots[robot_id]
        r.online=False
        cancel=None
        if r.current_task is not None:
            cancel=self.cancel_task(r.current_task,reason)
        self.failures.append({'robot_id':robot_id,'reason':reason,'time':now})
        return cancel

    def cancel_task(self,task_id,reason):
        t=self.tasks[str(task_id)]
        if t.status not in ('assigned','in_progress'):
            return None
        cancel={'robot_id':t.assigned_robot,'task_id':t.task_id,'assignment_id':t.assignment_id,
                'action':'cancel','reason':reason}
        self.robots[t.assigned_robot].current_task=None
        t.status='pending'
        t.assigned_robot=t.assignment_id=None
        return cancel

    def feedback(self, data):
        t=self.tasks.get(str(data['task_id']))
        if t is None or t.assignment_id!=data.get('assignment_id') or t.assigned_robot!=data.get('robot_id'):
            return False
        state=data['status']
        if state=='in_progress' and t.status=='assigned':
            t.status=state
        elif state=='completed' and t.status=='in_progress':
            t.status=state
            self.robots[t.assigned_robot].current_task=None
        elif state=='failed' and t.status in ('assigned','in_progress'):
            self.robots[t.assigned_robot].current_task=None
            t.status='pending'
            t.assigned_robot=t.assignment_id=None
        else:
            return False
        return True

    def vision(self,data):
        t=self.tasks.get(str(data['area_id']))
        if t is None:
            return
        confidence,hazard,rescue=[float(data[k]) for k in ('confidence','hazard_risk','rescue_signal')]
        if not all(math.isfinite(x) and 0<=x<=1 for x in (confidence,hazard,rescue)):
            raise ValueError('Vision scores must be probabilities')
        t.confidence,t.hazard_risk,t.rescue_signal=confidence,hazard,rescue
        signal=rescue if t.task_type=='area_search' else hazard if t.task_type=='hazard_inspection' else 0.
        t.priority=max(t.priority,signal)


class FailureTracker:
    """Call once per 0.1s wall-clock interval; three missed status sequences fail."""
    def __init__(self,robot_ids,miss_limit=3):
        self.last={rid:-1 for rid in robot_ids}
        self.missed={rid:0 for rid in robot_ids}
        self.failed=set()
        self.limit=miss_limit

    def tick(self,robots):
        failures=[]
        for rid,r in robots.items():
            if r.sequence<0:
                continue
            if r.sequence!=self.last[rid]:
                self.last[rid]=r.sequence
                self.missed[rid]=0
                if r.healthy and r.communication_available:
                    self.failed.discard(rid)
            else:
                self.missed[rid]+=1
            bad=not r.healthy or not r.communication_available or self.missed[rid]>=self.limit
            if bad and rid not in self.failed:
                self.failed.add(rid)
                failures.append(rid)
        return failures


def baseline_actions(fleet):
    """Explicit integration baseline; this is not QMIX."""
    selected=set()
    result=[]
    tasks=list(fleet.tasks.values())
    for robot in fleet.robots.values():
        candidates=[t for t in tasks if t.task_id not in selected and feasible(robot,t,fleet.min_battery,fleet.max_distance)]
        if not candidates:
            result.append(0)
            continue
        task=max(candidates,key=lambda t:t.priority-.01*math.hypot(robot.x-t.x,robot.y-t.y))
        result.append(tasks.index(task)+1)
        selected.add(task.task_id)
    return result


def observation(fleet,max_tasks):
    """Stable agent-major feature order; checkpoint metadata must match this layout."""
    tasks=list(fleet.tasks.values())
    if len(tasks)>max_tasks:
        raise ValueError('Task count exceeds checkpoint capacity')
    result=[]
    for r in fleet.robots.values():
        row=[r.x/50,r.y/50,r.battery/100,float(r.online and r.healthy and r.communication_available),float(r.current_task is not None)]
        for t in tasks:
            row.extend([(t.x-r.x)/50,(t.y-r.y)/50,t.priority,t.hazard_risk,t.rescue_signal,t.confidence,
                        float(t.status=='pending'),float(feasible(r,t,fleet.min_battery,fleet.max_distance))])
        row.extend([0.]*(8*(max_tasks-len(tasks))))
        result.append(row)
    return result
