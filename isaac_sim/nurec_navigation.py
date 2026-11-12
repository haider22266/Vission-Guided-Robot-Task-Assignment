"""Mesh-derived navigation and symbolic disaster tasks for NuRec scenes."""
import json
import math

from navigation import Explorer, clear_segment, footprint, obstacles_for, plan


def raster_rectangles(triangles, bounds, low, high, cell=.2):
    """Conservative XY raster of triangle bounds intersecting a vertical slab.

    Ignore floor surfaces below 10cm. Unknown outside the mesh bounds is not
    traversable. This map represents authored collision surfaces, not splats.
    """
    xmin,ymin,xmax,ymax=bounds
    nx,ny=math.ceil((xmax-xmin)/cell),math.ceil((ymax-ymin)/cell)
    blocked=set()
    for a,b in triangles:
        if b[2] < max(.1,low) or a[2] > high:
            continue
        x0=max(0,math.floor((a[0]-xmin)/cell))
        x1=min(nx-1,math.floor((b[0]-xmin)/cell))
        y0=max(0,math.floor((a[1]-ymin)/cell))
        y1=min(ny-1,math.floor((b[1]-ymin)/cell))
        blocked.update((x,y) for x in range(x0,x1+1) for y in range(y0,y1+1))
    # Merge horizontal runs, then identical runs in adjacent rows.
    active={}
    rectangles=[]
    for y in range(ny+1):
        runs=[]
        x=0
        while y<ny and x<nx:
            if (x,y) not in blocked:
                x+=1
                continue
            start=x
            while x<nx and (x,y) in blocked:
                x+=1
            runs.append((start,x))
        for run in list(active):
            if run not in runs:
                start_y=active.pop(run)
                rectangles.append((xmin+run[0]*cell,ymin+start_y*cell,
                                   xmin+run[1]*cell,ymin+y*cell))
        for run in runs:
            active.setdefault(run,y)
    return rectangles


def collision_triangles(stage):
    import numpy as np
    from pxr import Usd, UsdGeom, UsdPhysics
    triangle_bounds=[]
    world_min=None
    world_max=None
    for prim in stage.Traverse():
        path=str(prim.GetPath())
        if path.startswith(('/DisasterResponse','/World/NovaCarterNav')) or 'GroundPlane' in path:
            continue
        if not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
            continue
        mesh=UsdGeom.Mesh(prim)
        points=np.asarray(mesh.GetPointsAttr().Get(),dtype=float)
        if len(points)==0:
            continue
        matrix=np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        points=(np.c_[points,np.ones(len(points))] @ matrix)[:,:3]
        indices=list(mesh.GetFaceVertexIndicesAttr().Get())
        counts=list(mesh.GetFaceVertexCountsAttr().Get())
        offset=0
        for count in counts:
            face=indices[offset:offset+count]
            offset+=count
            for j in range(1,count-1):
                vertices=points[[face[0],face[j],face[j+1]]]
                triangle_bounds.append((tuple(vertices.min(axis=0)),tuple(vertices.max(axis=0))))
        lo,hi=points.min(axis=0),points.max(axis=0)
        world_min=lo if world_min is None else np.minimum(world_min,lo)
        world_max=hi if world_max is None else np.maximum(world_max,hi)
    if world_min is None:
        raise RuntimeError('NuRec navigation requires an authored collision mesh; none was found')
    bounds=(float(world_min[0]),float(world_min[1]),float(world_max[0]),float(world_max[1]))
    return triangle_bounds,bounds


def free_points(bounds, obstacles, radius, cell=.3):
    xmin,ymin,xmax,ymax=bounds
    result=[]
    x=xmin+radius+.05
    while x<xmax-radius:
        y=ymin+radius+.05
        while y<ymax-radius:
            if clear_segment((x,y),(x,y),obstacles,radius):
                result.append((x,y))
            y+=cell
        x+=cell
    return result


class NuRecResponse:
    """Kinematic A* motion; task execution is an explicitly symbolic timer."""
    def __init__(self, stage, catalog, explore_first=15.):
        from pxr import Gf, UsdGeom
        from disaster_mrta.tasks import Task, supports_capability
        self.stage,self.catalog=stage,catalog
        self.reserve=catalog.get('battery_reserve',10.)
        self.elapsed=0.
        self.explore_first=explore_first
        self.visits=[{} for _ in catalog['robots']]
        self.Gf=Gf
        self.tasks=[Task(**task) for task in catalog['tasks']]
        triangles,self.bounds=collision_triangles(stage)
        self.camera_obstacles=raster_rectangles(triangles,self.bounds,.25,1.6)
        self.camera_points=free_points(self.bounds,self.camera_obstacles,.08)
        self.explorers=[]
        self.maps=[]
        self.ops=[]
        self.candidates=[]
        self.remaining={}
        self.energy={}
        for robot in catalog['robots']:
            path='/DisasterResponse/Robots/'+robot['robot_id']
            position=(robot['x'],robot['y'],robot['z'])
            radius,vertical=footprint(stage,path,position)
            obstacles=raster_rectangles(triangles,self.bounds,position[2]+vertical[0],position[2]+vertical[1])
            points=free_points(self.bounds,obstacles,radius)
            if not points:
                raise RuntimeError(f'No clearance for {robot["robot_id"]}. Try --robot-scale .15')
            occupied=obstacles_for(Explorer([position,position],.2,radius,vertical),[],self.explorers)
            points.sort(key=lambda p:math.dist(p,position[:2]))
            spawn=next((p for p in points if clear_segment(p,p,occupied,radius)),None)
            if spawn is None:
                raise RuntimeError('No separated fleet spawn found; reduce --robot-scale')
            position=(*spawn,position[2])
            explorer=Explorer([position,position],.2,radius,vertical,self.bounds,.2)
            transform=UsdGeom.Xformable(stage.GetPrimAtPath(path))
            translation=next(op for op in transform.GetOrderedXformOps() if op.GetOpType()==UsdGeom.XformOp.TypeTranslate)
            translation.Set(Gf.Vec3d(*position))
            yaw=transform.AddRotateZOp()
            self.ops.append((translation,yaw))
            self.explorers.append(explorer)
            self.maps.append(obstacles)
            self.candidates.append(points)
            robot.update(x=position[0],y=position[1],z=position[2],clearance_radius_m=radius,
                         navigation_status='ready',control_mode='kinematic mesh A*; symbolic task execution')
        # Move synthetic task areas onto reachable floor points for their capable robot.
        for task in self.tasks:
            eligible=[i for i,r in enumerate(catalog['robots']) if supports_capability(r,task.required_capability)
                      and (task.task_type=='area_search' or r['robot_type']=='UGV')]
            if task.location_policy=='fixed':
                reachable=any(plan(self.explorers[i].position[:2],task.location,self.maps[i],
                              self.explorers[i].radius,self.bounds,.2) for i in eligible)
                if not reachable:
                    task.blocked_reason='No capable robot has a mesh-map route to the requested fixed coordinates'
                    print(f'Task {task.task_id} stays pending at {task.location}: {task.blocked_reason}',flush=True)
                continue
            i=eligible[0]
            explorer=self.explorers[i]
            choices=sorted(self.candidates[i],key=lambda p:math.dist(p,task.location))
            location=None
            for p in choices:
                if plan(explorer.position[:2],p,self.maps[i],explorer.radius,self.bounds,.2):
                    location=p
                    break
            if location is None:
                raise RuntimeError(f'No reachable task area for {task.task_id}')
            task.location=location
            prim=stage.GetPrimAtPath('/DisasterResponse/Tasks/task_'+task.task_id)
            prim.SetCustomDataByKey('task_json',json.dumps(task.to_dict()))
            marker=stage.GetPrimAtPath(str(prim.GetPath())+'/area')
            if marker:
                UsdGeom.Xformable(marker).GetOrderedXformOps()[0].Set(Gf.Vec3d(*location,.02))
        catalog['navigation']={'mode':'mesh-derived 2D A* at fixed altitudes','bounds':self.bounds,
                               'initial_exploration_seconds':explore_first,
                               'raster_resolution_m':.2,'task_execution':'symbolic timers; no physical manipulation',
                               'triangle_count':len(triangles)}
        catalog['assumptions']='Synthetic whole-image risk scores; Human Damage is an area clue. '
        catalog['assumptions']+='Kinematic navigation uses a static, conservative collision-mesh map, not onboard perception. '
        catalog['assumptions']+='No flight/drivetrain/lift controllers, trained classifier or QMIX policy. Tasks execute symbolically.'
        catalog.pop('initial_action_masks',None)
        self.sync()
        print('NuRec navigation ready: '+json.dumps(catalog['navigation']),flush=True)

    def sync(self):
        from disaster_mrta.tasks import action_mask, assignment_features
        self.catalog['tasks']=[t.to_dict() for t in self.tasks]
        routes={}
        for robot,explorer,obstacles in zip(self.catalog['robots'],self.explorers,self.maps):
            routes[robot['robot_id']]={}
            for task in self.tasks:
                feasible=assignment_features(robot,task,True,self.reserve)['feasible']
                routes[robot['robot_id']][task.task_id]=bool(feasible and plan(
                    explorer.position[:2],task.location,obstacles,explorer.radius,self.bounds,.2))
        self.catalog['route_safety_by_robot']=routes
        masks={r['robot_id']:action_mask(r,self.tasks,routes[r['robot_id']],self.reserve) for r in self.catalog['robots']}
        self.catalog.setdefault('initial_action_masks',masks)
        self.catalog['action_masks']=masks
        self.catalog['assignment_features']={r['robot_id']:[assignment_features(r,t,routes[r['robot_id']][t.task_id],self.reserve) for t in self.tasks]
                                             for r in self.catalog['robots']}
        self.stage.GetPrimAtPath('/DisasterResponse').SetCustomDataByKey('task_catalog_json',json.dumps(self.catalog))
        for task in self.tasks:
            self.stage.GetPrimAtPath('/DisasterResponse/Tasks/task_'+task.task_id).SetCustomDataByKey('task_json',json.dumps(task.to_dict()))

    def step(self,dt):
        from disaster_mrta.tasks import assignment_features
        self.elapsed+=dt
        for i,(robot,explorer) in enumerate(zip(self.catalog['robots'],self.explorers)):
            visits=self.visits[i]
            cell=(round(explorer.position[0]/.6),round(explorer.position[1]/.6))
            visits[cell]=visits.get(cell,0)+dt
            obstacles=self.maps[i]+obstacles_for(explorer,[],self.explorers)
            task=next((t for t in self.tasks if t.task_id==robot['current_task']),None)
            if task and task.status=='in_progress':
                self.remaining[task.task_id]-=dt
                if self.remaining[task.task_id]<=0:
                    task.status='completed'
                    robot.update(battery=max(0,robot['battery']-self.energy.pop(task.task_id)),current_task=None,available=True)
                continue
            if task is None and self.elapsed>=self.explore_first:
                offers=[]
                for t in self.tasks:
                    f=assignment_features(robot,t,route_safe=True,battery_reserve=self.reserve)
                    if f['feasible']:
                        offers.append((t.priority-f['distance_m']*.02,t,f))
                offers.sort(key=lambda offer:offer[0],reverse=True)
                for _,candidate,features in offers:
                    path=plan(explorer.position[:2],candidate.location,obstacles,explorer.radius,self.bounds,.2)
                    if not path:
                        continue
                    travel=0.
                    start=explorer.position[:2]
                    for point in path:
                        travel+=math.dist(start,point)
                        start=point
                    energy=travel*robot['energy_per_meter']+candidate.estimated_cost*robot['execution_energy_per_second']
                    if robot['battery']<energy+self.reserve:
                        continue
                    task=candidate
                    task.status='assigned'
                    task.assigned_robot=robot['robot_id']
                    task.blocked_reason=None
                    robot.update(current_task=task.task_id,available=False)
                    self.energy[task.task_id]=energy
                    explorer.route=[explorer.position,(*task.location,explorer.position[2])]
                    explorer.goal=1
                    explorer.path=path
                    break
            if task and math.dist(explorer.position[:2],task.location)<.06:
                task.status='in_progress'
                self.remaining[task.task_id]=task.estimated_cost
                explorer.path=[]
                robot['navigation_status']='executing_symbolic_task'
                continue
            if task is None and (not explorer.path or math.dist(explorer.position[:2],explorer.route[1][:2])<.1):
                # Prefer unvisited cells, then wider exploration distances.
                choices=sorted(self.candidates[i], key=lambda p:
                    visits.get((round(p[0]/.6),round(p[1]/.6)),0)*10-math.dist(p,explorer.position[:2]))
                for p in choices[::3]:
                    path=plan(explorer.position[:2],p,obstacles,explorer.radius,self.bounds,.2)
                    if path:
                        explorer.route=[explorer.position,(*p,explorer.position[2])]
                        explorer.goal=1
                        explorer.path=path
                        break
            old=explorer.position
            position,heading=explorer.step(dt,obstacles)
            translation,yaw=self.ops[i]
            translation.Set(self.Gf.Vec3d(*position))
            yaw.Set(heading)
            robot.update(x=position[0],y=position[1],z=position[2],velocity=math.dist(old,position)/dt if dt else 0,
                         navigation_status=explorer.state)
