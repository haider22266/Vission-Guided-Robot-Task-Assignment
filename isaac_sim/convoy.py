"""A shared collision-checked centreline, with spaced fleet motion."""
import math

from navigation import clear_segment, plan
from nurec_navigation import free_points


def point_on_path(points,distance):
    for a,b in zip(points,points[1:]):
        length=math.dist(a,b)
        if length and distance<=length:
            ratio=max(0,distance)/length
            return (a[0]+(b[0]-a[0])*ratio,a[1]+(b[1]-a[1])*ratio),math.degrees(math.atan2(b[1]-a[1],b[0]-a[0]))
        distance-=length
    a,b=points[-2:]
    return points[-1],math.degrees(math.atan2(b[1]-a[1],b[0]-a[0]))


def path_slice(points,start,end):
    result=[point_on_path(points,start)[0]]
    distance=0.
    for a,b in zip(points,points[1:]):
        distance+=math.dist(a,b)
        if start<distance<end:
            result.append(b)
    result.append(point_on_path(points,end)[0])
    return result


class Convoy:
    def __init__(self,response):
        self.response=response
        self.progress=0.
        self.direction=1
        self.speed=.18
        self.order=list(range(len(response.explorers)))
        self.visits={}
        self.legs=1
        self.obstacles=list(set(rect for obstacles in response.maps for rect in obstacles))
        radius=max(e.radius for e in response.explorers)
        # Ground robots need spacing for turns; aerial units keep distinct altitudes.
        self.spacing=max(.45,1.5*sum(e.radius for e in response.explorers[-2:]))
        self.span=self.spacing*(len(response.explorers)-1)
        points=free_points(response.bounds,self.obstacles,radius,cell=.25)
        self.coverage_points=points
        self.radius=radius
        if not points:
            raise RuntimeError('No common fleet corridor. Try --robot-scale .15')
        # Try distributed endpoints, keeping the longest reachable shared path.
        from camera_tour import distributed_points
        anchors=distributed_points(points,8)
        best=[]
        self.length=0.
        for a in anchors:
            for b in sorted(points,key=lambda p:math.dist(a,p),reverse=True)[::max(1,len(points)//12)]:
                path=plan(a,b,self.obstacles,radius,response.bounds,.2)
                if path:
                    candidate=[a]+path
                    length=sum(math.dist(x,y) for x,y in zip(candidate,candidate[1:]))
                    if length>self.length:
                        best,self.length=candidate,length
        if self.length<self.span+.25:
            raise RuntimeError('Common route is too short for a spaced convoy. Run with --robot-scale .15 or .1')
        self.path=best
        self.limit=self.length-self.span
        if not self.update_positions(0.,initial=True):
            raise RuntimeError('Convoy cannot fit around this route. Try --robot-scale .1')
        self.add_highlights()
        for robot in response.catalog['robots']:
            robot.update(current_task=None,available=True,navigation_status='shared_path_convoy')
        response.catalog['navigation']['fleet_mode']='convoy'
        response.catalog['navigation']['shared_path']=self.path
        response.catalog['navigation']['convoy_spacing_m']=self.spacing
        response.catalog['navigation']['task_allocation']='Paused while fleet follows the shared exploration route'
        response.catalog['navigation']['exploration']='Visit reachable common-map cells; extend shared route at each leg'

    def add_highlights(self):
        from pxr import UsdGeom, Gf
        colors=[(.1,1.,1.),(1.,.65,.1),(.3,1.,.2),(1.,.2,.7)]
        for robot,color in zip(self.response.catalog['robots'],colors):
            ring=UsdGeom.BasisCurves.Define(self.response.stage,'/DisasterResponse/Robots/'+robot['robot_id']+'/visibility_ring')
            ring.CreateTypeAttr('linear')
            ring.CreateWrapAttr('periodic')
            ring.CreateCurveVertexCountsAttr([32])
            ring.CreatePointsAttr([Gf.Vec3f(.09*math.cos(i*math.tau/32),.09*math.sin(i*math.tau/32),.06) for i in range(32)])
            ring.CreateWidthsAttr([.012])
            ring.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            ring.GetPrim().SetCustomDataByKey('annotation','Fleet visibility ring; not part of robot hardware')

    def formation(self,progress,path=None,order=None):
        path=self.path if path is None else path
        order=getattr(self,'order',list(range(len(self.response.explorers)))) if order is None else order
        positions=[]
        for i,e in enumerate(self.response.explorers):
            xy,heading=point_on_path(path,progress+order.index(i)*self.spacing)
            positions.append(((*xy,e.position[2]),heading+(180 if self.direction<0 else 0)))
        return positions

    def separated(self,positions):
        # Hold the whole formation if a bend would violate vertical/horizontal clearance.
        for i,(a,_) in enumerate(positions):
            e=self.response.explorers[i]
            for j in range(i):
                b=positions[j][0]
                peer=self.response.explorers[j]
                overlap=(a[2]+e.vertical_offsets[0]<=b[2]+peer.vertical_offsets[1]
                         and b[2]+peer.vertical_offsets[0]<=a[2]+e.vertical_offsets[1])
                if overlap and math.dist(a[:2],b[:2])<e.radius+peer.radius:
                    return False
        return True

    def update_positions(self,progress,initial=False):
        positions=self.formation(progress)
        if not self.separated(positions):
            return False
        if not initial and any(not clear_segment(e.position,p,self.obstacles,e.radius)
                               for e,(p,_) in zip(self.response.explorers,positions)):
            return False
        for robot,e,(translation,yaw),(position,heading) in zip(self.response.catalog['robots'],self.response.explorers,self.response.ops,positions):
            e.position=position
            translation.Set(self.response.Gf.Vec3d(*position))
            yaw.Set(heading)
            robot.update(x=position[0],y=position[1],z=position[2],heading_degrees=heading,
                         velocity=self.speed,navigation_status='shared_path_convoy')
        return True

    def extend_route(self):
        """Preserve the occupied centreline, then extend from either end."""
        prefix=path_slice(self.path,self.progress,self.progress+self.span)
        options=[]
        for reverse in (False,True):
            occupied=list(reversed(prefix)) if reverse else prefix
            order=list(reversed(self.order)) if reverse else self.order
            front=occupied[-1]
            choices=sorted(self.coverage_points,key=lambda p:
                self.visits.get((round(p[0]/.5),round(p[1]/.5)),0)*5-math.dist(p,front))
            for goal in choices[::max(1,len(choices)//35)]:
                if math.dist(front,goal)<.4:
                    continue
                extension=plan(front,goal,self.obstacles,self.radius,self.response.bounds,.2)
                if not extension:
                    continue
                path=occupied+extension
                length=sum(math.dist(a,b) for a,b in zip(path,path[1:]))
                limit=length-self.span
                # Reject folds which would make vehicles occupy the same space.
                valid=all(self.separated(self.formation(min(limit,i*.12),path,order))
                          for i in range(math.ceil(limit/.12)+1))
                if not valid:
                    continue
                visited=self.visits.get((round(goal[0]/.5),round(goal[1]/.5)),0)
                options.append((visited*5-math.dist(front,goal),path,order,length))
                break
        if not options:
            return False
        _,self.path,self.order,self.length=min(options,key=lambda o:o[0])
        self.limit=self.length-self.span
        self.progress=0.
        self.direction=1
        self.legs+=1
        navigation=self.response.catalog['navigation']
        navigation.update(shared_path=self.path,coverage_legs=self.legs,visited_cells=len(self.visits))
        return True

    def step(self,dt):
        for e in self.response.explorers:
            key=(round(e.position[0]/.5),round(e.position[1]/.5))
            self.visits[key]=self.visits.get(key,0)+dt
        candidate=self.progress+self.direction*self.speed*dt
        if candidate>self.limit:
            candidate=self.limit
        elif candidate<0:
            candidate=0
            self.direction=1
        # Each robot follows the exact same XY polyline at its own altitude.
        if self.update_positions(candidate):
            self.progress=candidate
            if self.direction>0 and self.progress>=self.limit-1e-8:
                if not self.extend_route():
                    self.direction=-1
        else:
            if not self.extend_route():
                self.direction*=-1
