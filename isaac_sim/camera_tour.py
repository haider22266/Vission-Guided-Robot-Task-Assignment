"""Automatic robot/room shots, with cuts instead of flying through walls."""
import math

from navigation import clear_segment


def distributed_points(points, count=4):
    if not points:
        return []
    chosen=[points[0]]
    while len(chosen)<min(count,len(points)):
        point=max(points,key=lambda p:min(math.dist(p,q) for q in chosen))
        if point in chosen:
            break
        chosen.append(point)
    return chosen


def chase_pose(robot,height=.45,distance=1.2):
    heading=math.radians(robot.get('heading_degrees',0.))
    dx,dy=math.cos(heading),math.sin(heading)
    return ((robot['x']-distance*dx,robot['y']-distance*dy,height),
            (robot['x']+.25*dx,robot['y']+.25*dy,robot['z']+.18))


class ChaseCamera:
    """Low aisle-height third-person view, focused on a ground robot."""
    def __init__(self,stage,response,viewport,robot_id='ugv_1',height=.45,distance=1.2):
        from pxr import UsdGeom
        self.response,self.viewport=response,viewport
        self.robot_id,self.height,self.distance=robot_id,height,distance
        if not any(r['robot_id']==robot_id and r['robot_type']=='UGV' for r in response.catalog['robots']):
            raise ValueError('--follow-robot must identify a ground robot in this fleet')
        self.path='/DisasterResponse/ChaseCamera'
        self.camera=UsdGeom.Camera.Define(stage,self.path)
        self.camera.CreateFocalLengthAttr(18)
        self.camera.CreateClippingRangeAttr((.015,100.))
        self.eye=None
        self.update(0.)

    def update(self,dt):
        from isaacsim.core.utils.viewports import set_camera_view
        robot=next(r for r in self.response.catalog['robots'] if r['robot_id']==self.robot_id)
        desired,target=chase_pose(robot,self.height,self.distance)
        obstacles=self.response.camera_obstacles
        bounds=self.response.bounds
        inside=(bounds[0]+.06<desired[0]<bounds[2]-.06 and bounds[1]+.06<desired[1]<bounds[3]-.06)
        if not inside or not clear_segment(desired,desired,obstacles,.06):
            heading=math.radians(robot.get('heading_degrees',0.))
            dx,dy=math.cos(heading),math.sin(heading)
            def score(p):
                ahead=max(0,(p[0]-robot['x'])*dx+(p[1]-robot['y'])*dy)
                occluded=not clear_segment(p,(robot['x'],robot['y']),obstacles,0.)
                return math.dist(p,desired[:2])+ahead*6+occluded*10
            candidates=sorted(self.response.camera_points,key=lambda p:math.dist(p,desired[:2]))[:40]
            p=min(candidates,key=score)
            desired=(*p,self.height)
        if self.eye is not None and clear_segment(self.eye,desired,obstacles,.06):
            alpha=min(1.,dt*5)
            self.eye=tuple(a+(b-a)*alpha for a,b in zip(self.eye,desired))
        else:
            self.eye=desired
        self.viewport.camera_path=self.path
        set_camera_view(eye=self.eye,target=target,camera_prim_path=self.path,viewport_api=self.viewport)
        self.response.catalog['camera_view']={'mode':'chase','robot_id':self.robot_id,
            'eye':self.eye,'target':target,'height_m':self.height,'follow_distance_m':self.distance}


class CameraTour:
    def __init__(self, stage, response, viewport, dwell=8.):
        from pxr import UsdGeom
        self.response,self.viewport,self.dwell=response,viewport,dwell
        self.path='/DisasterResponse/TourCamera'
        camera=UsdGeom.Camera.Define(stage,self.path)
        camera.CreateFocalLengthAttr(12)
        camera.CreateClippingRangeAttr((.02,100.))
        self.points=response.camera_points
        if not self.points:
            raise RuntimeError('No clear camera viewpoints found; use --camera-view manual')
        self.areas=distributed_points(self.points)
        self.elapsed=0.
        self.shot=-1
        self.visited=set()
        self.eye=None
        self.target=None
        self.update(0.)

    def update(self,dt):
        from isaacsim.core.utils.viewports import set_camera_view
        self.elapsed+=dt
        robots=self.response.catalog['robots']
        shot=int(self.elapsed/self.dwell)%(len(robots)+len(self.areas))
        obstacles=self.response.camera_obstacles
        if shot<len(robots):
            robot=robots[shot]
            target=(robot['x'],robot['y'],max(.25,robot['z']+.1))
            focus=target[:2]
        else:
            focus=self.areas[shot-len(robots)]
            target=(*focus,.6)
        # Keep the eye on free floorplan cells with a clear line toward focus.
        candidates=sorted(self.points,key=lambda p:abs(math.dist(p,focus)-1.3))
        eye=next((p for p in candidates if math.dist(p,focus)>.3
                  and clear_segment(p,focus,obstacles,0.)),None)
        if eye is None:
            eye=min(self.points,key=lambda p:math.dist(p,focus))
        desired=(*eye,.95)
        cut=shot!=self.shot or self.eye is None or not clear_segment(self.eye,desired,obstacles,.06)
        if cut:
            self.eye,self.target=desired,target
        else:
            # Short eased motion only inside a collision-free camera segment.
            alpha=min(1.,dt*2)
            self.eye=tuple(a+(b-a)*alpha for a,b in zip(self.eye,desired))
            self.target=tuple(a+(b-a)*alpha for a,b in zip(self.target,target))
        if math.dist(self.eye,self.target)<.05:
            self.target=(self.target[0]+.2,self.target[1],self.target[2])
        self.shot=shot
        self.visited.add(shot)
        self.response.catalog['camera_tour']={'shot':shot,'shots_visited':sorted(self.visited),
                                             'shot_count':len(robots)+len(self.areas),'dwell_seconds':self.dwell}
        self.viewport.camera_path=self.path
        set_camera_view(eye=self.eye,target=self.target,camera_prim_path=self.path,viewport_api=self.viewport)


class FixedOverviewCamera:
    """Frame the room once, clipping the captured roof, without following robots."""
    def __init__(self,stage,response,viewport):
        from pxr import UsdGeom
        from isaacsim.core.utils.viewports import set_camera_view
        xmin,ymin,xmax,ymax=response.bounds
        x,y=(xmin+xmax)/2,(ymin+ymax)/2
        resolution=viewport.resolution
        aspect=resolution[0]/max(1,resolution[1])
        width=max(xmax-xmin+.4,(ymax-ymin+.4)*aspect)
        path='/DisasterResponse/OverviewCamera'
        camera=UsdGeom.Camera.Define(stage,path)
        # An elevated perspective camera with a near plane below the captured
        # roof exposes the room. The viewport's built-in perspective camera
        # rewrites its focal length, so use a dedicated camera.
        camera.CreateProjectionAttr('perspective')
        camera.CreateHorizontalApertureAttr(20.955)
        camera.CreateVerticalApertureAttr(20.955/aspect)
        camera.CreateFocalLengthAttr(20.955*5.9/width)
        camera.CreateClippingRangeAttr((4.1,100.))
        viewport.camera_path=path
        set_camera_view(eye=[x,y-.01,6.],target=[x,y,.1],camera_prim_path=path,viewport_api=viewport)
        response.catalog['camera_view']={'mode':'overview','width_m':width,'tracks_all_robots':False,
                                        'eye':[x,y-.01,6.],'target':[x,y,.1],
                                        'near_clip_m':4.1}


class GroupCamera:
    """Track the complete fleet from below the reconstructed ceiling."""
    def __init__(self,stage,response,viewport):
        from pxr import UsdGeom
        self.response,self.viewport=response,viewport
        self.path='/DisasterResponse/GroupCamera'
        self.camera=UsdGeom.Camera.Define(stage,self.path)
        self.camera.CreateProjectionAttr('orthographic')
        self.camera.CreateClippingRangeAttr((.01,100.))
        self.update(0.)

    def update(self,dt):
        from isaacsim.core.utils.viewports import set_camera_view
        robots=self.response.catalog['robots']
        xmin=min(r['x']-r['clearance_radius_m'] for r in robots)
        xmax=max(r['x']+r['clearance_radius_m'] for r in robots)
        ymin=min(r['y']-r['clearance_radius_m'] for r in robots)
        ymax=max(r['y']+r['clearance_radius_m'] for r in robots)
        x,y=(xmin+xmax)/2,(ymin+ymax)/2
        width=max(2.,xmax-xmin+.8,(ymax-ymin+.8)*16/9)
        self.camera.GetHorizontalApertureAttr().Set(width*10)
        self.camera.GetVerticalApertureAttr().Set(width*10*9/16)
        # Near-vertical view avoids perspective size differences between UAVs/UGVs.
        self.viewport.camera_path=self.path
        set_camera_view(eye=[x,y,2.05],target=[x,y,.3],camera_prim_path=self.path,viewport_api=self.viewport)
        self.response.catalog['camera_view']={'mode':'group','width_m':width,'tracks_all_robots':True}


class RearCamera:
    """Follow the fleet from behind with a slightly elevated shoulder view."""
    def __init__(self,stage,response,viewport):
        from pxr import UsdGeom
        self.response,self.viewport=response,viewport
        self.path='/DisasterResponse/RearCamera'
        self.camera=UsdGeom.Camera.Define(stage,self.path)
        self.camera.CreateClippingRangeAttr((.02,100.))
        self.eye=None
        self.timer=0.
        self.update(0.)

    def update(self,dt):
        from isaacsim.core.utils.viewports import set_camera_view
        self.timer+=dt
        if self.eye is not None and self.timer<.1:
            return
        dt=self.timer
        self.timer=0.
        robots=self.response.catalog['robots']
        x=sum(r['x'] for r in robots)/len(robots)
        y=sum(r['y'] for r in robots)/len(robots)
        heading=math.radians(robots[-1].get('heading_degrees',0.))
        forward=(math.cos(heading),math.sin(heading))
        desired=(x-2.2*forward[0]-.6*forward[1],y-2.2*forward[1]+.6*forward[0])
        points=self.response.camera_points
        obstacles=self.response.camera_obstacles
        def score(p):
            # Prefer a rear position and clear sight lines toward the fleet.
            visible=sum(clear_segment(p,(r['x'],r['y']),obstacles,0) for r in robots)
            ahead=max(0,(p[0]-x)*forward[0]+(p[1]-y)*forward[1])
            return math.dist(p,desired)+ahead*8-visible*4
        candidates=[p for p in points if (p[0]-x)*forward[0]+(p[1]-y)*forward[1]<-.15]
        nearby=sorted(candidates or points,key=lambda p:math.dist(p,desired))[:40]
        eye=min(nearby,key=score)
        desired_eye=(*eye,1.8)
        if self.eye is not None and clear_segment(self.eye,desired_eye,obstacles,.06):
            alpha=min(1,dt*2)
            self.eye=tuple(a+(b-a)*alpha for a,b in zip(self.eye,desired_eye))
        else:
            self.eye=desired_eye
        target=(x,y,.7)
        radius=max(math.dist((r['x'],r['y'],r['z']),target)+r['clearance_radius_m'] for r in robots)
        distance=math.dist(self.eye,target)
        self.camera.GetFocalLengthAttr().Set(max(3.,min(12.,5.*distance/max(.5,radius))))
        self.viewport.camera_path=self.path
        set_camera_view(eye=self.eye,target=target,camera_prim_path=self.path,viewport_api=self.viewport)
        self.response.catalog['camera_view']={'mode':'rear','eye':self.eye,'target':target,'tracks_all_robots':True}
