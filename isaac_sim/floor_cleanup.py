"""Shared floor inventory and collision-free rack reservations (no Isaac imports)."""
import uuid
import math


class TrafficDeferred(RuntimeError):
    """A transient route conflict; not evidence of failed manipulation."""


def floor_groups(prims):
    """Group original compound props; never treat tape/tool parts as new jobs."""
    by_name={p.GetName():p for p in prims}
    groups=[]
    used=set()
    for name,prim in sorted(by_name.items()):
        companions=[]
        kind=None
        if name.startswith('Fallen_Carton_'):
            kind='carton';companions=[name.replace('Fallen_Carton_','Carton_Tape_')]
        elif name.startswith(('Rear_Left_Carton_','Rear_Right_Carton_')):
            kind='carton';companions=[name.replace('_Carton_','_Tape_')]
        elif name.startswith('Tool_') and '_Handle_' in name:
            kind='tool'
            parts={'Hammer':['Head'],'Screwdriver':['Shaft'],'Wrench':['JawA','JawB']}
            tool=name.split('_')[1]
            if tool in parts:companions=[name.replace('_Handle_','_'+part+'_') for part in parts[tool]]
            else:kind=None
        elif name.startswith(('Debris_','Rear_Left_Debris_','Rear_Right_Debris_')):kind='debris'
        elif name.startswith(('Fallen_Rack_Beam_','Rear_Left_Beam_','Rear_Right_Beam_')):kind='beam'
        if kind:
            group=[prim]+[by_name[n] for n in companions if n in by_name]
            paths=[str(p.GetPath()) for p in group]
            if used.intersection(paths):raise ValueError('Duplicate floor object component')
            used.update(paths)
            groups.append({'id':paths[0],'kind':kind,'prims':group,'paths':paths})
    unknown=[str(p.GetPath()) for name,p in by_name.items()
             if name.startswith(('Debris_','Fallen_','Rear_','Tool_','Carton_Tape_')) and str(p.GetPath()) not in used]
    return groups,unknown


def overlaps(a,b,gap=.025):
    """World XY rectangles, padded to keep carried/stored objects separate."""
    return a[0]<b[2]+gap and b[0]<a[2]+gap and a[1]<b[3]+gap and b[1]<a[3]+gap


def beam_fits_support(size,pallet_size):
    """Bound unsupported depth; do not promise storage for multi-metre beams."""
    return size[0]<=pallet_size[0]+.50 and size[1]<=pallet_size[1]-.06


def upper_face(directions):
    """Cube-local face that points most upward after tipping/rotation."""
    axis=max(range(3),key=lambda i:abs(directions[i][2])/max(1e-12,math.sqrt(sum(v*v for v in directions[i]))))
    return axis,1. if directions[axis][2]>=0 else -1.,[i for i in range(3) if i!=axis]


def supported_centres(low,high,half,preferred,step=.12):
    """Nearest-first pallet positions that fully support the measured XY load."""
    axes=[]
    for i in range(2):
        first,last=low[i]+half[i]+.015,high[i]-half[i]-.015
        if first>last:return []
        values=[first,last,min(last,max(first,preferred[i]))]
        values += [first+n*step for n in range(int((last-first)/step)+1)]
        axes.append(sorted(set(values)))
    return sorted([(x,y) for x in axes[0] for y in axes[1]],
                  key=lambda p:math.dist(p,preferred))


class CleanupQueue:
    def __init__(self,groups,unknown=(),static_occupancy=()):
        self.items={g['id']:{'id':g['id'],'kind':g['kind'],'paths':g['paths'],'state':'pending',
                              'robot':None,'attempts':{},'reason':None} for g in groups}
        if len(self.items)!=len(groups):raise ValueError('Duplicate cleanup object')
        paths=[path for group in groups for path in group['paths']]
        if len(paths)!=len(set(paths)):raise ValueError('A component belongs to multiple cleanup objects')
        self.unknown=list(unknown)
        self.epoch=0
        self.slots={}
        self.static_occupancy=list(static_occupancy)

    def available(self,robot):
        return [key for key,item in self.items.items()
                if item['state']=='pending' and item['attempts'].get(robot)!=self.epoch
                and not (item.get('avoid_robot')==robot and self.epoch<=item.get('avoid_until_epoch',-1))]

    def reject(self,robot,keys,reason):
        for key in keys:
            if self.items[key]['state']=='pending':
                self.items[key]['attempts'][robot]=self.epoch
                self.items[key]['reason']=reason
                if reason.startswith('Oversized beam:'):self.items[key]['state']='blocked'

    def slot_free(self,box):
        return not any(overlaps(box,other) for other in self.static_occupancy) and not any(
            overlaps(box,slot['box']) for slot in self.slots.values())

    def claim(self,key,robot,slot):
        item=self.items[key]
        if item['state']!='pending' or not self.slot_free(slot['box']) or any(
                other['state']=='assigned' and other['robot']==robot for other in self.items.values()):
            raise ValueError('Object or rack footprint already reserved')
        lease=uuid.uuid4().hex
        self.slots[lease]=dict(slot,robot=robot,object_id=key,state='reserved')
        item.update(state='assigned',robot=robot,lease=lease,reason=None)
        return lease

    def complete(self,key,verified,actual_box=None):
        item=self.items[key]
        if item['state']!='assigned' or not verified:raise ValueError('Verified placement required')
        if actual_box is not None:
            own=self.slots[item['lease']]
            combined=(min(own['box'][0],actual_box[0]),min(own['box'][1],actual_box[1]),
                      max(own['box'][2],actual_box[2]),max(own['box'][3],actual_box[3]))
            if any(overlaps(combined,box) for box in self.static_occupancy) or any(
                    overlaps(combined,slot['box']) for lease,slot in self.slots.items() if lease!=item['lease']):
                raise ValueError('Settled object footprint conflicts with another reserved rack location')
            own['box']=combined
        item['state']='stored'
        self.slots[item['lease']]['state']='occupied'
        self.epoch+=1

    def can_retarget(self,key,slot):
        item=self.items[key]
        if item['state']!='assigned':return False
        box=slot['box']
        return not any(overlaps(box,other) for other in self.static_occupancy) and not any(
            overlaps(box,other['box']) for lease,other in self.slots.items() if lease!=item['lease'])

    def retarget(self,key,slot):
        if not self.can_retarget(key,slot):raise ValueError('Adjusted placement conflicts with reserved/occupied rack space')
        item=self.items[key]
        self.slots[item['lease']]=dict(slot,robot=item['robot'],object_id=key,state='reserved')

    def fail(self,key,reason,before_manipulation=False):
        item=self.items[key]
        if item['state']!='assigned':return
        if before_manipulation:
            self.slots.pop(item['lease'])
            item.pop('lease',None)
            item['attempts'][item['robot']]=self.epoch
            item.update(state='pending',robot=None)
        else:
            # Never reassign an object whose source geometry was transferred
            # into a payload, or reuse its rack reservation after a failure.
            item['state']='failed'
            self.epoch+=1
        item['reason']=reason

    def recovered(self,key,pose_error,speed,on_floor):
        """Requeue only after a physical release/settle and exact geometry handoff."""
        item=self.items[key]
        if item['state']!='failed' or not on_floor or not all(math.isfinite(v) for v in (pose_error,speed)) or pose_error>.002 or speed>.05:
            raise ValueError('Verified stable floor recovery required before reassignment')
        self.slots.pop(item.get('lease'),None)
        item.pop('lease',None)
        item['recoveries']=item.get('recoveries',0)+1
        self.epoch+=1
        item['attempts'][item['robot']]=self.epoch
        item['avoid_robot']=item['robot']
        item['avoid_until_epoch']=self.epoch+1
        item.update(state='pending' if item['recoveries']<3 else 'blocked',robot=None,
                    reason='Recovered physically to floor; retry with another grasp/robot' if item['recoveries']<3 else 'Repeated manipulation failures; needs operator review')

    def recovered_storage(self,key,contact_verified,support_gap,speed,box):
        item=self.items[key]
        if item['state']!='failed' or not contact_verified or not all(math.isfinite(v) for v in (support_gap,speed)) or abs(support_gap)>.025 or speed>.05:
            raise ValueError('Verified contact, support and settling required')
        item['state']='assigned'
        try:self.complete(key,True,box)
        except Exception:
            item['state']='failed'
            raise
        item['reason']='Verified supported rack settling after recovery release'

    def summary(self):
        counts={state:sum(item['state']==state for item in self.items.values())
                for state in ('pending','assigned','stored','blocked','failed')}
        return {'mode':'continuous shared floor cleanup','floor_clean':all(item['state']=='stored' for item in self.items.values()) and not self.unknown,
                'counts':counts,'total_objects':len(self.items),'unregistered_components':self.unknown,
                'remaining_objects':[item for item in self.items.values() if item['state']!='stored'],
                'rack_reservations':list(self.slots.values()),
                'scope':'Registered movable disaster props; warehouse stock and rack structure are not cleanup objects.'}


class FairTripGate:
    """One complete travel/manipulation trip at a time through shared aisles."""
    def __init__(self):
        self.owner=None
        self.queue=[]

    def request(self,robot):
        if robot!=self.owner and robot not in self.queue:self.queue.append(robot)

    def enter(self,robot):
        if self.owner==robot:return True
        if self.owner is not None or not self.queue or self.queue[0]!=robot:return False
        self.queue.pop(0)
        self.owner=robot
        return True

    def release(self,robot):
        if self.owner==robot:self.owner=None
        if robot in self.queue:self.queue.remove(robot)


def _point_segment_distance(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    length=dx*dx+dy*dy
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length)) if length else 0.
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def route_prefix(path,distance=2.):
    """Rolling reservation horizon; release used space before reserving more."""
    points=[tuple(float(v) for v in p[:2]) for p in path]
    if not points or distance<=0:raise ValueError('Positive route horizon and nonempty path required')
    result=[points[0]]
    for a,b in zip(points,points[1:]):
        length=math.dist(a,b)
        if length>distance:
            result.append(tuple(a[i]+(b[i]-a[i])*distance/length for i in (0,1)))
            return result
        result.append(b)
        distance-=length
        if distance<1e-8:return result
    return result


def _segment_distance(a,b,c,d):
    def cross(p,q,r):return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    # Intersection, including collinear overlapping segments.
    if (cross(a,b,c)*cross(a,b,d)<=0 and cross(c,d,a)*cross(c,d,b)<=0
            and max(min(a[0],b[0]),min(c[0],d[0]))<=min(max(a[0],b[0]),max(c[0],d[0]))
            and max(min(a[1],b[1]),min(c[1],d[1]))<=min(max(a[1],b[1]),max(c[1],d[1]))):return 0.
    return min(_point_segment_distance(a,c,d),_point_segment_distance(b,c,d),
               _point_segment_distance(c,a,b),_point_segment_distance(d,a,b))


def _segment_box(a,b,box):
    low,high=0.,1.
    for axis in (0,1):
        delta=b[axis]-a[axis]
        if abs(delta)<1e-12:
            if not box[axis]<=a[axis]<=box[axis+2]:return False
        else:
            t0,t1=(box[axis]-a[axis])/delta,(box[axis+2]-a[axis])/delta
            low=max(low,min(t0,t1));high=min(high,max(t0,t1))
            if low>high:return False
    return True


class RouteReservations:
    """Admit independent paths concurrently; conflicting swept paths yield.

    Radius encloses the base and carried load in XY, including goal rotations.
    This coordinates planned routes; the live local collision check remains active.
    """
    def __init__(self):
        self.routes={}
        self.peak_active=0
        self.denials=0
        self.events=[]

    def reserve(self,robot,path,radius,positions,time=0.,footprint=None,occupied_boxes=(),goal_radius=None):
        points=[tuple(float(v) for v in p[:2]) for p in path]
        if not points or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in points) or not math.isfinite(radius) or radius<=0:
            raise ValueError('Invalid reserved path')
        if len(points)==1:points*=2
        segments=list(zip(points,points[1:]))
        for other,entry in self.routes.items():
            if other==robot:continue
            if any(_segment_distance(a,b,c,d)<radius+entry['radius']-.001
                   for a,b in segments for c,d in zip(entry['path'],entry['path'][1:])):
                self.denials+=1
                return False
        for other,(point,other_radius) in positions.items():
            if other==robot or other in self.routes:continue
            if footprint is not None:
                box=(point[0]-other_radius-footprint[2],point[1]-other_radius-footprint[3],
                     point[0]+other_radius-footprint[0],point[1]+other_radius-footprint[1])
                conflict=any(_segment_box(a,b,box) for a,b in segments)
            else:conflict=any(_point_segment_distance(point,a,b)<radius+other_radius-.001 for a,b in segments)
            if goal_radius and math.dist(points[-1],point)<goal_radius+other_radius:conflict=True
            if conflict:
                self.denials+=1
                return False
        if footprint is not None:
            for occupied in occupied_boxes:
                box=(occupied[0]-footprint[2],occupied[1]-footprint[3],occupied[2]-footprint[0],occupied[3]-footprint[1])
                if any(_segment_box(a,b,box) for a,b in segments):
                    self.denials+=1
                    return False
        self.routes[robot]={'path':points,'radius':radius}
        self.peak_active=max(self.peak_active,len(self.routes))
        self.events.append({'robot':robot,'event':'route_reserved','time':time,'active_routes':len(self.routes)})
        return True

    def release(self,robot,time=0.):
        if robot in self.routes:
            del self.routes[robot]
            self.events.append({'robot':robot,'event':'route_released','time':time,'active_routes':len(self.routes)})
