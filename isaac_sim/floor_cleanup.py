"""Shared floor inventory and collision-free rack reservations (no Isaac imports)."""
import uuid


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
                if item['state']=='pending' and item['attempts'].get(robot)!=self.epoch]

    def reject(self,robot,keys,reason):
        for key in keys:
            if self.items[key]['state']=='pending':
                self.items[key]['attempts'][robot]=self.epoch
                self.items[key]['reason']=reason

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

    def summary(self):
        counts={state:sum(item['state']==state for item in self.items.values())
                for state in ('pending','assigned','stored','failed')}
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
