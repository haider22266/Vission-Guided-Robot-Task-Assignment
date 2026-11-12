"""Exclusive object/slot reservations for continuous rack sorting."""
from dataclasses import dataclass
import math


@dataclass
class Item:
    name: str
    kind: str
    lane: int
    width: float
    height: float
    state: str = 'available'
    owner: str | None = None
    slot: str | None = None
    completed: int = 0


class TaskBroker:
    def __init__(self,items,slots):
        self.items={item.name:item for item in items}
        if len(self.items)!=len(items):
            raise ValueError('Duplicate object names')
        self.slots=slots
        self.reserved={}
        self.occupied={}
        self.active={}
        self.history=[]
        self.serial=0

    def claim(self,robot,kind,lane):
        if robot in self.active:
            return self.active[robot]
        available=[i for i in self.items.values() if i.kind==kind and i.lane==lane and i.owner is None
                   and i.state in ('available','racked')]
        available.sort(key=lambda i:(i.state=='racked',i.completed,i.name))
        for item in available:
            choices=[name for name,data in self.slots.items() if data['lane']==lane
                     and name not in self.reserved and name not in self.occupied]
            if not choices:
                continue
            slot=choices[0]
            self.serial+=1
            task={'id':self.serial,'robot':robot,'object':item.name,'kind':kind,'slot':slot,
                  'status':'assigned','source_slot':item.slot,'grasp_verified':False}
            item.owner=robot
            self.reserved[slot]=robot
            self.active[robot]=task
            return task
        return None

    def grasp(self,robot,errors):
        task=self.active[robot]
        if not verified_contact(**errors):
            return False
        item=self.items[task['object']]
        task.update(status='carrying',grasp_verified=True,contact_errors=errors)
        item.state='carried'
        if item.slot:
            self.occupied.pop(item.slot,None)
            item.slot=None
        return True

    def complete(self,robot,position_error,release_clearance):
        task=self.active[robot]
        if (not task['grasp_verified'] or not math.isfinite(position_error) or position_error<0 or position_error>.004
                or not math.isfinite(release_clearance) or release_clearance<.003):
            return False
        item=self.items[task['object']]
        item.state='racked'
        item.slot=task['slot']
        item.owner=None
        item.completed+=1
        self.occupied[item.slot]=item.name
        self.reserved.pop(item.slot)
        task.update(status='completed',placement_error_m=position_error)
        self.history.append(task.copy())
        del self.active[robot]
        return True


def verified_contact(center_error,left_gap,right_gap,orientation_error,stable_frames):
    values=(center_error,left_gap,right_gap,orientation_error)
    return (all(math.isfinite(v) for v in values) and 0<=center_error<=.0015
            and abs(left_gap)<=.002 and abs(right_gap)<=.002
            and 0<=orientation_error<=.005 and stable_frames>=5)
