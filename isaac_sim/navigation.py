"""Clearance-aware 2D A* for kinematic scene exploration at fixed altitude."""
import heapq
import math


def clear_segment(a, b, rectangles, radius):
    """Exact segment versus inflated axis-aligned boxes, including contact."""
    for xmin, ymin, xmax, ymax in rectangles:
        lo, hi = 0., 1.
        for start, delta, low, high in ((a[0], b[0]-a[0], xmin-radius, xmax+radius),
                                       (a[1], b[1]-a[1], ymin-radius, ymax+radius)):
            if abs(delta) < 1e-12:
                if start < low or start > high:
                    break
            else:
                near, far = sorted(((low-start)/delta, (high-start)/delta))
                lo, hi = max(lo, near), min(hi, far)
                if lo > hi:
                    break
        else:
            return False
    return True


def plan(start, goal, rectangles, radius, bounds=(-12.,-15.,12.,15.), cell=.3):
    """Return collision-checked XY waypoints, or [] if the goal is unreachable."""
    xmin,ymin,xmax,ymax = bounds
    def inside(p):
        return xmin+radius <= p[0] <= xmax-radius and ymin+radius <= p[1] <= ymax-radius
    if not inside(start) or not inside(goal) or not clear_segment(start,start,rectangles,radius) or not clear_segment(goal,goal,rectangles,radius):
        return []
    if clear_segment(start,goal,rectangles,radius):
        return [goal]
    def node(p):
        return round((p[0]-xmin)/cell), round((p[1]-ymin)/cell)
    def point(n):
        return xmin+n[0]*cell, ymin+n[1]*cell
    # Virtual endpoints connect to nearby free grid nodes without teleporting.
    def connectors(p):
        cx,cy = node(p)
        return [(cx+dx,cy+dy) for dx in (-1,0,1) for dy in (-1,0,1)
                if inside(point((cx+dx,cy+dy))) and clear_segment(p,point((cx+dx,cy+dy)),rectangles,radius)]
    starts, ends = connectors(start), set(connectors(goal))
    if not starts or not ends:
        return []
    blocked = set()
    for x0,y0,x1,y1 in rectangles:
        for ix in range(max(0,math.floor((x0-radius-xmin)/cell)), min(math.ceil((xmax-xmin)/cell),math.ceil((x1+radius-xmin)/cell))+1):
            for iy in range(max(0,math.floor((y0-radius-ymin)/cell)), min(math.ceil((ymax-ymin)/cell),math.ceil((y1+radius-ymin)/cell))+1):
                p=point((ix,iy))
                if x0-radius <= p[0] <= x1+radius and y0-radius <= p[1] <= y1+radius:
                    blocked.add((ix,iy))
    queue, costs, parents = [], {}, {}
    for n in starts:
        cost=math.dist(start,point(n))
        costs[n]=cost
        parents[n]=None
        heapq.heappush(queue,(cost+math.dist(point(n),goal),cost,n))
    while queue:
        _,cost,current=heapq.heappop(queue)
        if cost > costs[current]:
            continue
        if current in ends:
            path=[goal]
            while current is not None:
                path.append(point(current))
                current=parents[current]
            path.reverse()
            # Every shortcut still checks the full robot clearance.
            smooth=[]
            origin=start
            for i in range(len(path)-1):
                if not clear_segment(origin,path[i+1],rectangles,radius):
                    smooth.append(path[i])
                    origin=path[i]
            return smooth+[goal]
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            nxt=current[0]+dx,current[1]+dy
            p=point(nxt)
            if nxt in blocked or not inside(p):
                continue
            if not clear_segment(point(current),p,rectangles,radius):
                continue
            new=cost+math.hypot(dx,dy)*cell
            if new < costs.get(nxt,float('inf')):
                costs[nxt]=new
                parents[nxt]=current
                heapq.heappush(queue,(new+math.dist(p,goal),new,nxt))
    return []


class Explorer:
    def __init__(self, route, speed, radius, vertical_offsets, bounds=(-12.,-15.,12.,15.), cell=.3):
        self.route, self.speed, self.radius = route, speed, radius
        self.position = tuple(route[0])
        self.vertical_offsets = vertical_offsets
        self.goal = 1
        self.path = []
        self.heading = 0.
        self.retry = 0.
        self.state = 'exploring'
        self.bounds, self.cell = bounds, cell

    def step(self, dt, obstacles):
        if dt <= 0:
            return self.position, self.heading
        destination=self.route[self.goal]
        if math.dist(self.position[:2],destination[:2]) < .05:
            self.goal=(self.goal+1)%len(self.route)
            destination=self.route[self.goal]
            self.path=[]
        if self.path and not clear_segment(self.position,self.path[0],obstacles,self.radius):
            self.path=[]
            self.retry=0
        self.retry=max(0,self.retry-dt)
        if not self.path and self.retry == 0:
            self.path=plan(self.position[:2],destination[:2],obstacles,self.radius,self.bounds,self.cell)
            self.retry=1.
        self.state='exploring' if self.path else 'blocked'
        if self.path:
            target=self.path[0]
            distance=math.dist(self.position[:2],target)
            step=min(self.speed*dt,distance)
            if distance > 1e-9:
                x,y,z=self.position
                dx,dy=target[0]-x,target[1]-y
                new=(x+dx/distance*step,y+dy/distance*step,z)
                if clear_segment(self.position,new,obstacles,self.radius):
                    self.position=new
                    self.heading=math.degrees(math.atan2(dy,dx))
            if distance <= step+1e-9:
                self.path.pop(0)
        return self.position,self.heading


def scene_obstacles(stage):
    """World bounds of visible static geometry; omit floors and robot visuals."""
    from pxr import Usd, UsdGeom, UsdPhysics
    cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default','render','proxy'])
    boxes=[]
    for prim in stage.Traverse():
        path=str(prim.GetPath())
        if path.startswith('/World/Robots') or path.startswith('/World/Zones'):
            continue
        if not prim.IsA(UsdGeom.Boundable) or 'Floor' in path or 'GroundPlane' in path:
            continue
        if UsdGeom.Imageable(prim).ComputeVisibility() == 'invisible':
            continue
        collision=prim.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        if not collision and not path.startswith(('/World/FireMarkers','/World/Rubble','/World/Obstacles')):
            continue
        bounds=cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if not bounds.IsEmpty():
            boxes.append((tuple(bounds.GetMin()),tuple(bounds.GetMax())))
    return boxes


def footprint(stage, path, position):
    from pxr import Usd, UsdGeom
    box=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy']).ComputeWorldBound(stage.GetPrimAtPath(path+'/display')).ComputeAlignedRange()
    low,high=box.GetMin(),box.GetMax()
    radius=math.hypot(max(abs(low[0]-position[0]),abs(high[0]-position[0])),
                      max(abs(low[1]-position[1]),abs(high[1]-position[1])))+.12
    return radius,(low[2]-position[2]-.06,high[2]-position[2]+.06)


def obstacles_for(explorer, boxes, peers):
    low=explorer.position[2]+explorer.vertical_offsets[0]
    high=explorer.position[2]+explorer.vertical_offsets[1]
    result=[(a[0],a[1],b[0],b[1]) for a,b in boxes if a[2] <= high and b[2] >= low]
    for peer in peers:
        if peer is explorer:
            continue
        if peer.position[2]+peer.vertical_offsets[0] <= high and peer.position[2]+peer.vertical_offsets[1] >= low:
            x,y,_=peer.position
            r=peer.radius
            result.append((x-r,y-r,x+r,y+r))
    return result
