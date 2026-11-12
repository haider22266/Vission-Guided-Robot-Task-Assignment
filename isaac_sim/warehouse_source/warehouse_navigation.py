"""Small deterministic 2D planner for the generated warehouse (meters)."""
import heapq
import math


class FloorPlanner:
    def __init__(self, rectangles, radius=.64, resolution=.20,
                 bounds=(-10.8, 11.9, -10.5, 12.5)):
        self.rectangles = [(a-radius, b+radius, c-radius, d+radius)
                           for a, b, c, d in rectangles]
        self.resolution, self.bounds = resolution, bounds

    def free(self, p):
        x, y = p
        a, b, c, d = self.bounds
        return (a <= x <= b and c <= y <= d
                and not any(a <= x <= b and c <= y <= d for a,b,c,d in self.rectangles))

    def clear(self, a, b):
        n = max(1, math.ceil(math.dist(a, b) / .07))
        return all(self.free((a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n))
                   for i in range(n+1))

    def plan(self, start, goal):
        start, goal = tuple(start), tuple(goal)
        if not self.free(start) or not self.free(goal):
            raise ValueError('Start or goal lacks robot clearance')
        if self.clear(start, goal):
            return [start, goal]
        a, _, c, _ = self.bounds
        def cell(p):
            return (round((p[0]-a)/self.resolution), round((p[1]-c)/self.resolution))
        def point(k):
            return (a+k[0]*self.resolution, c+k[1]*self.resolution)
        def nearby(p):
            k = cell(p)
            options = [(k[0]+i,k[1]+j) for i in range(-2,3) for j in range(-2,3)]
            options = [k for k in options if self.clear(p,point(k))]
            if not options:
                raise ValueError('Cannot connect pose to the navigation grid')
            return min(options,key=lambda k: math.dist(p,point(k)))
        first, last = nearby(start), nearby(goal)
        queue, cost, parent = [(0.,first)], {first:0.}, {}
        visited = set()
        while queue:
            _, k = heapq.heappop(queue)
            if k in visited:
                continue
            visited.add(k)
            if k == last:
                raw = [goal,point(k)]
                while k in parent:
                    k = parent[k]
                    raw.append(point(k))
                raw.append(start)
                raw.reverse()
                # Remove redundant grid points, checking the complete segment.
                route = [raw[0]]
                i = 0
                while i < len(raw)-1:
                    j = len(raw)-1
                    while j > i+1 and not self.clear(raw[i],raw[j]):
                        j -= 1
                    route.append(raw[j])
                    i = j
                if not all(self.clear(p,q) for p,q in zip(route,route[1:])):
                    raise ValueError('Route clearance verification failed')
                return route
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)):
                nxt = (k[0]+dx,k[1]+dy)
                if nxt in visited or not self.clear(point(k),point(nxt)):
                    continue
                candidate = cost[k] + math.hypot(dx,dy)*self.resolution
                if candidate < cost.get(nxt,float('inf')):
                    cost[nxt], parent[nxt] = candidate,k
                    heapq.heappush(queue,(candidate+math.dist(point(nxt),point(last)),nxt))
        raise ValueError('No collision-clear route found')
