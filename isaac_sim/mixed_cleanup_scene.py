"""Twenty visible tasks in the five original disaster-prop families."""
import math


def mixed_pickup_yaws(kind):
    # These source beams/shafts lie along X. Heading along Y keeps their
    # long dimension crosswise during carrying and rack approach.
    return (math.pi/2,-math.pi/2) if kind in ('beam','tool') else (0.,math.pi,math.pi/2,-math.pi/2)


def mixed_rack_footprint(box):
    return (box[0]-.04,box[1]-.04,box[2]+.04,box[3]+.04)


def mixed_rack_yaws(yaws,source_x,shelf_low_x,shelf_high_x):
    return tuple(sorted(yaws,key=lambda yaw:abs(
        (shelf_high_x if math.cos(yaw)<0 else shelf_low_x)-.70*math.cos(yaw)-source_x)))


def mixed_prop_specs(count=20):
    if count not in (5,10,15,20):raise ValueError('Choose 5, 10, 15 or 20 tasks')
    families=[('Rear-left carton','Rear_Left_Carton_Mixed',(-7.1,4.1),(.28,.24,.26),(.64,.43,.24)),
              ('Aisle rubble','Debris_Mixed',(-3.7,-1.7),(.22,.19,.16),(.40,.43,.46)),
              ('Screwdriver','Tool_Screwdriver_Handle_Mixed',(-2.8,-4.0),(.18,.10,.14),(.78,.12,.08)),
              ('Rear beam','Rear_Right_Beam_Mixed',(3.5,3.3),(.62,.10,.16),(.75,.43,.10)),
              ('Rear-right carton','Rear_Right_Carton_Mixed',(7.1,4.1),(.26,.24,.28),(.69,.49,.27))]
    offsets=[(-.48,-.48),(.48,-.48),(-.48,.48),(.48,.48)]
    result=[]
    for index in range(count):
        column=index%5
        label,prefix,centre,size,colour=families[column]
        dx,dy=offsets[index//5]
        result.append(dict(robot=column+1,label=label,name=prefix+'_%02d'%(index//5),
                           position=(centre[0]+dx,centre[1]+dy),size=size,colour=colour))
    return result


def prepare_mixed_cleanup(stage,count=20):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    root=stage.GetPrimAtPath('/World/DisasterAftermath/Props')
    for prim in list(root.GetChildren()):
        UsdGeom.Imageable(prim).MakeInvisible()
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)
    jobs=[dict(robot=i,task='',candidates=[]) for i in range(1,6)]

    def box(name,centre,size,colour):
        cube=UsdGeom.Cube.Define(stage,str(root.GetPath())+'/'+name)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        return cube.GetPrim()

    for spec in mixed_prop_specs(count):
        x,y=spec['position'];sx,sy,sz=spec['size']
        prim=box(spec['name'],(x,y,sz/2),spec['size'],spec['colour'])
        group=[prim]
        if 'carton' in spec['label']:
            tape=UsdGeom.Cube.Define(stage,str(prim.GetPath())+'/TapeBand')
            tape.CreateSizeAttr(1.)
            tape.AddScaleOp().Set(Gf.Vec3f(.15,1.008,1.008))
            tape.CreateDisplayColorAttr([Gf.Vec3f(.86,.73,.48)])
        elif spec['label']=='Screwdriver':
            shaft=box(spec['name'].replace('_Handle_','_Shaft_'),(x+sx/2+.12,y,sz/2),(.24,.028,.028),(.65,.68,.71))
            group.append(shaft)
        elif spec['label']=='Rear beam':
            # Visual metal flanges; the short beam's body is the grasp collider.
            for end in (-1,1):
                cap=UsdGeom.Cube.Define(stage,str(prim.GetPath())+'/Flange_'+str(end).replace('-','N'))
                cap.CreateSizeAttr(1.)
                cap.AddTranslateOp().Set(Gf.Vec3d(0,0,end*.43))
                cap.AddScaleOp().Set(Gf.Vec3f(1.005,1.25,.14))
                cap.CreateDisplayColorAttr([Gf.Vec3f(.38,.40,.42)])
        job=jobs[spec['robot']-1]
        job['task']=spec['label']
        job['candidates'].append(group)
    return jobs
