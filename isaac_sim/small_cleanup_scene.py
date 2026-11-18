"""Spaced small floor props for a bounded five-robot warehouse cleanup demo."""


def small_box_positions(count=20):
    if count not in (5,10,15,20):raise ValueError('Choose 5, 10, 15 or 20 boxes')
    return [(x,y) for y in (-3.7,-1.2,1.3,3.8) for x in (-7.4,-4.,-1.5,3.4,7.)][:count]


def grouped_box_positions(count=20,shared_pile=False):
    small_box_positions(count)  # Validate the supported counts.
    if shared_pile:
        return [(x,y) for y in (-5.9,-5.,-4.1,-3.2) for x in (-2.,-1.,0.,1.,2.)][:count]
    centres=small_box_positions(5)
    offsets=[(-.45,-.45),(.45,-.45),(-.45,.45),(.45,.45)]
    return [(x+dx,y+dy) for dx,dy in offsets for x,y in centres][:count]


def prepare_small_cleanup(stage,count=20,layout='grouped'):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    root=stage.GetPrimAtPath('/World/DisasterAftermath/Props')
    # Session-only scene preparation, before physics starts. Original USD stays
    # untouched; excluded disaster props are neither visible nor colliding.
    for prim in list(root.GetChildren()):
        UsdGeom.Imageable(prim).MakeInvisible()
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)
    positions=small_box_positions(count) if layout=='batched' else grouped_box_positions(count,layout=='pile')
    colours=[(.57,.38,.19),(.70,.52,.30),(.63,.43,.23),(.77,.61,.39),(.52,.35,.19)]
    jobs=[dict(robot=i,task='Store small boxes',candidates=[]) for i in range(1,6)]
    for index,(x,y) in enumerate(positions,1):
        column=(index-1)%5
        colour=colours[column]
        cube=UsdGeom.Cube.Define(stage,str(root.GetPath())+'/Debris_Demo_%02d'%index)
        cube.CreateSizeAttr(.24)
        cube.AddTranslateOp().Set(Gf.Vec3d(x,y,.12))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
        # A visual tape band belongs to the carton and travels with its payload.
        # The box remains the collision/grasp geometry, within 1 mm of the tape.
        tape=UsdGeom.Cube.Define(stage,str(cube.GetPath())+'/TapeBand')
        tape.CreateSizeAttr(1.)
        tape.AddScaleOp().Set(Gf.Vec3f(.04,.242,.242))
        tape.CreateDisplayColorAttr([Gf.Vec3f(.87,.73,.46)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        if layout=='batched' and index>5:
            UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
            UsdPhysics.CollisionAPI(cube.GetPrim()).CreateCollisionEnabledAttr(False)
        jobs[column]['candidates'].append([cube.GetPrim()])
    return jobs


def activate_small_wave(stage,keys):
    """Introduce new tasks before pickup; never move a carried/completed box."""
    from pxr import Gf, UsdGeom, UsdPhysics
    positions=small_box_positions(5)
    for key in keys:
        prim=stage.GetPrimAtPath(key)
        index=int(prim.GetName().rsplit('_',1)[-1])-1
        x,y=positions[index%5]
        UsdGeom.Xformable(prim).GetOrderedXformOps()[0].Set(Gf.Vec3d(x,y,.12))
        UsdGeom.Imageable(prim).MakeVisible()
        UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(True)
