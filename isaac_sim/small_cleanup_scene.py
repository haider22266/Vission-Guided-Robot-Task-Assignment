"""Spaced small floor props for a bounded five-robot warehouse cleanup demo."""


def small_box_positions(count=20):
    if count not in (5,10,15,20):raise ValueError('Choose 5, 10, 15 or 20 boxes')
    return [(x,y) for y in (-3.7,-1.2,1.3,3.8) for x in (-7.4,-4.,-1.5,3.4,7.)][:count]


def prepare_small_cleanup(stage,count=20):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    root=stage.GetPrimAtPath('/World/DisasterAftermath/Props')
    # Session-only scene preparation, before physics starts. Original USD stays
    # untouched; excluded disaster props are neither visible nor colliding.
    for prim in list(root.GetChildren()):
        UsdGeom.Imageable(prim).MakeInvisible()
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)
    positions=small_box_positions(count)
    colours=[(.85,.30,.12),(.15,.55,.85),(.85,.65,.15),(.25,.70,.30),(.65,.30,.80)]
    jobs=[dict(robot=i,task='Store small boxes',candidates=[]) for i in range(1,6)]
    for index,(x,y) in enumerate(positions,1):
        column=(index-1)%5
        colour=colours[column]
        cube=UsdGeom.Cube.Define(stage,str(root.GetPath())+'/Debris_Demo_%02d'%index)
        cube.CreateSizeAttr(.24)
        cube.AddTranslateOp().Set(Gf.Vec3d(x,y,.12))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        if index>5:
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
