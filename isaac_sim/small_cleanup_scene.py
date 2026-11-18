"""Five isolated, small floor props for a bounded warehouse cleanup demo."""


def prepare_small_cleanup(stage):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    root=stage.GetPrimAtPath('/World/DisasterAftermath/Props')
    # Session-only scene preparation, before physics starts. Original USD stays
    # untouched; excluded disaster props are neither visible nor colliding.
    for prim in list(root.GetChildren()):
        UsdGeom.Imageable(prim).MakeInvisible()
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)
    positions=[(-7.4,-3.7),(-4.,-3.7),(-1.5,-3.7),(3.4,-3.7),(7.,-3.7)]
    colours=[(.85,.30,.12),(.15,.55,.85),(.85,.65,.15),(.25,.70,.30),(.65,.30,.80)]
    jobs=[]
    for index,((x,y),colour) in enumerate(zip(positions,colours),1):
        cube=UsdGeom.Cube.Define(stage,str(root.GetPath())+'/Debris_Demo_%02d'%index)
        cube.CreateSizeAttr(.24)
        cube.AddTranslateOp().Set(Gf.Vec3d(x,y,.12))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        jobs.append(dict(robot=index,task='Store small box %02d'%index,candidates=[[cube.GetPrim()]]))
    return jobs
