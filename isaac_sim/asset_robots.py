"""Official Isaac Sim robot references, displayed without robot controllers."""
from copy import deepcopy


ASSETS = {
    'crazyflie': 'Bitcraze/Crazyflie/cf2x.usd',
    'ingenuity': 'NASA/Ingenuity/ingenuity.usd',
    'jackal': 'Clearpath/Jackal/jackal.usd',
    'forklift': 'IsaacSim/ForkliftC/forklift_c.usd',
}


def asset_fleet(configured_robots):
    """Task capabilities are scenario assumptions, not certifications of hardware."""
    by_id = {r['robot_id']: r for r in configured_robots}
    fleet = [deepcopy(by_id[name]) for name in ('uav_1', 'uav_1', 'ugv_1', 'ugv_3')]
    fleet[1]['robot_id'] = 'uav_2'
    fleet[2]['capabilities'] = sorted(set(fleet[2]['capabilities'] + by_id['ugv_2']['capabilities']))
    for robot, model, role in zip(fleet, ASSETS, ('area_search', 'area_search', 'hazard_inspection', 'supply_delivery')):
        robot.update(asset_model=model, asset_relative_path=ASSETS[model], task_role=role)
    return fleet


def resolve_assets_root(override=None):
    from isaacsim.storage.native import get_assets_root_path
    root = override or get_assets_root_path(skip_check=True)
    if not root:
        raise RuntimeError('Isaac asset root unavailable; provide --robot-assets-root <Isaac-version-root>')
    return str(root).rstrip('/\\')


def add_asset_robot(stage, path, robot, assets_root, display_scale=.25):
    """Reference the real USD at native scale, with physics disabled for display.

    Disabling physics is an override in the scene; source USD files stay intact.
    Flight, wheel, lift and manipulation control are separate future work.
    """
    import omni.client
    import math
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    if not math.isfinite(display_scale) or display_scale <= 0:
        raise ValueError('Robot scale must be positive and finite')
    # Scale a wrapper so authored asset transforms remain intact.
    UsdGeom.Xform.Define(stage, path+'/display').AddScaleOp().Set(Gf.Vec3f(display_scale))
    asset = assets_root + '/Isaac/Robots/' + robot['asset_relative_path']
    result, _ = omni.client.stat(asset)
    if result != omni.client.Result.OK:
        raise RuntimeError(f'Robot USD unavailable: {asset} ({result}). '
                           'Check network access or supply a local --robot-assets-root. No placeholder was substituted.')
    model = UsdGeom.Xform.Define(stage, path+'/display/model').GetPrim()
    if not model.GetReferences().AddReference(asset):
        raise RuntimeError(f'Could not reference robot USD: {asset}')
    children = list(Usd.PrimRange(model))
    if len(children) < 2:
        raise RuntimeError(f'Robot reference is empty: {asset}')
    meshes = 0
    for prim in children:
        for name in ('drawLines', 'drawPoints'):
            attr = prim.GetAttribute(name)
            if attr:
                attr.Set(False)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        if prim.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
        if prim.IsA(UsdGeom.Mesh):
            meshes += 1
    if not meshes:
        raise RuntimeError(f'No robot meshes loaded from {asset}')
    robot.update(asset_url=asset, loaded_mesh_count=meshes, display_scale=display_scale,
                 control_mode='USD display; physics disabled')
    stage.GetPrimAtPath(path).SetCustomDataByKey('asset_url', asset)
    print(f'Loaded {robot["robot_id"]}: {robot["asset_model"]} ({meshes} meshes)', flush=True)
    return model
