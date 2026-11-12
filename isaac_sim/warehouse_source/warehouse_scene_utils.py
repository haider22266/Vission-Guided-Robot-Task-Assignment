"""Shared scene edits for the generated warehouse and its task controller."""


def remove_floating_hazard_signs(stage):
    """Deactivate the four unsupported placards and their stripe decorations."""
    root = stage.GetPrimAtPath('/World/DisasterAftermath/Props')
    removed = []
    if root:
        for prim in list(root.GetChildren()):
            if prim.GetName().startswith('Hazard_Sign_'):
                removed.append(str(prim.GetPath()))
                prim.SetActive(False)
    return removed


def remove_front_walls(stage):
    """Deactivate the front facade and its short corner returns, with colliders.

    The Simple Warehouse front is the minimum horizontal-Y boundary. Select
    architectural wall assemblies by their world bounds, not numeric suffixes.
    Changes are authored into the caller's current edit target.
    """
    from pxr import Usd, UsdGeom

    front_axis = 1 if UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z else 2
    units = UsdGeom.GetStageMetersPerUnit(stage)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render', 'proxy'])
    walls = []
    for prim in stage.Traverse():
        if not prim.GetName().lower().startswith('sm_wall') or not prim.IsA(UsdGeom.Xform):
            continue
        bound = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if not bound.IsEmpty():
            walls.append((prim, float(bound.GetMin()[front_axis])*units,
                          float(bound.GetMax()[front_axis])*units))
    if not walls:
        return []
    front = min(lo for _, lo, _ in walls)
    # Flat panels are 0.2 m deep; the front corner returns are 3 m deep.
    selected = [prim for prim, lo, hi in walls
                if lo <= front + .15 and hi <= front + 3.25]
    removed = []
    for prim in selected:
        path = str(prim.GetPath())
        if any(path.startswith(parent + '/') for parent in removed):
            continue
        prim.SetActive(False)
        removed.append(path)
    return removed
