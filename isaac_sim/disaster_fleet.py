"""Add official Isaac USD task-role robots and a synthetic task catalog to a USD stage."""
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'disaster_mrta'))
from disaster_mrta.config_loader import load_config
from disaster_mrta.tasks import Task, area_tasks, action_mask, assignment_features, load_tasks


def add_disaster_fleet(stage, origin, assets_root=None, robot_scale=.25, tasks_file=None):
    from compat import cube, camera, Gf, UsdGeom, UsdLux
    base = '/DisasterResponse'
    if stage.GetPrimAtPath(base):
        raise ValueError('Stage already contains /DisasterResponse')
    root = UsdGeom.Xform.Define(stage, base).GetPrim()
    directory = Path(__file__).resolve().parents[1] / 'src' / 'disaster_mrta' / 'config'
    from asset_robots import asset_fleet, resolve_assets_root, add_asset_robot
    robots = asset_fleet(load_config(directory / 'robots.yaml')['robots'])
    assets_root = resolve_assets_root(assets_root)
    reserve = load_config(directory / 'mrta.yaml')['battery_reserve']
    ox, oy, oz = origin
    roles = [r['task_role'] for r in robots]
    offsets = [(-.5, 0, 1.2), (.8, .7, 1.4), (-1.2, 0, .1), (1.5, 0, .05)]
    colors = [(1.,.75,.06), (.95,.3,.08), (.12,.55,.95), (.15,.8,.3)]
    for robot, offset, role, color in zip(robots, offsets, roles, colors):
        path = base + '/Robots/' + robot['robot_id']
        shape = UsdGeom.Xform.Define(stage, path)
        location = (ox+offset[0], oy+offset[1], oz+offset[2])
        shape.AddTranslateOp().Set(Gf.Vec3d(*location))
        robot.update(x=location[0], y=location[1], z=location[2])
        shape.GetPrim().SetCustomDataByKey('robot_id', robot['robot_id'])
        shape.GetPrim().SetCustomDataByKey('task_role', role)
        shape.GetPrim().SetCustomDataByKey('capabilities', ','.join(robot['capabilities']))
        add_asset_robot(stage, path, robot, assets_root, robot_scale)
        camera(stage, path+'/camera', (.35,0,.15))

    # Each observation is synthetic and explicitly tied to an area, not a person.
    search = area_tasks('search_area', (ox-1, oy+2), {'Human_Damage': .9})
    hazard = area_tasks('hazard_area', (ox+1, oy+2), {
        'Fire_Disaster': .85, 'Water_Disaster': .7,
        'Land_Disaster': .65, 'Damaged_Infrastructure': .8})
    tasks = search + [task for task in hazard if task.task_type == 'hazard_inspection']
    tasks.append(Task('supply_request', 'supply_delivery', (ox, oy+3), .8,
                      'supply_delivery', 15, {}, evidence='Synthetic supply request; route unverified'))
    if tasks_file:
        tasks=load_tasks(tasks_file)
    for i, task in enumerate(tasks):
        marker = UsdGeom.Xform.Define(stage, base+'/Tasks/task_'+task.task_id)
        marker.GetPrim().SetCustomDataByKey('task_json', json.dumps(task.to_dict()))
        # Multiple hazard requirements share the same observation-area location.
        if tasks_file or i in (0, 1, len(tasks)-1):
            color = {'area_search': (1.,.75,.06), 'hazard_inspection': (.95,.3,.08),
                     'supply_delivery': (.15,.8,.3)}[task.task_type]
            cube(stage, str(marker.GetPath())+'/area', (*task.location, oz+.015), (.7,.7,.02), color)
    catalog = {
        'battery_reserve': reserve,
        'robots': robots, 'tasks': [t.to_dict() for t in tasks],
        'action_order': ['idle'] + [t.task_id for t in tasks],
        'initial_action_masks': {r['robot_id']: action_mask(r, tasks, battery_reserve=reserve) for r in robots},
        'assignment_features': {r['robot_id']: [assignment_features(r, t, battery_reserve=reserve)
                                               for t in tasks] for r in robots},
        'score_source': 'synthetic demonstration; no classifier run',
        'assumptions': 'Official USD robot assets displayed with physics disabled; no flight, drivetrain, navigation or QMIX controller. '
                       'Spawn and task coordinates require validation against the scene mesh. '
                       'Human Damage is an area search clue. Delivery blocked until route safety is confirmed.',
    }
    root.SetCustomDataByKey('task_catalog_json', json.dumps(catalog))
    # Local light helps display polygonal robots alongside captured scene radiance.
    light = UsdLux.DomeLight.Define(stage, base+'/RobotLighting')
    light.CreateIntensityAttr(500)
    return catalog
