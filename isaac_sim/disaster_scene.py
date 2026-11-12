"""Build an editable disaster USD scene and optionally run the Isaac GUI.

Robots reference official Isaac USD assets with physics disabled for display.
Cameras are real USD cameras; ROS publication is not enabled in this scene.
"""
import argparse
import logging
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'disaster_mrta'))
from disaster_mrta.tasks import Task, area_tasks, action_mask, load_tasks
from disaster_mrta.config_loader import load_config

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--headless', action='store_true')
parser.add_argument('--robot-scale', type=float, default=.25, help='Robot display scale relative to original USD (default .25)')
parser.add_argument('--parked', action='store_true', help='Disable scripted exploration')
parser.add_argument('--tasks-file', type=Path, help='JSON list of fixed id/type/x/y tasks')
parser.add_argument('--robot-assets-root', help='Local folder or URL containing Isaac/Robots')
parser.add_argument('--build-only', action='store_true')
parser.add_argument('--steps', type=int, default=0)
parser.add_argument('--environment', choices=['warehouse', 'disaster'], default='warehouse')
parser.add_argument('--preview', type=Path, help='Save a rendered PNG of the initial scene')
parser.add_argument('--scenario', choices=['scenario_1', 'scenario_2', 'scenario_3'], default='scenario_3')
parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'scenes' / 'disaster_scene.usda')
args, kit_args = parser.parse_known_args()
if not 0 < args.robot_scale < float('inf'):
    parser.error('--robot-scale must be positive and finite')
from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless, 'width': 1280, 'height': 720})
try:
    from compat import World, cube, camera, set_camera_view, Gf, UsdGeom, UsdLux
    import omni.usd
    import math
    logging.basicConfig(level=logging.INFO)
    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    if args.environment == 'disaster':
        world.scene.add_default_ground_plane()
    light = UsdLux.DomeLight.Define(stage, '/World/Lighting')
    light.CreateIntensityAttr(1800)
    if args.environment == 'warehouse':
        from warehouse import build_warehouse
        build_warehouse(stage)
        light.CreateIntensityAttr(450)
    stage.GetPrimAtPath('/World').SetCustomDataByKey('research_assumptions', 'Official robot USD references with physics disabled; synthetic disaster markers; no trained perception or ROS bridge; no validated vehicle dynamics.')
    zones = [
        ('fire_zone', (-8, 5), (1,.15,.03)),
        ('flood_zone', (8, 5), (.05,.35,.9)),
        ('damaged_building_zone', (-8,-5), (.6,.5,.35)),
        ('human_damage_search_area', (8,-5), (.9,.15,.6)),
        ('landslide_zone', (0,10), (.45,.3,.15)),
    ]
    if args.environment == 'warehouse':
        zones = [
            ('fire_zone', (-6, 8), (1,.15,.03)),
            ('flood_zone', (6, 8), (.05,.35,.9)),
            ('damaged_building_zone', (-4, -6), (.6,.5,.35)),
            ('human_damage_search_area', (6, 1), (.9,.15,.6)),
            ('landslide_zone', (-6, 1), (.45,.3,.15)),
        ]
    chosen = {'scenario_1': {'fire_zone','damaged_building_zone'}, 'scenario_2': {'flood_zone','human_damage_search_area'}, 'scenario_3': {z[0] for z in zones}}[args.scenario]
    labels = {'fire_zone': 'Fire Disaster', 'flood_zone': 'Water Disaster',
              'damaged_building_zone': 'Damaged Infrastructure',
              'human_damage_search_area': 'Human Damage', 'landslide_zone': 'Land Disaster'}
    tasks = []
    for name, (x,y), color in zones:
        if name not in chosen:
            continue
        size = (1.2, 1.2, .015) if args.environment == 'warehouse' else (5,5,.05)
        region = cube(stage, '/World/Zones/'+name, (x,y,.025), size, color)
        region.GetPrim().SetCustomDataByKey('zone_type', name)
        region.GetPrim().SetCustomDataByKey('synthetic_marker', True)
        # Synthetic class scores illustrate the interface; no model is run here.
        tasks.extend(area_tasks(name, (x, y), {labels[name]: 0.9}))
        if name == 'damaged_building_zone' and args.environment == 'disaster':
            for i in range(4):
                cube(stage, f'/World/Rubble/block_{i}', (x+i*.7-1,y,.4+i*.2), (.6,1,.8+i*.4), (.45,.45,.45))
        elif name == 'human_damage_search_area':
            region.GetPrim().SetCustomDataByKey('interpretation', 'Area search clue; no person detected or localized')
        elif name == 'fire_zone':
            for i in range(3):
                cube(stage, f'/World/FireMarkers/marker_{i}', (x+i-.9,y,.6), (.5,.5,1.2), (1,.3,.03))
    if args.environment == 'disaster':
        for i in range(6):
            cube(stage, f'/World/Obstacles/barrier_{i}', (i*3-8,0,.5), (1,.6,1), (.4,.43,.46))
    roots = []
    config_dir = Path(__file__).resolve().parents[1] / 'src' / 'disaster_mrta' / 'config'
    from asset_robots import asset_fleet, resolve_assets_root, add_asset_robot
    robots = asset_fleet(load_config(config_dir / 'robots.yaml')['robots'])
    assets_root = resolve_assets_root(args.robot_assets_root)
    reserve = load_config(config_dir / 'mrta.yaml')['battery_reserve']
    # Explicit scenario demand at the staging area, independent of image classes.
    tasks.append(Task('staging_supply', 'supply_delivery', (6, -8), 0.7,
                      'supply_delivery', 15.0, {}, evidence='Synthetic staging-area supply request'))
    if args.tasks_file:
        tasks=load_tasks(args.tasks_file)
        for task in tasks:
            marker=cube(stage,'/World/CustomTasks/task_'+task.task_id,(*task.location,.035),(.5,.5,.02),(.15,.8,.5))
            marker.GetPrim().SetCustomDataByKey('task_json',json.dumps(task.to_dict()))
    starts = [(-3,-10,2),(-6,-10,2),(0,-10,.1),(6,-10,.05)]
    if args.environment == 'warehouse':
        starts = [(-6,-9,1.5),(-3,-9,1.6),(0,-12,.1),(6,-9,.05)]
    from exploration import warehouse_patrols
    patrols = warehouse_patrols()
    if args.environment == 'disaster':
        patrols = [([(x,-12,z),(x,-9,z)], .4) for x, _, z in starts]
    if not args.parked:
        starts = [route[0][0] for route in patrols]
    colors = [(.95,.75,.1),(.1,.7,.25),(.15,.5,.95),(.8,.25,.65)]
    for i, robot in enumerate(robots):
        name = robot['robot_id']
        path = '/World/Robots/'+name
        root = UsdGeom.Xform.Define(stage, path)
        translation = root.AddTranslateOp()
        translation.Set(Gf.Vec3d(*starts[i]))
        yaw = root.AddRotateZOp()
        roots.append((translation, yaw))
        robots[i].update(x=starts[i][0], y=starts[i][1], z=starts[i][2])
        root.GetPrim().SetCustomDataByKey('robot_id', name)
        root.GetPrim().SetCustomDataByKey('camera_topic', '/'+name+'/camera/image_raw')
        add_asset_robot(stage, path, robot, assets_root, args.robot_scale)
        root.GetPrim().SetCustomDataByKey('task_role', robot['task_role'])
        camera(stage, path+'/camera', (.65,0,.3))
    from navigation import Explorer, scene_obstacles, footprint, obstacles_for
    explorers=[]
    for robot, (route, speed) in zip(robots, patrols):
        radius, vertical = footprint(stage, '/World/Robots/'+robot['robot_id'], route[0])
        robot.update(clearance_radius_m=radius)
        explorers.append(Explorer(route, speed, radius, vertical))
    if args.environment == 'warehouse':
        set_camera_view(eye=[0,-32,15], target=[0,2,2])
    else:
        set_camera_view(eye=[24,-30,25], target=[0,0,0])
    world.reset()
    for _ in range(3):
        world.step(render=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    task_data = {
        'scenario': args.scenario,
        'environment': args.environment,
        'score_source': 'synthetic demonstration; no classifier inference',
        'cost_units': 'execution seconds; energy in battery percentage points',
        'action_order': ['idle'] + [task.task_id for task in tasks],
        'tasks': [task.to_dict() for task in tasks],
        'robots': robots,
        'initial_action_masks': {robot['robot_id']: action_mask(robot, tasks, battery_reserve=reserve)
                                 for robot in robots},
        'route_safety': 'Unknown; delivery blocked until a planner confirms safety',
        'policy': 'Parked' if args.parked else 'Kinematic exploration with clearance-aware A* and obstacle checks; no QMIX',
        'patrols': [] if args.parked else [{'robot_id': robot['robot_id'], 'waypoints': route, 'speed_mps': speed}
                                          for robot, (route, speed) in zip(robots, patrols)],
    }
    stage.GetPrimAtPath('/World').SetCustomDataByKey('task_catalog_json', json.dumps(task_data))
    if not stage.Export(str(args.output.resolve())):
        raise RuntimeError('USD export failed')
    args.output.with_suffix('.tasks.json').write_text(json.dumps(task_data, indent=2) + '\n', encoding='utf-8')
    logging.info('Saved editable scene: %s', args.output.resolve())
    if args.preview:
        import asyncio
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        args.preview.parent.mkdir(parents=True, exist_ok=True)
        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError('No viewport available for preview capture')
        viewport.resolution = (1280, 720)
        for _ in range(40):
            world.step(render=True)
        capture = capture_viewport_to_file(viewport, str(args.preview.resolve()))
        pending = asyncio.ensure_future(capture.wait_for_result())
        for _ in range(300):
            app.update()
            if pending.done():
                pending.result()
                break
        else:
            raise RuntimeError('Preview capture timed out')
    if not args.build_only:
        # Start playback automatically; pause in the GUI to pause patrol motion.
        world.play()
        previous_time = world.current_time
        boxes=scene_obstacles(stage)
        refresh_at=previous_time+1
        frame = 0
        while app.is_running() and (not args.steps or frame < args.steps):
            world.step(render=True)
            if world.is_playing() and not args.parked:
                now=world.current_time
                dt=max(0,min(now-previous_time,.1))
                if now >= refresh_at or now < previous_time:
                    boxes=scene_obstacles(stage)
                    refresh_at=now+1
                for robot, (translation, yaw), explorer in zip(robots, roots, explorers):
                    old=explorer.position
                    position, heading = explorer.step(dt, obstacles_for(explorer,boxes,explorers))
                    translation.Set(Gf.Vec3d(*position))
                    yaw.Set(heading)
                    robot.update(x=position[0], y=position[1], z=position[2],
                                 velocity=math.dist(old,position)/dt if dt else 0,
                                 navigation_status=explorer.state)
                previous_time=now
            frame += 1
        print('Exploration state: '+json.dumps([{k:r.get(k) for k in ('robot_id','x','y','navigation_status')}
                                               for r in robots]), flush=True)
except Exception:
    logging.exception('Disaster scene failed')
    raise
finally:
    app.close()
