"""Load a reconstructed NuRec scene and optionally the NVIDIA Carter nav sample.

Adapted from NVIDIA's Isaac Sim Neural Volume Rendering example. This uses a
new editable root layer rather than opening USDZ as the root stage. Gaussian
rendering is not collision geometry; navigation requires the asset's mesh.
"""
import argparse
from pathlib import Path


SCENARIOS = {
    'andoria': ('hand_hold-endeavor-andoria', (0., .5, 0.), (0., 6., 0.)),
    'wormhole': ('hand_hold-endeavor-wormhole', (5., 0., 0.), (0., -4., 0.)),
    'cafe': ('nova_carter-cafe', (0., 0., 0.), (-3., -1.5, 0.)),
    'galileo': ('nova_carter-galileo', (-2.5, 2.5, 0.), (4., 0., 0.)),
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-root', type=Path, default=Path(__file__).resolve().parents[1]
                        / 'assets' / 'nurec')
    parser.add_argument('--scene', choices=SCENARIOS, default='galileo')
    parser.add_argument('--asset-file', default=None,
                        help='Filename within the scene folder; match the downloaded revision')
    parser.add_argument('--stage', type=Path, help='Override the dataset scene with a local USD/USDZ')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--steps', type=int, default=None, help='Simulation updates; 0 runs until closed')
    parser.add_argument('--navigation', action='store_true', help='Combined disaster fleet, tasks, mesh A* exploration and avoidance')
    parser.add_argument('--no-disaster-props', action='store_true', help='Omit added cartons, tools, debris and fallen rack beams')
    parser.add_argument('--tasks-file', type=Path, help='JSON task list using id/type/x/y; coordinates remain fixed')
    parser.add_argument('--stop-when-complete', action='store_true', help='Exit after all independent tasks finish')
    parser.add_argument('--camera-view', choices=['chase','rear','group','overview','tour','manual'], default=None, help='Robot follow views, fixed full-room overview, tour, or manual')
    parser.add_argument('--follow-robot', default='ugv_1', help='Ground robot to follow in chase view (ugv_1 Jackal or ugv_3 Forklift)')
    parser.add_argument('--camera-height', type=float, default=.45, help='Chase camera height in metres')
    parser.add_argument('--camera-distance', type=float, default=1.2, help='Chase camera distance behind the robot in metres')
    parser.add_argument('--camera-eye',type=float,nargs=3,help='Manual viewport eye XYZ')
    parser.add_argument('--camera-target',type=float,nargs=3,help='Manual viewport target XYZ (with --camera-eye)')
    parser.add_argument('--exploration-speed', type=float, default=.3, help='Convoy travel speed in m/s')
    parser.add_argument('--fleet-mode', choices=['convoy','independent'], default='convoy', help='Shared route together, or independent task allocation')
    parser.add_argument('--camera-dwell', type=float, default=8., help='Seconds per camera tour shot')
    parser.add_argument('--explore-first', type=float, default=15., help='Explore this many seconds before allocating tasks')
    parser.add_argument('--scene-only', action='store_true', help='View environment without Carter')
    parser.add_argument('--disaster-robots', action='store_true', help='Add Crazyflie, Ingenuity, Jackal, Forklift C and synthetic tasks')
    parser.add_argument('--robot-assets-root', help='Local folder or URL containing Isaac/Robots; defaults to Isaac asset server')
    parser.add_argument('--robot-scale', type=float, default=None, help='Robot USD scale (convoy default .15, otherwise .25)')
    parser.add_argument('--fleet-origin', type=float, nargs=3, help='Disaster fleet XYZ; independent of Carter spawn')
    parser.add_argument('--carter-usd', help='Override the NVIDIA navigation sample asset path/URL')
    parser.add_argument('--start', type=float, nargs=3, help='Robot start XYZ in scene coordinates')
    parser.add_argument('--target', type=float, nargs=3, help='Target XYZ relative to Carter nav root')
    parser.add_argument('--collision-ground', action='store_true', help='Add an invisible floor at spawn height')
    parser.add_argument('--output', type=Path, help='Export an editable .usda wrapper')
    parser.add_argument('--preview', type=Path, help='Capture initial viewport PNG')
    args = parser.parse_args()
    import math
    if (args.camera_eye is None)!=(args.camera_target is None):
        parser.error('--camera-eye and --camera-target must be supplied together')
    if args.camera_eye is not None:
        if not all(math.isfinite(v) for v in args.camera_eye+args.camera_target) or args.camera_eye==args.camera_target:
            parser.error('Camera eye/target must be finite and distinct')
        args.camera_view='manual'
    if not math.isfinite(args.camera_dwell) or args.camera_dwell<=0 or not math.isfinite(args.explore_first) or args.explore_first<0:
        parser.error('Camera dwell must be positive; explore-first must be nonnegative; both must be finite')
    if not math.isfinite(args.exploration_speed) or args.exploration_speed<=0:
        parser.error('--exploration-speed must be positive and finite')
    if not all(math.isfinite(v) and v>0 for v in (args.camera_height,args.camera_distance)):
        parser.error('Camera height and distance must be positive and finite')
    args.camera_view=args.camera_view or ('chase' if args.navigation else 'manual')
    if args.robot_scale is None:
        args.robot_scale=.15 if args.navigation and args.fleet_mode=='convoy' else .25
    if args.navigation:
        args.disaster_robots=True
        args.scene_only=True
    if args.stop_when_complete and (not args.navigation or args.fleet_mode!='independent'):
        parser.error('--stop-when-complete requires --navigation --fleet-mode independent')
    if args.steps is None:
        args.steps=0 if args.navigation and not args.headless else 500
    if args.asset_file is None:
        args.asset_file='3dgrt/real2sim_galileo.usd' if args.navigation and args.scene=='galileo' else 'stage_particle.usdz'
    if not 0 < args.robot_scale < float('inf'):
        parser.error('--robot-scale must be positive and finite')
    folder, start, target = SCENARIOS[args.scene]
    args.stage = (args.stage or args.dataset_root / folder / args.asset_file).resolve()
    args.start, args.target = args.start or start, args.target or target
    if args.steps < 0 or (args.headless and args.steps == 0):
        parser.error('Use positive --steps for headless execution')
    if not args.stage.is_file():
        parser.error(f'NuRec asset not found: {args.stage}. See isaac_sim/NUREC.md for dataset access/download.')
    if args.output and args.output.suffix != '.usda':
        parser.error('--output must end in .usda')
    if args.output and args.output.resolve() == args.stage:
        parser.error('Output must differ from input stage')
    return args


def run(args, app):
    import asyncio
    import omni.timeline
    import omni.usd
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.storage.native import get_assets_root_path
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics
    from isaacsim.core.version import get_version

    version = get_version()
    if version[2] == '5' and args.stage.name == 'stage_particle.usdz':
        raise RuntimeError(
            'The particle-stage variant failed rendering on this Isaac Sim 5.1 installation. '
            'Use --stage assets/nurec/nova_carter-galileo/3dgrt/real2sim_galileo.usd '
            '(verified locally), or use a compatible newer Isaac Sim runtime.')

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    source = Usd.Stage.Open(str(args.stage))
    if source is None:
        raise RuntimeError(f'Cannot open NuRec asset: {args.stage}')
    # Sublayer the complete scene, preserving all its root prims and absolute
    # relationships. A separate root remains writable even for USDZ inputs.
    stage.GetRootLayer().subLayerPaths.append(str(args.stage))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(source))
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(source))
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z or abs(UsdGeom.GetStageMetersPerUnit(stage)-1) > 1e-6:
        raise RuntimeError('Carter presets require a Z-up scene in metres; convert the asset before loading')
    source = None
    for _ in range(20):
        app.update()
    if not list(stage.Traverse()):
        raise RuntimeError('The referenced scene contains no loaded prims')
    props=None
    if args.scene=='galileo' and not args.no_disaster_props:
        from disaster_props import add_disaster_props
        props=add_disaster_props(stage)

    scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    if not scenes:
        scenes = [UsdPhysics.Scene.Define(stage, '/NuRecPhysics').GetPrim()]
    for prim in scenes:
        PhysxSchema.PhysxSceneAPI.Apply(prim).CreateUpdateTypeAttr('Synchronous')

    if not args.scene_only:
        collisions = [prim for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.CollisionAPI)]
        if not collisions and not args.collision_ground:
            raise RuntimeError('No collision geometry found. Supply a mesh or explicitly use --collision-ground.')
        if stage.GetPrimAtPath('/World/NovaCarterNav'):
            raise RuntimeError('Asset already contains /World/NovaCarterNav')
        asset = args.carter_usd
        if not asset:
            root = get_assets_root_path()
            if not root:
                raise RuntimeError('Cannot locate Isaac assets. Provide --carter-usd or use --scene-only.')
            asset = root + '/Isaac/Samples/Replicator/OmniGraph/nova_carter_nav_only.usd'
        robot = add_reference_to_stage(usd_path=asset, prim_path='/World/NovaCarterNav')
        target = stage.GetPrimAtPath('/World/NovaCarterNav/targetXform')
        if not target.IsValid():
            raise RuntimeError(f'Carter navigation target missing; check asset availability: {asset}')
        for prim, location in ((robot, args.start), (target, args.target)):
            attr = prim.GetAttribute('xformOp:translate')
            if not attr:
                attr = UsdGeom.Xformable(prim).AddTranslateOp().GetAttr()
            attr.Set(Gf.Vec3d(*location))

    if args.collision_ground:
        plane = UsdGeom.Mesh.Define(stage, '/NuRecCollisionGround')
        x, y, z = args.start
        plane.CreatePointsAttr([(x-10,y-10,z), (x+10,y-10,z), (x+10,y+10,z), (x-10,y+10,z)])
        plane.CreateFaceVertexCountsAttr([4])
        plane.CreateFaceVertexIndicesAttr([0,1,2,3])
        plane.CreateVisibilityAttr('invisible')
        UsdPhysics.CollisionAPI.Apply(plane.GetPrim())

    # Use an authored asset camera when available; preserve captured viewpoint.
    viewport = get_active_viewport()
    cameras = [p for p in stage.Traverse() if p.IsA(UsdGeom.Camera)
               and not str(p.GetPath()).startswith('/OmniverseKit')]
    if viewport and cameras:
        viewport.camera_path = str(cameras[0].GetPath())
    catalog = None
    response = None
    convoy = None
    tour = None
    if args.disaster_robots:
        from disaster_fleet import add_disaster_fleet
        from isaacsim.core.utils.viewports import set_camera_view
        fleet_origin = args.fleet_origin or ((-2., 0., 0.) if args.scene == 'galileo' else args.start)
        catalog = add_disaster_fleet(stage, fleet_origin, args.robot_assets_root, args.robot_scale,args.tasks_file)
        catalog['environment_props']=props
        if args.navigation:
            from nurec_navigation import NuRecResponse
            response=NuRecResponse(stage,catalog,args.explore_first)
            if args.fleet_mode=='convoy':
                from convoy import Convoy
                convoy=Convoy(response)
                convoy.speed=args.exploration_speed
                response.sync()
        if viewport:
            viewport.camera_path = '/OmniverseKit_Persp'
            x, y, z = fleet_origin
            set_camera_view(eye=[x+2,y,z+1], target=[x,y,z+.7])
            UsdGeom.Camera.Get(stage, '/OmniverseKit_Persp').GetFocalLengthAttr().Set(12)
        if response and viewport and args.camera_view=='overview':
            from camera_tour import FixedOverviewCamera
            FixedOverviewCamera(stage,response,viewport)
        elif response and viewport and args.camera_view=='tour':
            from camera_tour import CameraTour
            tour=CameraTour(stage,response,viewport,args.camera_dwell)
        elif response and viewport and args.camera_view=='group':
            from camera_tour import GroupCamera
            tour=GroupCamera(stage,response,viewport)
        elif response and viewport and args.camera_view=='rear':
            from camera_tour import RearCamera
            tour=RearCamera(stage,response,viewport)
        elif response and viewport and args.camera_view=='chase':
            from camera_tour import ChaseCamera
            tour=ChaseCamera(stage,response,viewport,args.follow_robot,args.camera_height,args.camera_distance)
    if viewport and args.camera_eye is not None:
        from isaacsim.core.utils.viewports import set_camera_view
        viewport.camera_path='/OmniverseKit_Persp'
        set_camera_view(eye=args.camera_eye,target=args.camera_target)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not stage.GetRootLayer().Export(str(args.output.resolve())):
            raise RuntimeError('Wrapper export failed')
        if catalog:
            import json
            args.output.with_suffix('.tasks.json').write_text(json.dumps(catalog, indent=2)+'\n', encoding='utf-8')
    if args.preview:
        if viewport is None:
            raise RuntimeError('No viewport for preview')
        args.preview.parent.mkdir(parents=True, exist_ok=True)
        for _ in range(40):
            app.update()
        capture = capture_viewport_to_file(viewport, str(args.preview.resolve()))
        pending = asyncio.ensure_future(capture.wait_for_result())
        for _ in range(300):
            app.update()
            if pending.done():
                pending.result()
                break
        else:
            raise RuntimeError('Preview capture timed out')
    timeline = omni.timeline.get_timeline_interface()
    panel=None
    if response and not args.headless:
        from task_panel import TaskPanel
        panel=TaskPanel(response)
    last_status={}
    timeline.play()
    previous_time=timeline.get_current_time()
    next_sync=previous_time+1
    frame = 0
    try:
        while app.is_running() and (not args.steps or frame < args.steps):
            app.update()
            now=timeline.get_current_time()
            if response and timeline.is_playing():
                dt=max(0,min(now-previous_time,.1))
                if dt:
                    if convoy:
                        convoy.step(dt)
                    else:
                        response.step(dt)
                    if tour:
                        tour.update(dt)
                if now>=next_sync:
                    response.sync()
                    next_sync=now+1
                if panel:
                    panel.update()
                for task in response.tasks:
                    state=(task.status,task.assigned_robot)
                    if last_status.get(task.task_id)!=state:
                        print(f'Task {task.task_id} ({task.task_type}): {task.status}; robot={task.assigned_robot}',flush=True)
                        last_status[task.task_id]=state
                if args.stop_when_complete and response.tasks and all(t.status=='completed' for t in response.tasks):
                    print('ALL TASKS COMPLETED (symbolic simulation)',flush=True)
                    frame+=1
                    break
            previous_time=now
            frame += 1
    finally:
        timeline.stop()
    if response:
        import json
        response.sync()
        if args.output:
            args.output.with_suffix('.state.json').write_text(json.dumps(catalog,indent=2)+'\n',encoding='utf-8')
        print('Response state: '+json.dumps([{'id':r['robot_id'],'position':[r['x'],r['y'],r['z']],
               'task':r['current_task'],'navigation':r['navigation_status']} for r in catalog['robots']]),flush=True)
    print(f'Completed {frame} updates: {args.stage}', flush=True)


if __name__ == '__main__':
    options = parse_args()
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({'headless': options.headless,
                                    'width': 640 if options.headless else 1280,
                                    'height': 480 if options.headless else 720})
    try:
        run(options, simulation_app)
    finally:
        simulation_app.close()
