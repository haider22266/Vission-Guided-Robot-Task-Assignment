"""Five official Franka arms on mobile display bases sort environment objects.

Open-front editable warehouse, one fixed camera, exclusive object and rack
reservations, Lula IK/FK arm motion, and bilateral geometric grasp verification.
Kinematic carrying is allowed only after the visible contact pads touch the
object. This is not force-validated dynamics or a real hardware controller.
"""
import argparse
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--steps',type=int,default=0)
    parser.add_argument('--stop-after',type=int,default=0,help='Stop after this many completed rack placements')
    parser.add_argument('--robot-assets-root')
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'scenes'/'five_robot_pick_place.usda')
    parser.add_argument('--preview',type=Path)
    args=parser.parse_args()
    if args.steps<0 or args.stop_after<0 or (args.headless and not args.steps and not args.stop_after):
        parser.error('Use nonnegative limits and a finite --steps or --stop-after in headless mode')
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':args.headless,'width':960,'height':640,'fast_shutdown':True})
    try:
        import asyncio
        import omni.timeline
        import omni.usd
        from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
        from asset_robots import resolve_assets_root
        from pick_place_scene import build_scene
        from pick_place_tasks import TaskBroker
        from pick_place_robot import Picker
        context=omni.usd.get_context()
        context.new_stage()
        stage=context.get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage,1.)
        physics=UsdPhysics.Scene.Define(stage,'/World/Physics')
        PhysxSchema.PhysxSceneAPI.Apply(physics.GetPrim()).CreateEnableGPUDynamicsAttr(False)
        items,slots,data=build_scene(stage)
        broker=TaskBroker(items,slots)
        asset_root=resolve_assets_root(args.robot_assets_root)
        robots=[Picker(stage,f'robot_{i+1}',data[f'robot_{i+1}'],data,broker,asset_root) for i in range(5)]
        camera=UsdGeom.Camera.Define(stage,'/World/FixedCamera')
        camera.CreateFocalLengthAttr(22)
        camera.CreateClippingRangeAttr((.05,100.))
        viewport=get_active_viewport()
        if viewport:
            viewport.camera_path='/World/FixedCamera'
        # The ONLY camera pose update in this script. No camera code runs in the loop.
        set_camera_view(eye=[5.8,-7.5,5.3],target=[.2,.1,.55],camera_prim_path='/World/FixedCamera')
        fixed_camera=UsdGeom.Xformable(camera).GetLocalTransformation()
        labels=[]
        window=None
        if not args.headless:
            import omni.ui as ui
            window=ui.Window('Five robots: object to rack',width=510,height=360)
            with window.frame:
                with ui.VStack(spacing=8):
                    summary=ui.Label('Starting continuous sorting',height=28)
                    labels=[ui.Label('',height=48,word_wrap=True) for _ in robots]
                    ui.Label('Kinematic IK/FK. Pickup requires verified finger-pad contact.\nFixed camera; no front wall. Objects are grasp-sized demo props.',height=42,word_wrap=True)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        stage.GetRootLayer().Export(str(args.output.resolve()))
        timeline=omni.timeline.get_timeline_interface()
        timeline.play()
        frames=0
        while app.is_running() and (not args.steps or frames<args.steps):
            app.update()
            if not timeline.is_playing():
                continue
            for robot in robots:
                robot.step(1/60)
            if labels:
                summary.text=f'Rack placements completed: {len(broker.history)}'
                for label,robot in zip(labels,robots):
                    task=robot.task
                    label.text=(f'{robot.id} | {robot.info["kind"]} | {robot.phase}\n'
                                f'Object: {task["object"] if task else "waiting"} | '
                                f'Rack: {task["slot"] if task else "--"} | Done: {robot.completed}')
            frames+=1
            if args.stop_after and len(broker.history)>=args.stop_after:
                break
        timeline.stop()
        if args.preview and viewport:
            args.preview.parent.mkdir(parents=True,exist_ok=True)
            for _ in range(20):
                app.update()
            capture=capture_viewport_to_file(viewport,str(args.preview.resolve()))
            pending=asyncio.ensure_future(capture.wait_for_result())
            for _ in range(300):
                app.update()
                if pending.done():
                    pending.result()
                    break
        camera_unchanged=UsdGeom.Xformable(camera).GetLocalTransformation()==fixed_camera
        result={'mode':'kinematic geometric-contact-gated manipulation','frames':frames,
                'camera_unchanged':camera_unchanged,'front_wall_present':False,
                'completed':broker.history,'active':broker.active,'occupied_slots':broker.occupied,
                'robots':[{'id':r.id,'phase':r.phase,'completed':r.completed,'error':r.last_error} for r in robots]}
        args.output.with_suffix('.results.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        stage.GetRootLayer().Export(str(args.output.resolve()))
        print('RESULT: '+json.dumps(result),flush=True)
        if args.headless and args.stop_after and len(broker.history)<args.stop_after:
            raise RuntimeError('Requested placements did not complete; inspect .results.json. No pickup success was fabricated.')
    except Exception:
        import traceback
        traceback.print_exc()
        app.app.post_quit(1)
        raise
    finally:
        app.close()


if __name__=='__main__':
    main()
