"""Recover existing warehouse disaster props to existing shelves.

Existing disaster props and rack pallets, A* routes, planar base drives,
physics arm joints, and contact-verified assisted surface grasps. All selected
robots start together on one simulation clock; shared aisles use reservations.
The camera stays fixed. The source USD is never saved.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import traceback
import os
BUNDLED_SOURCE=Path(__file__).with_name('warehouse_source')
SOURCE_DIR=Path(os.environ.get('WAREHOUSE_SOURCE_DIR',str(BUNDLED_SOURCE) if BUNDLED_SOURCE.is_dir()
                            else r'E:\ICAST\Research Project\PhysicalAiRoleSwitchin_IsaacSim'))
sys.path.insert(0,str(SOURCE_DIR))
from warehouse_navigation import FloorPlanner


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene', type=Path, default=SOURCE_DIR/'warehouse_verified.usd')
    p.add_argument('--perception-backend',choices=('off','mock','efficientnet'),default='off')
    p.add_argument('--fast-cleanup',action='store_true',help='Claim the first feasible checked task instead of exhaustively ranking grasp candidates')
    p.add_argument('--efficientnet-model',type=Path,default=Path(__file__).resolve().parents[1]/'models/cdd/efficientnet_cdd.keras')
    p.add_argument('--classifier-python',default=sys.executable,help='Python environment containing TensorFlow')
    p.add_argument('--classifier-wsl',action='store_true')
    p.add_argument('--classifier-distro',default='Ubuntu-22.04')
    p.add_argument('--headless', action='store_true')
    p.add_argument('--duration', type=float, default=0, help='Simulated time limit; 0 means until cleanup stops (GUI only)')
    p.add_argument('--speed', type=float, default=.8, help='Base speed in meters per second')
    p.add_argument('--camera', choices=('fixed',), default='fixed', help='Fixed perspective view; no camera tracking')
    p.add_argument('--robot',type=int,choices=range(1,6),help='Run just one robot for review')
    p.add_argument('--only-object',help='Diagnostic: attempt only this exact original prop name; not full floor cleanup')
    p.add_argument('--inject-pickup-failure',action='store_true',help=argparse.SUPPRESS)
    p.add_argument('--tasks-per-robot',type=int,default=0,help='0: shared continuous floor cleanup; 1 or 2: original bounded role demo')
    p.add_argument('--serial-cleanup',action='store_true',help='Optional one-at-a-time debugging mode; default is concurrent')
    p.add_argument('--exit-on-complete', action='store_true')
    p.add_argument('--report', type=Path, default=Path(__file__).with_name('scenes')/'environment_cdd_report.json')
    p.add_argument('--inspect-racks', action='store_true', help=argparse.SUPPRESS)
    p.add_argument('--screenshot', type=Path)
    args = p.parse_args()
    if args.perception_backend=='efficientnet' and (not args.efficientnet_model or not args.efficientnet_model.is_file()):
        p.error('EfficientNet mode requires --efficientnet-model pointing to trained CDD weights')
    if not args.scene.is_file():
        p.error('Scene not found: ' + str(args.scene))
    if not math.isfinite(args.duration) or args.duration<0 or not 0 < args.speed <= 1.5 or args.tasks_per_robot not in (0,1,2):
        p.error('Use nonnegative finite --duration, tasks-per-robot 0/1/2, and 0 < --speed <= 1.5')
    if args.headless and args.duration==0:p.error('Headless execution needs a positive --duration')
    args.duration=args.duration or math.inf
    return args


def main():
    args = arguments()
    sys.stdout.reconfigure(line_buffering=True)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': args.headless, 'width': 1600, 'height': 900})
    research=None
    fault_injection={'used':False}
    try:
        import numpy as np
        import omni.usd
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, SingleRigidPrim, RigidPrim
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.utils.viewports import set_camera_view
        from isaacsim.robot_motion.motion_generation import (
            ArticulationKinematicsSolver, LulaKinematicsSolver, interface_config_loader,
        )
        context = omni.usd.get_context()
        if context.open_stage(str(args.scene.resolve())) is False:
            raise RuntimeError('Cannot open ' + str(args.scene))
        for _ in range(1800):
            app.update()
            stage = context.get_stage()
            if stage and stage.GetPrimAtPath('/World/Robots/Robot_05') and context.get_stage_loading_status()[2] == 0:
                break
        else:
            raise RuntimeError('Five-robot scene loading timed out')
        if UsdGeom.GetStageUpAxis(stage) != 'Z' or not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1):
            raise RuntimeError('Expected the generated Z-up, meter-unit scene')
        stage.SetEditTarget(stage.GetSessionLayer())
        from warehouse_scene_utils import remove_front_walls, remove_floating_hazard_signs
        remove_floating_hazard_signs(stage)
        removed_walls=remove_front_walls(stage)
        print('Removed front wall assemblies for this session:',len(removed_walls))
        task_root = '/World/RobotTasks'
        UsdGeom.Xform.Define(stage, task_root)
        from omni.kit.viewport.utility import get_active_viewport,get_active_viewport_window
        viewport=get_active_viewport()
        if viewport is not None:viewport.camera_path='/OmniverseKit_Persp'
        viewport_window=get_active_viewport_window()
        if viewport_window is not None and not args.headless:viewport_window.visible=True
        set_camera_view(eye=[0, -23, 7.5], target=[0, -1, .5])
        camera = UsdGeom.Camera(stage.GetPrimAtPath('/OmniverseKit_Persp'))
        camera.GetFocalLengthAttr().Set(28.)
        camera.GetHorizontalApertureAttr().Set(36.)
        camera.GetVerticalApertureAttr().Set(20.25)

        live, payload_bounds = {}, {}
        pose_cache,bounds_cache={},{}

        def matrix(prim):
            path=str(prim.GetPath())
            if path in live:
                if path not in pose_cache:
                    p,q=live[path].get_world_pose()
                    pose_cache[path]=Gf.Matrix4d().SetRotate(Gf.Quatd(float(q[0]),Gf.Vec3d(*[float(v) for v in q[1:]]))).SetTranslateOnly(Gf.Vec3d(*[float(v) for v in p]))
                return pose_cache[path]
            return UsdGeom.XformCache().GetLocalToWorldTransform(prim).RemoveScaleShear()

        def pose(prim):
            m = matrix(prim)
            q = m.ExtractRotationQuat()
            return np.array(m.ExtractTranslation()), np.array([q.GetReal(), *q.GetImaginary()])

        def bounds(prims):
            key=tuple(str(p.GetPath()) for p in prims)
            if key in bounds_cache:
                return bounds_cache[key]
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render', 'proxy'])
            lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
            for prim in prims:
                path=str(prim.GetPath())
                if path in payload_bounds and path in live:
                    lower,upper=payload_bounds[path]
                    m=matrix(prim)
                    corners=[m.Transform(Gf.Vec3d(x,y,z)) for x in (lower[0],upper[0]) for y in (lower[1],upper[1]) for z in (lower[2],upper[2])]
                    lo,hi=np.minimum(lo,np.min(corners,axis=0)),np.maximum(hi,np.max(corners,axis=0))
                    continue
                b = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                lo, hi = np.minimum(lo, b.GetMin()), np.maximum(hi, b.GetMax())
            bounds_cache[key]=(lo,hi)
            return lo, hi

        def attach(path, body, hand):
            joint = UsdPhysics.FixedJoint.Define(stage, path)
            local = matrix(body) * matrix(hand).GetInverse()
            q = local.ExtractRotationQuat()
            joint.CreateBody0Rel().SetTargets([hand.GetPath()])
            joint.CreateBody1Rel().SetTargets([body.GetPath()])
            joint.CreateLocalPos0Attr(Gf.Vec3f(*local.ExtractTranslation()))
            joint.CreateLocalRot0Attr(Gf.Quatf(q.GetReal(), Gf.Vec3f(*q.GetImaginary())))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0))
            joint.CreateLocalRot1Attr(Gf.Quatf(1))
            joint.CreateCollisionEnabledAttr(False)

        from warehouse_environment_jobs import existing_prop_jobs
        environment_jobs=existing_prop_jobs(stage)
        from floor_cleanup import floor_groups,CleanupQueue,FairTripGate,RouteReservations,beam_fits_support,route_prefix,upper_face,supported_centres,TrafficDeferred,idle_clear_due
        route_reservations=RouteReservations()
        concurrent_cleanup=args.tasks_per_robot==0 and not args.serial_cleanup
        motion_stats={'peak_moving_robots':0,'overlap_frames':0}
        cleanup_groups,unknown_components=floor_groups(list(stage.GetPrimAtPath('/World/DisasterAftermath/Props').GetChildren()))
        cleanup=CleanupQueue(cleanup_groups,unknown_components) if args.tasks_per_robot==0 else None
        cleanup_by_id={g['id']:g for g in cleanup_groups}
        cleanup_pallets=[]
        tasks = [job['task'] for job in environment_jobs]
        records = []
        for i, name in enumerate(tasks, 1):
            path = '/World/Robots/Robot_%02d/RidgebackFranka' % i
            model = stage.GetPrimAtPath(path)
            if not model:
                raise RuntimeError('Expected mobile manipulator at ' + path)
            roots = [p for p in Usd.PrimRange(model) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
            if len(roots) != 1:
                raise RuntimeError('Expected one articulation: ' + path)
            robot = SingleArticulation(str(roots[0].GetPath()), name='mobile_worker_%d' % i)
            base = stage.GetPrimAtPath(path+'/base_link')
            # The asset's virtual planar chain starts at a very light floating
            # "world" link. Anchor that link, not base_link: otherwise the
            # drives move the dummy links while the heavy base stays still.
            virtual_world=stage.GetPrimAtPath(path+'/world')
            anchor=UsdPhysics.FixedJoint.Define(stage,task_root+'/PlanarWorldAnchor_%02d'%i)
            wp,wq=pose(virtual_world)
            anchor.CreateBody1Rel().SetTargets([virtual_world.GetPath()])
            anchor.CreateLocalPos0Attr(Gf.Vec3f(*wp))
            anchor.CreateLocalRot0Attr(Gf.Quatf(float(wq[0]),Gf.Vec3f(*wq[1:])))
            anchor.CreateLocalPos1Attr(Gf.Vec3f(0))
            anchor.CreateLocalRot1Attr(Gf.Quatf(1))
            records.append(dict(id=i, task=name, robot=robot, model=model, base=base,
                                shoulder=stage.GetPrimAtPath(path+'/panda_link0'),
                                hand=stage.GetPrimAtPath(path+'/panda_hand'), home=pose(base)[0],
                                state='waiting', success=False, failure=None, transitions=[], route=[],
                                distance=0., item=None, originals=[], inspection=[], yaw=0.))
        props = stage.GetPrimAtPath('/World/DisasterAftermath/Props')
        contact_paths=[str(p.GetPath()) for p in props.GetChildren() if p.HasAPI(UsdPhysics.CollisionAPI)]
        ui_labels={}
        ui_cleanup_summary=None
        if not args.headless:
            import omni.ui as ui
            task_window=ui.Window('Existing warehouse object tasks',width=700,height=370)
            with task_window.frame:
                with ui.VStack(spacing=8):
                    ui_cleanup_summary=ui.Label('Continuous floor cleanup | verified grasp and rack placement' if cleanup is not None
                                                else 'Existing props | contact-verified assisted grasp',height=25)
                    for r in records:
                        ui_labels[r['id']]=ui.Label('Robot %02d | %s | waiting'%(r['id'],r['task']),height=52,word_wrap=True)
            viewport_ui=ui.Workspace.get_window('Viewport')
            if viewport_ui is not None:task_window.dock_in(viewport_ui,ui.DockPosition.BOTTOM,.28)
        for r in records:
            r['finger_prims']=[stage.GetPrimAtPath(str(r['model'].GetPath())+'/panda_'+side+'finger') for side in ('left','right')]
            r['contacts']=RigidPrim([str(p.GetPath()) for p in r['finger_prims']],name='source_contacts_%d'%r['id'],
                                    contact_filter_prim_paths_expr=[contact_paths,contact_paths],track_contact_forces=True)
        # Give the ORIGINAL pallet meshes physical support, without adding racks.
        for shelf, pallet_name in ((0,'357'),(0,'12'),(1,'357'),(2,'12'),(2,'357'),
                                   (0,'5'),(0,'318'),(1,'12'),(2,'5'),(1,'5'),(0,'319')):
            pallet=stage.GetPrimAtPath('/Root/Shelf_%d/SM_PaletteA_%s/SM_PaletteA_01'%(shelf,pallet_name))
            if not pallet:raise RuntimeError('Missing original destination pallet')
            UsdPhysics.CollisionAPI.Apply(pallet).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(pallet).CreateApproximationAttr('convexHull')
        if cleanup is not None:
            for prim in stage.Traverse():
                path=str(prim.GetPath())
                if path.startswith('/Root/Shelf_') and prim.GetName()=='SM_PaletteA_01' and prim.IsA(UsdGeom.Mesh):
                    lo,hi=bounds([prim])
                    if lo[2]<.5 and hi[2]<.65:
                        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
                        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
                        cleanup_pallets.append({'path':path,'lo':lo.copy(),'hi':hi.copy()})
                if path.startswith('/Root/') and prim.IsA(UsdGeom.Mesh) and any(s in path for s in ('CardBox','KLT','Crate')):
                    lo,hi=bounds([prim])
                    if lo[2]<1.3 and hi[2]>.45:
                        cleanup.static_occupancy.append((float(lo[0]),float(lo[1]),float(hi[0]),float(hi[1])))
            print('CONTINUOUS CLEANUP:',len(cleanup_groups),'objects;',len(cleanup_pallets),'original low rack pallets;',
                  'unregistered components:',unknown_components,flush=True)
        world = World(physics_dt=1/60, rendering_dt=1/60, stage_units_in_meters=1)
        # Ensure USD queries see current physics poses, including attachment frames.
        physics = world.get_physics_context()
        physics.enable_fabric(False)
        physics.set_physx_update_transformations_settings(update_to_usd=True)
        for r in records:
            world.scene.add(r['robot'])
            world.scene.add(r['contacts'])
        world.reset()
        for r in records:
            for prim in (r['base'],r['shoulder'],r['hand'],*r['finger_prims']):
                rigid=SingleRigidPrim(str(prim.GetPath()))
                rigid.initialize()
                live[str(prim.GetPath())]=rigid
            r['finger_vertices']=[]
            for finger in r['finger_prims']:
                vertices=[]
                inverse=UsdGeom.XformCache().GetLocalToWorldTransform(finger).GetInverse()
                for mesh in Usd.PrimRange(finger):
                    if mesh.IsA(UsdGeom.Mesh) and '/visuals/' in str(mesh.GetPath()):
                        transform=UsdGeom.XformCache().GetLocalToWorldTransform(mesh)*inverse
                        vertices.extend([list(transform.Transform(Gf.Vec3d(*p)))+[1.] for p in UsdGeom.Mesh(mesh).GetPointsAttr().Get()])
                r['finger_vertices'].append(np.array(vertices))
        config = interface_config_loader.load_supported_lula_kinematics_solver_config('Franka')
        folded = np.array([0., -.7, 0., -2.35, 0., 1.65, .78])
        for r in records:
            robot = r['robot']
            arm = np.array([robot.get_dof_index('panda_joint%d' % j) for j in range(1,8)])
            fingers = np.array([robot.get_dof_index('panda_finger_joint%d' % j) for j in (1,2)])
            planar = np.array([robot.get_dof_index(n) for n in ('dummy_base_prismatic_x_joint',
                              'dummy_base_prismatic_y_joint', 'dummy_base_revolute_z_joint')])
            robot.set_joint_positions(folded, joint_indices=arm)
            robot.apply_action(ArticulationAction(joint_positions=folded, joint_indices=arm))
            robot.apply_action(ArticulationAction(joint_positions=np.array([.04,.04]), joint_indices=fingers))
            robot.set_solver_position_iteration_count(32)
            solver = LulaKinematicsSolver(**config)
            r.update(arm=arm, fingers=fingers, planar=planar, solver=solver,
                     ik=ArticulationKinematicsSolver(robot,solver,'right_gripper'))
        for _ in range(60):
            world.step(render=True)
        pose_cache.clear()
        bounds_cache.clear()
        for r in records:
            r['home'], _ = pose(r['base'])
            r['shoulder_z'] = pose(r['shoulder'])[0][2]
            r['origin_q'] = r['robot'].get_joint_positions()[r['planar']].copy()
        clock = [0]

        def tick():
            if not app.is_running():
                raise RuntimeError('Simulator closed')
            while app.is_running() and not world.is_playing():
                app.update()
            if clock[0]/60 >= args.duration:
                raise TimeoutError('Maximum simulated duration exceeded')
            render_interval=(16 if args.headless else 6) if args.perception_backend=='off' else (8 if args.headless else 3)
            render = clock[0] % render_interval == 0
            world.step(render=render)
            if research is not None and render:
                research.update(clock[0]/60)
            pose_cache.clear()
            bounds_cache.clear()
            clock[0] += 1

        def event(r, name):
            r['state'] = name
            p, _ = pose(r['base'])
            r['transitions'].append(dict(state=name,time=round(clock[0]/60,3),base_position=p.tolist()))
            if r['id'] in ui_labels:
                ui_labels[r['id']].text='Robot %02d | %s | %s | %s'%(r['id'],r['task'],', '.join(p.rsplit('/',1)[-1] for p in r.get('selected_props',[])),name)
            print('[%6.1fs] Robot_%02d %s: %s' % (clock[0]/60,r['id'],r['task'],name))

        props = stage.GetPrimAtPath('/World/DisasterAftermath/Props')
        shelves = [p for p in stage.Traverse() if p.GetName().startswith('Shelf_') and p.IsA(UsdGeom.Xform)]
        if args.inspect_racks:
            for shelf in shelves:
                print('SHELF',shelf.GetPath(),bounds([shelf]))
                count=0
                for p in Usd.PrimRange(shelf):
                    if p.IsA(UsdGeom.Mesh):
                        lo,hi=bounds([p])
                        if lo[2]<.5 and 'Palette' in str(p.GetPath()):
                            print('LOW MESH',p.GetPath(),lo,hi)
                            count+=1
            return 0

        def obstacles(r):
            result = [bounds([p]) for p in shelves]+[bounds([p]) for p in stored_props if p!=r['item']]
            for prim in props.GetChildren():
                if prim in r['originals'] and r['item']:
                    continue
                if not prim.GetName().startswith(('Debris_', 'Fallen_', 'Rear_', 'Tool_')):
                    continue
                if '_Tape_' in prim.GetName():
                    continue
                if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
                    continue
                lo, hi = bounds([prim])
                if hi[2] > .04:
                    result.append((lo,hi))
            # Moving peers are handled by per-frame local avoidance. Treating
            # a passing peer as a permanent A* wall can trap both workers.
            return result

        def navigation_radius(r):
            radius=.70
            if r.get('carrying') and r.get('item'):
                lo,hi=bounds([r['item']])
                p=pose(r['base'])[0]
                radius=max(radius,max(float(np.linalg.norm(np.array([x,y])-p[:2]))
                                      for x in (lo[0],hi[0]) for y in (lo[1],hi[1]))+.06)
            return radius

        def navigation_footprint(r):
            footprint=[-.70,-.70,.70,.70]
            if r.get('carrying') and r.get('item'):
                lo,hi=bounds([r['item']]);p=pose(r['base'])[0]
                footprint=[min(footprint[0],float(lo[0]-p[0]-.06)),min(footprint[1],float(lo[1]-p[1]-.06)),
                           max(footprint[2],float(hi[0]-p[0]+.06)),max(footprint[3],float(hi[1]-p[1]+.06))]
            return footprint

        def planner(r):
            rects = []
            bp, _ = pose(r['base'])
            carried = bounds([r['item']]) if r.get('carrying') else None
            for lo,hi in obstacles(r):
                rects.append((lo[0]-.64,hi[0]+.64,lo[1]-.64,hi[1]+.64))
                # The procedural source piles already interpenetrate. Permit
                # the load to withdraw from that small original pile; the
                # mobile base still reserves full obstacle clearance there.
                source_cluster = (r.get('item_start') is not None and hi[2]<1.1
                                  and np.linalg.norm(((lo+hi)/2)[:2]-np.array(r['item_start'])[:2])<2.)
                target_rack=False
                if r.get('destination_shelf'):
                    destination_lo,destination_hi=bounds([stage.GetPrimAtPath(r['destination_shelf'])])
                    target_rack=np.allclose(lo,destination_lo) and np.allclose(hi,destination_hi)
                if carried and hi[2] > carried[0][2]+.02 and not source_cluster and not target_rack:
                    clo, chi = carried
                    rects.append((lo[0]-(chi[0]-bp[0])-.06,hi[0]-(clo[0]-bp[0])+.06,
                                  lo[1]-(chi[1]-bp[1])-.06,hi[1]-(clo[1]-bp[1])+.06))
            for other in records:
                if other is r:continue
                parked=(args.robot and other['id']!=args.robot) or other['state'] in (
                    'all_assigned_environment_tasks_complete','idle_no_more_reachable_original_props','cleanup_waiting',
                    'yield_cleanup_aisle_turn','yield_rear_pickup_zone_to_other_robot','yield_shared_left_rack_corridor','failed')
                if concurrent_cleanup and not other.get('nav_active',False):parked=True
                if not parked:continue
                p,_=pose(other['base'])
                if concurrent_cleanup:
                    footprint=navigation_footprint(r)
                    rects.append((p[0]-.70-footprint[2],p[0]+.70-footprint[0],p[1]-.70-footprint[3],p[1]+.70-footprint[1]))
                    if other.get('carrying') and other.get('item'):
                        olo,ohi=bounds([other['item']])
                        rects.append((olo[0]-footprint[2]-.06,ohi[0]-footprint[0]+.06,olo[1]-footprint[3]-.06,ohi[1]-footprint[1]+.06))
                    continue
                lo,hi=p[:2]-[.48,.42],p[:2]+[.48,.42]
                # Parked peers are static route obstacles. Match local traffic
                # clearance, with a sub-millimetre boundary tolerance.
                rects.append((lo[0]-.5995,hi[0]+.5995,lo[1]-.5995,hi[1]+.5995))
                if carried:
                    clo,chi=carried
                    rects.append((lo[0]-(chi[0]-bp[0])-.06,hi[0]-(clo[0]-bp[0])+.06,
                                  lo[1]-(chi[1]-bp[1])-.06,hi[1]-(clo[1]-bp[1])+.06))
            return FloorPlanner(rects,radius=0.)

        def drive(r, xy, yaw):
            q = r['origin_q']+np.array([xy[0]-r['home'][0],xy[1]-r['home'][1],yaw])
            r['robot'].apply_action(ArticulationAction(joint_positions=q,joint_indices=r['planar']))

        def traffic_overlap(r, command):
            """Keep moving bases/loads clear while independent routes overlap."""
            current,_=pose(r['base'])
            # Moving peers retain 12 cm beyond the actual 48 cm half-width.
            # Static route planning keeps its larger 64 cm clearance. This
            # avoids trapping two peers on opposite safety-margin boundaries.
            footprints=[(np.array(command)-.60,np.array(command)+.60)]
            if r.get('carrying'):
                lo,hi=bounds([r['item']])
                shift=np.array(command)-current[:2]
                footprints.append((lo[:2]+shift-.06,hi[:2]+shift+.06))
            overlap=0.
            for other in records:
                if other is r:
                    continue
                p,_=pose(other['base'])
                occupied=[(p[:2]-[.48,.42],p[:2]+[.48,.42])]
                if other['item']:
                    lo,hi=bounds([other['item']])
                    occupied.append((lo[:2],hi[:2]))
                for a,b in footprints:
                    for c,d in occupied:
                        overlap+=float(np.prod(np.maximum(0.,np.minimum(b,d)-np.maximum(a,c))))
            return overlap

        def traffic_clear(r,command):
            return traffic_overlap(r,command)<1e-4

        def clear_initial_traffic_margin(r):
            # A newly lifted long prop can overlap another worker's conservative
            # safety margin. Move the BASE out of that margin before replanning.
            # This never changes the prop pose or ignores a physical collision.
            for _ in range(12):
                current=pose(r['base'])[0][:2]
                overlap=traffic_overlap(r,current)
                if overlap<1e-4:return
                nav=planner(r)
                candidates=[current+np.array([dx,dy]) for dx,dy in
                            ((.06,0),(-.06,0),(0,.06),(0,-.06),(.06,.06),(-.06,.06),(.06,-.06),(-.06,-.06))]
                candidates=[p for p in candidates if nav.free(p) and traffic_overlap(r,p)<overlap*.8]
                if not candidates:return
                command=min(candidates,key=lambda p:traffic_overlap(r,p))
                for _ in range(12):
                    drive(r,command,r['yaw'])
                    yield

        def reserve_navigation_route(r,route,goal,partial=False):
            route_reservations.release(r['id'],clock[0]/60)
            r['nav_active']=False
            original_state=r['state']
            waiting=False
            for attempt in range(240 if r.get('carrying') else 48):
                positions={other['id']:(pose(other['base'])[0][:2],.70) for other in records}
                occupied=[]
                for other in records:
                    if other is not r and other.get('carrying') and other.get('item'):
                        lo,hi=bounds([other['item']]);occupied.append((float(lo[0]-.06),float(lo[1]-.06),float(hi[0]+.06),float(hi[1]+.06)))
                candidate=route_prefix(route,(2.,1.2,.8,.5)[attempt%4]) if partial else route
                if route_reservations.reserve(r['id'],candidate,navigation_radius(r),positions,clock[0]/60,
                    footprint=navigation_footprint(r),occupied_boxes=occupied,
                    goal_radius=navigation_radius(r) if r.get('reservation_rotate') else None):
                    r['nav_active']=True
                    if waiting:event(r,original_state)
                    return candidate
                if not waiting:
                    event(r,'yield_conflicting_route')
                    waiting=True
                if attempt and attempt%16==0:
                    if (yield from backoff_route(r)):
                        try:route=planner(r).plan(pose(r['base'])[0][:2],goal)
                        except ValueError:pass
                drive(r,pose(r['base'])[0][:2],r['yaw'])
                for _ in range(15):yield
                if attempt%32==31:
                    try:route=planner(r).plan(pose(r['base'])[0][:2],goal)
                    except ValueError:pass
            raise TrafficDeferred('Traffic remains blocked; release this route and defer the task')

        def backoff_route(r):
            """Move away from a conflict through a short, reserved free route."""
            if r.get('in_route_backoff'):return False
            current=pose(r['base'])[0][:2].copy()
            peers=[pose(other['base'])[0][:2].copy() for other in records if other is not r]
            nearest=min(float(np.linalg.norm(current-p)) for p in peers)
            nav=planner(r)
            candidates=[]
            for distance in (.5,1.,1.5):
                for angle in np.linspace(0,2*math.pi,12,endpoint=False):
                    target=current+distance*np.array([math.cos(angle),math.sin(angle)])
                    separation=min(float(np.linalg.norm(target-p)) for p in peers)
                    if separation>nearest+.15 and nav.clear(current,target) and traffic_clear(r,target):
                        candidates.append((float(np.linalg.norm(target-r['home'][:2])),target))
            for _,target in sorted(candidates,key=lambda item:item[0]):
                positions={other['id']:(pose(other['base'])[0][:2],.70) for other in records}
                path=[current.tolist(),target.tolist()]
                if not route_reservations.reserve(r['id'],path,navigation_radius(r),positions,clock[0]/60,footprint=navigation_footprint(r)):continue
                original_state=r['state']
                event(r,'back_off_to_clear_peer_route')
                r['nav_active']=True
                r['in_route_backoff']=True
                r['reserved_route']=path
                try:
                    yield from navigate_unreserved(r,target)
                    r['route_backoffs']=r.get('route_backoffs',0)+1
                finally:
                    route_reservations.release(r['id'],clock[0]/60)
                    r['nav_active']=False
                    r['in_route_backoff']=False
                    r.pop('reserved_route',None)
                event(r,original_state)
                return True
            return False

        def navigate(r,goal,yaw=None):
            if not concurrent_cleanup:
                yield from navigate_unreserved(r,goal,yaw)
                return
            try:
                if not planner(r).free(pose(r['base'])[0][:2]) and not r.get('carrying'):
                    yield from escape_static_margin(r)
                yield from clear_initial_traffic_margin(r)
                for leg in range(1000):
                    route=None
                    for attempt in range(40):
                        try:
                            route=planner(r).plan(pose(r['base'])[0][:2],goal)
                            break
                        except ValueError:
                            if attempt%8==7:yield from backoff_route(r)
                            drive(r,pose(r['base'])[0][:2],r['yaw'])
                            for _ in range(30):yield
                    if route is None:raise TrafficDeferred('No collision-clear route currently available')
                    prefix=route_prefix(route,2.)
                    final=math.dist(prefix[-1],goal)<.001
                    r['reservation_rotate']=final and yaw is not None and abs((yaw-r['yaw']+math.pi)%(2*math.pi)-math.pi)>.02
                    r['reserved_route']=yield from reserve_navigation_route(r,prefix,prefix[-1],partial=True)
                    leg_goal=r['reserved_route'][-1]
                    final=math.dist(leg_goal,goal)<.001
                    yield from navigate_unreserved(r,leg_goal,yaw if final else None)
                    route_reservations.release(r['id'],clock[0]/60)
                    r['nav_active']=False
                    r.pop('reserved_route',None)
                    if final:return
                raise RuntimeError('Navigation rolling-horizon limit exceeded')
            finally:
                route_reservations.release(r['id'],clock[0]/60)
                r['nav_active']=False
                r.pop('reserved_route',None)

        def escape_static_margin(r):
            """Leave inflated planning margins only through physically clear space."""
            start=pose(r['base'])[0][:2].copy()
            nav=planner(r)
            if nav.free(start):return
            # Actual rotated base half-extents plus 2 cm, rather than the
            # conservative 64 cm planning inflation. Object geometry remains.
            c,s=abs(math.cos(r['yaw'])),abs(math.sin(r['yaw']))
            hx,hy=.48*c+.42*s+.02,.48*s+.42*c+.02
            physical=FloorPlanner([(lo[0]-hx,hi[0]+hx,lo[1]-hy,hi[1]+hy) for lo,hi in obstacles(r)],radius=0.)
            if not physical.free(start):
                raise TrafficDeferred('Base is inside physical obstacle clearance; automatic escape refused')
            positions={other['id']:(pose(other['base'])[0][:2],.70) for other in records}
            escape_attempts=0
            for distance in np.arange(.10,1.61,.10):
                for angle in np.linspace(0,2*math.pi,24,endpoint=False):
                    start=pose(r['base'])[0][:2].copy()
                    goal=start+distance*np.array([math.cos(angle),math.sin(angle)])
                    if not nav.free(goal) or not physical.clear(start,goal):continue
                    if not route_reservations.reserve(r['id'],[start,goal],navigation_radius(r),positions,
                            clock[0]/60,footprint=navigation_footprint(r)):continue
                    escape_attempts+=1
                    event(r,'exit_inflated_margin_with_physical_clearance')
                    r['nav_active']=True
                    stalled=0
                    try:
                        for _ in range(900):
                            current=pose(r['base'])[0][:2].copy()
                            delta=goal-current
                            length=float(np.linalg.norm(delta))
                            if length<.02:
                                cleanup.epoch+=1
                                return
                            command=current+delta/max(length,1e-8)*min(length,.25/60)
                            if not physical.clear(current,command) or not traffic_clear(r,command):
                                break
                            drive(r,command,r['yaw'])
                            yield
                            moved=float(np.linalg.norm(pose(r['base'])[0][:2]-current))
                            r['distance']+=moved
                            stalled=stalled+1 if moved<.00005 else 0
                            if stalled>=30:break
                    finally:
                        route_reservations.release(r['id'],clock[0]/60)
                        r['nav_active']=False
                    if escape_attempts>=12:
                        raise TrafficDeferred('Checked margin exits remain physically blocked')
            raise TrafficDeferred('No physically clear exit from the inflated planning margin')

        def navigate_unreserved(r, goal, yaw=None):
            if not concurrent_cleanup:yield from clear_initial_traffic_margin(r)
            nav = planner(r)
            initial, _ = pose(r['base'])
            if not nav.free(initial[:2]) or not nav.free(goal):
                print('NAV_CLEARANCE',r['id'],'start',initial[:2],nav.free(initial[:2]),'goal',goal,nav.free(goal),
                      'load',bounds([r['item']]) if r['item'] else None)
                print('BLOCKING_RECTS',[rect for rect in nav.rectangles if rect[0]<=goal[0]<=rect[1] and rect[2]<=goal[1]<=rect[3]])
            route = r.get('reserved_route') or nav.plan(initial[:2],goal)
            r['route'].append([list(p) for p in route])
            print('  Route:',len(route),'waypoints; goal:',tuple(round(float(x),2) for x in goal))
            last = initial[:2].copy()
            index,blocked,attempts=1,0,0
            while index<len(route):
                waypoint=route[index]
                for _ in range(6000):
                    current, _ = pose(r['base'])
                    delta = np.array(waypoint)-current[:2]
                    length = float(np.linalg.norm(delta))
                    if length < .025:
                        drive(r,waypoint,r['yaw'])
                        index+=1
                        break
                    speed = min(args.speed,math.sqrt(max(.02,length)))
                    command = current[:2]+delta/max(length,1e-8)*min(length,speed/60)
                    if not traffic_clear(r,command):
                        # Take a collision-clear lateral step instead of all
                        # peers waiting on one another's stationary footprint.
                        overlap=traffic_overlap(r,current[:2])
                        candidates=[]
                        step=min(length,speed/60)
                        for angle in np.linspace(0,2*math.pi,16,endpoint=False):
                            candidate=current[:2]+step*np.array([math.cos(angle),math.sin(angle)])
                            penalty=traffic_overlap(r,candidate)
                            if nav.clear(current[:2],candidate) and (penalty<1e-4 or penalty<overlap-1e-7):
                                right_bias=.003*(delta[0]*math.sin(angle)-delta[1]*math.cos(angle))/max(length,1e-8)
                                candidates.append((0. if penalty<1e-4 else penalty,
                                                   np.linalg.norm(candidate-np.array(waypoint))+right_bias,candidate))
                        if candidates:
                            command=min(candidates,key=lambda c:(c[0],c[1]))[2]
                            drive(r,command,r['yaw'])
                        else:
                            drive(r,current[:2],r['yaw'])
                        blocked+=1
                        r['traffic_wait_frames']=r.get('traffic_wait_frames',0)+1
                        yield
                        # Give lower robot IDs first chance to route around
                        # a conflict; no task waits for another task to finish.
                        if blocked%(20+10*r['id'])==0:
                            yield from clear_initial_traffic_margin(r)
                            current=pose(r['base'])[0]
                            try:
                                replacement=planner(r).plan(current[:2],goal)
                            except ValueError:
                                continue
                            if concurrent_cleanup:
                                replacement=yield from reserve_navigation_route(r,replacement,goal)
                            route=replacement
                            index=1
                            r['route'].append([list(p) for p in route])
                            r['traffic_replans']=r.get('traffic_replans',0)+1
                            break
                        continue
                    blocked=0
                    drive(r,command,r['yaw'])
                    yield
                    actual, _ = pose(r['base'])
                    moved=float(np.linalg.norm(actual[:2]-last))
                    if moved>1e-5:
                        r.setdefault('first_motion_time',clock[0]/60)
                    r['distance'] += moved
                    last = actual[:2].copy()
                else:
                    raise RuntimeError('Base route tracking timeout')
                attempts+=1
                if attempts>2000:
                    raise RuntimeError('Traffic replanning limit exceeded')
            if yaw is not None:
                for _ in range(360):
                    error = (yaw-r['yaw']+math.pi)%(2*math.pi)-math.pi
                    r['yaw'] += max(-.012,min(.012,error))
                    drive(r,goal,r['yaw'])
                    yield
                    if abs(error)<.002:
                        break
            for _ in range(15):
                drive(r,goal,r['yaw'])
                yield
            actual,_ = pose(r['base'])
            if np.linalg.norm(actual[:2]-goal)>.065:
                raise RuntimeError('Base did not reach docking pose')

        def down(yaw):
            return np.array([0.,math.cos(yaw/2),math.sin(yaw/2),0.])

        def arm_to(r,target,seconds=2.,closed=False):
            solver,ik = r['solver'],r['ik']
            solver.set_robot_base_pose(*pose(r['shoulder']))
            start = ik.compute_end_effector_pose()[0]
            for frame in range(int((seconds+4)*60)):
                solver.set_robot_base_pose(*pose(r['shoulder']))
                t = min(1.,frame/(seconds*60))
                blend = t*t*(3-2*t)
                command = start*(1-blend)+np.array(target)*blend
                action,ok = ik.compute_inverse_kinematics(command,down(r['yaw']),position_tolerance=.004)
                if not ok:
                    raise RuntimeError('IK failed in '+r['state'])
                r['robot'].apply_action(action)
                r['robot'].apply_action(ArticulationAction(joint_positions=np.array([.012,.012] if closed else [.04,.04]),joint_indices=r['fingers']))
                yield
                if t==1 and np.linalg.norm(ik.compute_end_effector_pose()[0]-target)<.004:
                    return
            tip=ik.compute_end_effector_pose()[0]
            actual_tip=matrix(r['hand']).Transform(Gf.Vec3d(0,0,.10))
            print('ARM DIAGNOSTIC target',target,'FK',tip,'hand TCP',actual_tip,'shoulder',pose(r['shoulder'])[0])
            raise RuntimeError('Arm tracking timeout in '+r['state']+'; error %.3f m'%np.linalg.norm(tip-target))

        def choose(r,groups):
            current,_ = pose(r['base'])
            nav = planner(r)
            r['selection_nav']=nav
            options = []
            best_cost=math.inf
            r['search_diagnostics']={}
            r['searched_keys']=[]
            groups=sorted(groups,key=lambda g:(
                0 if cleanup is not None and cleanup.items[str(g[0].GetPath())].get('recoveries',0) else 1,
                np.linalg.norm(((sum(bounds(g)))/2)[:2]-current[:2])))
            for group_index,group in enumerate(groups):
                if cleanup is not None and group_index>=6:break
                if group_index and group_index%2==0:yield
                if cleanup is not None and cleanup.items[str(group[0].GetPath())]['state']!='pending':continue
                key=str(group[0].GetPath())
                r['candidate_object_id']=key
                r['searched_keys'].append(key)
                checks=dict(docking_blocked=0,overhead_blocked=0,route_blocked=0,grasp_ik_failed=0,rack_fit_failed=0,lease_changed=0)
                r['search_diagnostics'][key]=checks
                def rejected(stage):
                    checks[stage]+=1
                    r.setdefault('placement_rejections',{})[key]='Search rejection counts: '+json.dumps(checks,sort_keys=True)
                if cleanup is not None:r['candidate_kind']=cleanup_by_id[str(group[0].GetPath())]['kind']
                lo,hi = bounds(group)
                if cleanup is not None and r['candidate_kind']=='beam':
                    size=hi-lo
                    if not any(beam_fits_support((size[0],size[1]),(p['hi'][0]-p['lo'][0],p['hi'][1]-p['lo'][1])) or
                               beam_fits_support((size[1],size[0]),(p['hi'][0]-p['lo'][0],p['hi'][1]-p['lo'][1])) for p in cleanup_pallets):
                        r.setdefault('placement_rejections',{})[str(group[0].GetPath())]='Oversized beam: needs longer storage support; existing low pallets cannot fit its footprint'
                        continue
                middle = (lo+hi)/2
                primary=group[0]
                transform=UsdGeom.XformCache().GetLocalToWorldTransform(primary)
                half=float(UsdGeom.Cube(primary).GetSizeAttr().Get())/2
                face,sign,axes=upper_face([tuple(transform.TransformDir(Gf.Vec3d(*(1. if i==j else 0. for j in range(3))))) for i in range(3)])
                approaches=[]
                for u in (0.,-.30,.30,-.45,.45):
                    for v in (0.,-.30,.30,-.45,.45):
                        local=[0.,0.,0.]
                        local[face]=sign*half
                        local[axes[0]],local[axes[1]]=u*2*half,v*2*half
                        top=np.array(transform.Transform(Gf.Vec3d(*local)))
                        for yaw in (0.,math.pi,math.pi/2,-math.pi/2):
                            direction=np.array([math.cos(yaw),math.sin(yaw)])
                            point=top+np.array([0,0,.013])
                            for stand_off in (.90,.75,1.05):
                                dock=top[:2]-stand_off*direction
                                approaches.append((dock,point,yaw))
                for approach_index,(dock,point,yaw) in enumerate(approaches):
                    if approach_index and approach_index%12==0:yield
                    if cleanup is not None and cleanup.items[key]['state']!='pending':break
                    # A path cannot beat its straight-line lower bound. Avoid
                    # freezing the UI by running A* for thousands of farther
                    # grasp candidates after a nearer reachable task is found.
                    if np.linalg.norm(np.array(dock)-current[:2])>best_cost+1e-8:
                        continue
                    if not nav.free(dock):
                        rejected('docking_blocked')
                        continue
                    if cleanup is not None:
                        import omni.physx
                        query=omni.physx.get_physx_scene_query_interface()
                        overhead=False
                        for dx,dy in ((0,0),(.04,0),(-.04,0),(0,.04),(0,-.04)):
                            hit=query.raycast_closest((float(point[0]+dx),float(point[1]+dy),float(point[2]+.015)),(0.,0.,1.),.12)
                            if hit['hit'] and not any(str(hit.get(k,'')).startswith(str(p.GetPath()))
                                    for k in ('rigidBody','collision') for p in group):
                                overhead=True
                                break
                        if overhead:
                            rejected('overhead_blocked')
                            continue
                    # Reject disconnected approaches before the much more
                    # expensive search through rack IK candidates.
                    try:route=nav.plan(current[:2],dock)
                    except ValueError:
                        rejected('route_blocked')
                        continue
                    direction=np.array([math.cos(yaw),math.sin(yaw)])
                    carry=np.array([*(np.array(dock)+.52*direction),max(.85,point[2]+.15)])
                    shoulder_xy=np.array(dock)+.30*np.array([math.cos(yaw),math.sin(yaw)])
                    r['solver'].set_robot_base_pose(np.array([*shoulder_xy,r['shoulder_z']]),np.array([math.cos(yaw/2),0,0,math.sin(yaw/2)]))
                    if not all(r['solver'].compute_inverse_kinematics('right_gripper',np.array(p),down(yaw))[1]
                               for p in (point,np.array(point)+[0,0,.06],carry)):
                        rejected('grasp_ik_failed')
                        continue
                    if not (yield from rack_ik_available(r,np.array(point),yaw,lo,hi)):
                        rejected('rack_fit_failed')
                        continue
                    # Other cooperative workers may claim the object/slot
                    # during the IK search's yields. Recheck before the fast
                    # return; the caller claims immediately without yielding.
                    if cleanup is not None and (cleanup.items[str(group[0].GetPath())]['state']!='pending'
                            or not cleanup.slot_free(r['candidate_rack_slot']['box'])):
                        rejected('lease_changed')
                        continue
                    cost = sum(math.dist(a,b) for a,b in zip(route,route[1:]))
                    best_cost=min(best_cost,cost)
                    options.append((cost,group,np.array(dock),np.array(point),carry,yaw,
                                    dict(r['candidate_rack_slot']) if cleanup is not None else None))
                    if args.fast_cleanup:
                        return options[-1]
            if cleanup is not None:
                options=[option for option in options if cleanup.items[str(option[1][0].GetPath())]['state']=='pending'
                         and cleanup.slot_free(option[6]['box'])]
            if not options:
                raise RuntimeError('No reachable floor object with a collision-clear docking pose')
            return min(options,key=lambda entry:entry[0])

        def mobilize(r,originals):
            """Transfer existing geometry into one dynamic payload in-session."""
            lo,hi = bounds(originals)
            center = (lo+hi)/2
            path = task_root+'/Payload_%02d_%02d'%(r['id'],r['task_index'])
            payload = UsdGeom.Xform.Define(stage,path)
            payload.AddTranslateOp().Set(Gf.Vec3d(*center))
            for i,original in enumerate(originals):
                destination = Sdf.Path(path+'/Part_%02d'%i)
                original_matrix = UsdGeom.XformCache().GetLocalToWorldTransform(original)
                if not Sdf.CopySpec(stage.GetRootLayer(),original.GetPath(),stage.GetSessionLayer(),destination):
                    raise RuntimeError('Cannot transfer floor prop '+str(original.GetPath()))
                part = stage.GetPrimAtPath(destination)
                relative = original_matrix*Gf.Matrix4d().SetTranslate(Gf.Vec3d(*(-center)))
                UsdGeom.Xformable(part).MakeMatrixXform().Set(relative)
                UsdGeom.Imageable(part).MakeVisible()
                UsdPhysics.CollisionAPI.Apply(part).CreateCollisionEnabledAttr(True)
                UsdGeom.Imageable(original).MakeInvisible()
                if original.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(original).CreateCollisionEnabledAttr(False)
            UsdPhysics.RigidBodyAPI.Apply(payload.GetPrim()).CreateKinematicEnabledAttr(True)
            UsdPhysics.MassAPI.Apply(payload.GetPrim()).CreateMassAttr(.25)
            r.update(item=payload.GetPrim(),originals=originals,item_start=center.tolist())
            payload_bounds[path]=(lo-center,hi-center)
            # Assisted pickup must not explode interpenetrating source piles
            # or catch a long beam on the carrier's own links.
            robot_bodies=[p.GetPath() for p in Usd.PrimRange(r['model']) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
            neighbors=[p.GetPath() for p in props.GetChildren() if p not in originals
                       and np.linalg.norm(((sum(bounds([p])))/2)[:2]-center[:2])<2.]
            UsdPhysics.FilteredPairsAPI.Apply(payload.GetPrim()).CreateFilteredPairsRel().SetTargets(robot_bodies+neighbors)
            r['robot_body_filters']=robot_bodies
            # Install filters before PhysX first discovers the dynamic body.
            yield
            rigid=SingleRigidPrim(path)
            rigid.initialize()
            live[path]=rigid
            return payload.GetPrim()

        candidate_groups={job['robot']:job['candidates'] for job in environment_jobs}
        reserved=set()
        stored_props=[]
        rear_zone_owner=[None]
        left_corridor_owner=[1 if not args.robot or args.robot==1 else None]
        if cleanup is not None:left_corridor_owner[0]=None
        left_corridor_queue=[]
        cleanup_transit=FairTripGate()

        def visible_gaps(r,group):
            gaps=[]
            for finger,vertices in zip(r['finger_prims'],r['finger_vertices']):
                world_points=vertices@np.array(matrix(finger))
                distances=[]
                for obj in group:
                    transform=UsdGeom.XformCache().GetLocalToWorldTransform(obj)
                    inverse=transform.GetInverse()
                    half=float(UsdGeom.Cube(obj).GetSizeAttr().Get())/2
                    local=world_points@np.array(inverse)
                    closest=np.array(local,copy=True)
                    closest[:,:3]=np.clip(closest[:,:3],-half,half)
                    projected=closest@np.array(transform)
                    distances.append(float(np.min(np.linalg.norm(projected[:,:3]-world_points[:,:3],axis=1))))
                gaps.append(min(distances))
            return np.array(gaps)

        def exact_payload_bounds(prim):
            # Navigation uses conservative compound bounds. Support checks need
            # actual cube corners after rubble has rotated and settled.
            points=[]
            cache=UsdGeom.XformCache()
            for part in prim.GetChildren():
                if not part.IsA(UsdGeom.Cube):continue
                half=float(UsdGeom.Cube(part).GetSizeAttr().Get())/2
                transform=cache.GetLocalToWorldTransform(part)
                points.extend([list(transform.Transform(Gf.Vec3d(x,y,z)))
                    for x in (-half,half) for y in (-half,half) for z in (-half,half)])
            if not points:raise RuntimeError('No original payload geometry for support verification')
            return np.min(points,axis=0),np.max(points,axis=0)

        def surface_contact(r,group,grip):
            event(r,'align_existing_gripper_to_surface')
            yield from arm_to(r,grip+[0,0,.06],closed=True)
            yield from arm_to(r,grip,closed=True)
            indices=[contact_paths.index(str(p.GetPath())) for p in group if str(p.GetPath()) in contact_paths]
            for attempt in range(9):
                for _ in range(12):yield
                gaps=visible_gaps(r,group)
                forces=np.asarray(r['contacts'].get_contact_force_matrix(dt=1/60))[:,indices,:]
                force=np.linalg.norm(np.asarray(r['contacts'].get_net_contact_forces(dt=1/60)),axis=1)
                print('SOURCE_CONTACT',r['id'],'gaps',gaps,'forces',force)
                if np.any((gaps<.003)&(force>.02)):
                    r['contact_check']=dict(visible_gaps_m=gaps.tolist(),forces_N=force.tolist(),verified=True)
                    event(r,'surface_contact_verified')
                    return
                # Only the arm moves. Original props remain at their source pose.
                yield from arm_to(r,grip-[0,0,.001*(attempt+1)],seconds=.25,closed=True)
            raise RuntimeError('No verified visible surface contact; refusing attachment')

        # These are existing bottom-rack pallets, not a new front workbench.
        # Each destination is a separate physical support in the source USD.
        rack_specs={1:('/Root/Shelf_0/SM_PaletteA_357/SM_PaletteA_01',math.pi),
                    2:('/Root/Shelf_0/SM_PaletteA_12/SM_PaletteA_01',math.pi),
                    3:('/Root/Shelf_0/SM_PaletteA_5/SM_PaletteA_01',math.pi),
                    4:('/Root/Shelf_2/SM_PaletteA_12/SM_PaletteA_01',0.),
                    5:('/Root/Shelf_2/SM_PaletteA_357/SM_PaletteA_01',0.)}
        next_rack_specs={1:('/Root/Shelf_0/SM_PaletteA_318/SM_PaletteA_01',math.pi),
                         2:('/Root/Shelf_0/SM_PaletteA_12/SM_PaletteA_01',math.pi),
                         3:('/Root/Shelf_0/SM_PaletteA_5/SM_PaletteA_01',math.pi),
                         4:('/Root/Shelf_2/SM_PaletteA_5/SM_PaletteA_01',0.),
                         5:('/Root/Shelf_0/SM_PaletteA_319/SM_PaletteA_01',math.pi)}

        def rack_ik_available(r,grip,source_yaw,source_lo,source_hi):
            """A task is available only if its original prop can also be placed."""
            if cleanup is not None:
                if cleanup.items[r['candidate_object_id']]['state']!='pending':return False
                half=(source_hi-source_lo)/2
                rack_checks=0
                preferred={rack_specs[r['id']][0],next_rack_specs[r['id']][0]}
                for pallet in sorted(cleanup_pallets,key=lambda p:p['path'] not in preferred):
                    plo,phi=pallet['lo'],pallet['hi']
                    path=pallet['path']
                    shelf_lo,shelf_hi=bounds([stage.GetPrimAtPath(path.split('/SM_')[0])])
                    yaws=(math.pi,) if '/Shelf_0/' in path else ((0.,) if '/Shelf_2/' in path else (0.,math.pi))
                    for yaw in yaws:
                        angle=yaw-source_yaw
                        rotation=np.array([[math.cos(angle),-math.sin(angle),0],
                                           [math.sin(angle),math.cos(angle),0],[0,0,1]])
                        projected=np.abs(rotation)@half
                        if r.get('candidate_kind')=='beam' and not beam_fits_support(projected[:2],(phi[0]-plo[0],phi[1]-plo[1])):continue
                        low,high=plo[1]+projected[1]+.035,phi[1]-projected[1]-.035
                        if low>high:continue
                        middle=(low+high)/2
                        ys=sorted(set([float(low),float(high),float(middle)]+
                                      [float(v) for v in np.arange(low,high+.001,.12)]),
                                  key=lambda v:abs(v-(middle-.30 if r['candidate_kind'] in ('debris','tool') else middle)))
                        toward=np.array([math.cos(yaw),math.sin(yaw)])
                        edge=shelf_hi[0] if yaw==math.pi else shelf_lo[0]
                        support_edge=phi[0] if yaw==math.pi else plo[0]
                        cx=support_edge-toward[0]*.015+toward[0]*projected[0]
                        if r.get('candidate_kind')=='beam':cx=(plo[0]+phi[0])/2+.15
                        if not plo[0]-.015<=cx<=phi[0]+.015:continue
                        xs=[float(cx)]
                        if r.get('candidate_kind')!='beam':
                            xlow,xhigh=plo[0]+projected[0]+.035,phi[0]-projected[0]-.035
                            if xlow<=xhigh:
                                xs+=sorted(set([float(xlow),float(xhigh)]+[float(v) for v in np.arange(xlow,xhigh+.001,.12)]),key=lambda x:abs(x-cx))
                        for cx,y in ((x,y) for x in xs for y in ys):
                            rack_checks+=1
                            if rack_checks%16==0:
                                yield
                                if cleanup.items[r['candidate_object_id']]['state']!='pending':return False
                            box=(float(cx-projected[0]),float(y-projected[1]),float(cx+projected[0]),float(y+projected[1]))
                            if not cleanup.slot_free(box):continue
                            dock=np.array([edge-toward[0]*.70,y])
                            if not r['selection_nav'].free(dock):continue
                            import omni.physx
                            hit=omni.physx.get_physx_scene_query_interface().raycast_closest(
                                (float(cx),float(y),float(phi[2]+.15)),(0.,0.,-1.),.6)
                            if not hit['hit'] or not any(str(hit.get(k,'')).startswith(path) for k in ('rigidBody','collision')):continue
                            support_z=float(phi[2]+.15-hit['distance'])
                            centre=np.array([cx,y,support_z+.003+projected[2]])
                            target=centre+rotation@(grip-(source_lo+source_hi)/2)
                            shoulder=np.array([*(dock+.30*toward),r['shoulder_z']])
                            if float(np.linalg.norm(target-shoulder))>1.2:continue
                            r['solver'].set_robot_base_pose(shoulder,np.array([math.cos(yaw/2),0,0,math.sin(yaw/2)]))
                            if all(r['solver'].compute_inverse_kinematics('right_gripper',p,down(yaw))[1]
                                   for p in (target,target+[0,0,.06],target+[0,0,.15])):
                                r['candidate_rack_slot']={'path':path,'yaw':float(yaw),'y':float(y),
                                    'centre_x':float(cx),'support_z':support_z,'box':box,'kind':r['candidate_kind']}
                                return True
                return False
            path,yaw=(rack_specs if r['task_index']==0 else next_rack_specs)[r['id']]
            pallet_lo,pallet_hi=bounds([stage.GetPrimAtPath(path)])
            shelf=stage.GetPrimAtPath(path.split('/SM_')[0])
            shelf_lo,shelf_hi=bounds([shelf])
            toward=np.array([math.cos(yaw),math.sin(yaw)])
            edge=shelf_hi[0] if yaw==math.pi else shelf_lo[0]
            rack_y=(pallet_lo[1]+pallet_hi[1])/2
            if r['id'] in (2,3):rack_y+=(-.30 if r['task_index']==0 else .30)
            dock=np.array([edge-toward[0]*.70,rack_y])
            angle=yaw-source_yaw
            rotation=np.array([[math.cos(angle),-math.sin(angle),0],
                               [math.sin(angle),math.cos(angle),0],[0,0,1]])
            half=(source_hi-source_lo)/2
            projected=np.abs(rotation)@half
            support_edge=pallet_hi[0] if yaw==math.pi else pallet_lo[0]
            centre_x=support_edge-toward[0]*.015+toward[0]*projected[0]
            if r['id']==4:centre_x=(pallet_lo[0]+pallet_hi[0])/2+.15
            centre=np.array([centre_x,rack_y,pallet_hi[2]+.003+projected[2]])
            target=centre+rotation@(grip-(source_lo+source_hi)/2)
            shoulder=np.array([*(dock+.30*toward),r['shoulder_z']])
            r['solver'].set_robot_base_pose(shoulder,np.array([math.cos(yaw/2),0,0,math.sin(yaw/2)]))
            return all(r['solver'].compute_inverse_kinematics('right_gripper',p,down(yaw))[1]
                       for p in (target,target+[0,0,.06]))

        def reserve_left_corridor(r,holding):
            if left_corridor_owner[0]==r['id']:return
            if r['id'] not in left_corridor_queue:left_corridor_queue.append(r['id'])
            if left_corridor_owner[0] is not None or left_corridor_queue[0]!=r['id']:
                event(r,'move_to_clear_corridor_waiting_point')
                yield from navigate(r,np.array(holding))
                event(r,'yield_shared_left_rack_corridor')
                while left_corridor_owner[0] is not None or left_corridor_queue[0]!=r['id']:yield
            left_corridor_queue.pop(0)
            left_corridor_owner[0]=r['id']
            event(r,'shared_left_rack_corridor_reserved')

        def placement_path_feasible(r,start,target,shoulder):
            """Check actual carried-grasp paths with continuous IK seeds."""
            r['solver'].set_robot_base_pose(*shoulder)
            seed=r['robot'].get_joint_positions()[r['arm']].copy()
            points=(np.array(start),np.array(target)+[0,0,.06],np.array(target),np.array(target)+[0,0,.10])
            for a,b in zip(points,points[1:]):
                for t in np.linspace(0,1,13):
                    joints,ok=r['solver'].compute_inverse_kinematics('right_gripper',a+(b-a)*t,down(r['yaw']),
                        warm_start=seed,position_tolerance=.004)
                    if not ok:
                        r['placement_ik_failure']={'point':(a+(b-a)*t).tolist(),'stage':'unreachable IK'}
                        return False
                    seed=joints
            return True

        def fit_actual_placement(r,item,pallet_lo,pallet_hi,rack_yaw,tried=None):
            """Adjust reservation/dock through robot motion, never object edits."""
            import omni.physx
            original=dict(r['cleanup_slot'])
            tried=set(tried or ())|{original['path']}
            offsets=sorted([(x,y) for x in (0.,-.04,.04,-.08,.08) for y in (0.,-.04,.04,-.08,.08)],key=lambda pair:math.hypot(*pair))
            docks=sorted([(d,y) for d in (0.,.035,.08,.14,-.06,-.12,-.20) for y in (0.,-.08,.08,-.16,.16)],key=lambda pair:math.hypot(*pair))
            for round_index in range(3):
                r['solver'].set_robot_base_pose(*pose(r['shoulder']))
                tcp=r['ik'].compute_end_effector_pose()[0]
                body_lo,body_hi=exact_payload_bounds(item)
                box_center=(body_lo+body_hi)/2
                centres=[(original['centre_x']+dx,original['y']+dy) for dx,dy in offsets]
                centres+=supported_centres(pallet_lo,pallet_hi,(body_hi-body_lo)/2,(original['centre_x'],original['y']))
                base=pose(r['base'])[0]
                shoulder,orientation=pose(r['shoulder'])
                nav=planner(r)
                chosen=None
                for dock_index,(forward,lateral) in enumerate(docks):
                    delta=np.array([forward*math.cos(rack_yaw),lateral,0.])
                    goal=base[:2]+delta[:2]
                    if not nav.free(goal) or not nav.clear(base[:2],goal) or not traffic_clear(r,goal):continue
                    for candidate_index,(cx,cy) in enumerate(centres):
                        if candidate_index and candidate_index%24==0:yield
                        if not (pallet_lo[0]+.005<=cx<=pallet_hi[0]-.005 and pallet_lo[1]+.005<=cy<=pallet_hi[1]-.005):continue
                        shift=np.array([cx-box_center[0],cy-box_center[1],0.])
                        box=(float(body_lo[0]+shift[0]),float(body_lo[1]+shift[1]),float(body_hi[0]+shift[0]),float(body_hi[1]+shift[1]))
                        slot=dict(original,centre_x=float(cx),y=float(cy),box=box)
                        if not cleanup.can_retarget(r['cleanup_id'],slot):continue
                        hit=omni.physx.get_physx_scene_query_interface().raycast_closest((float(cx),float(cy),float(pallet_hi[2]+.15)),(0.,0.,-1.),.6)
                        if not hit['hit'] or not any(str(hit.get(k,'')).startswith(original['path']) for k in ('rigidBody','collision')):continue
                        support_z=float(pallet_hi[2]+.15-hit['distance'])
                        shift[2]=support_z+.003-body_lo[2]
                        destination=tcp+shift
                        if placement_path_feasible(r,tcp+delta,destination,(shoulder+delta,orientation)):
                            chosen=(slot,goal,destination,support_z,shift)
                            break
                    if chosen:break
                    if dock_index%3==2:
                        r['solver'].set_robot_base_pose(*pose(r['shoulder']))
                        yield
                r['solver'].set_robot_base_pose(*pose(r['shoulder']))
                if chosen is None:
                    print('PLACEMENT_DIAGNOSTIC',r['id'],'tcp',tcp,'bounds',body_lo,body_hi,'base',base,
                          'slot',original,'last_ik_failure',r.get('placement_ik_failure'),flush=True)
                    # Keep the actual grasp and try another existing support
                    # with the same approach orientation; no object-pose edit.
                    if len(tried)<3:
                        for pallet in sorted(cleanup_pallets,key=lambda p:math.dist(((p['lo']+p['hi'])/2)[:2],base[:2])):
                            path=pallet['path']
                            if path in tried:continue
                            allowed=(math.pi,) if '/Shelf_0/' in path else ((0.,) if '/Shelf_2/' in path else (0.,math.pi))
                            if rack_yaw not in allowed:continue
                            plo,phi=pallet['lo'],pallet['hi']
                            shelf=path.split('/SM_')[0]
                            slo,shi=bounds([stage.GetPrimAtPath(shelf)])
                            toward=np.array([math.cos(rack_yaw),math.sin(rack_yaw)])
                            edge=shi[0] if rack_yaw==math.pi else slo[0]
                            for cx,cy in supported_centres(plo,phi,(body_hi-body_lo)/2,((plo[0]+phi[0])/2,original['y'])):
                                yield
                                shift=np.array([cx-box_center[0],cy-box_center[1],0.])
                                box=(float(body_lo[0]+shift[0]),float(body_lo[1]+shift[1]),float(body_hi[0]+shift[0]),float(body_hi[1]+shift[1]))
                                slot=dict(original,path=path,centre_x=float(cx),y=float(cy),box=box)
                                if not cleanup.can_retarget(r['cleanup_id'],slot):continue
                                hit=omni.physx.get_physx_scene_query_interface().raycast_closest((float(cx),float(cy),float(phi[2]+.15)),(0.,0.,-1.),.6)
                                if not hit['hit'] or not any(str(hit.get(k,'')).startswith(path) for k in ('rigidBody','collision')):continue
                                support_z=float(phi[2]+.15-hit['distance'])
                                shift[2]=support_z+.003-body_lo[2]
                                goal=np.array([edge-toward[0]*.70,cy])
                                delta=np.array([*(goal-base[:2]),0.])
                                if not placement_path_feasible(r,tcp+delta,tcp+shift,(shoulder+delta,orientation)):continue
                                old_shelf=r['destination_shelf']
                                r['destination_shelf']=shelf
                                try:planner(r).plan(base[:2],goal)
                                except ValueError:
                                    r['destination_shelf']=old_shelf
                                    continue
                                # No yield between final lease check and update.
                                if not cleanup.can_retarget(r['cleanup_id'],slot):
                                    r['destination_shelf']=old_shelf
                                    continue
                                slot['support_z']=support_z
                                cleanup.retarget(r['cleanup_id'],slot)
                                r['cleanup_slot']=slot
                                r['rack_path']=path
                                event(r,'retry_actual_grasp_on_alternative_existing_pallet')
                                r['solver'].set_robot_base_pose(*pose(r['shoulder']))
                                yield from navigate(r,goal,yaw=rack_yaw)
                                return (yield from fit_actual_placement(r,item,plo,phi,rack_yaw,tried))
                    r['solver'].set_robot_base_pose(*pose(r['shoulder']))
                    raise RuntimeError('No reachable placement/retreat path after checked existing-pallet alternatives')
                slot,goal,destination,support_z,shift=chosen
                slot['support_z']=support_z
                cleanup.retarget(r['cleanup_id'],slot)
                r['cleanup_slot']=slot
                if float(np.linalg.norm(goal-base[:2]))>.005:
                    event(r,'adjust_docking_for_reachable_placement')
                    r.setdefault('placement_adjustments',[]).append({'time':clock[0]/60,'dock':goal.tolist(),'rack_y':slot['y'],'centre_x':slot['centre_x']})
                    print('PLACEMENT_REDOCK',r['id'],'dock',goal,'slot',slot['centre_x'],slot['y'],flush=True)
                    yield from navigate(r,goal,yaw=rack_yaw)
                    continue
                print('PLACEMENT_PREFLIGHT',r['id'],'tcp',tcp,'target',destination,'offset',slot['centre_x']-original['centre_x'],slot['y']-original['y'],flush=True)
                r['drop_goal']=(pose(item)[0]+shift).tolist()
                return destination,support_z,slot['y']
            raise RuntimeError('Placement docking did not converge to a reachable actual grasp pose')

        def available_cleanup(r):
            keys=cleanup.available(r['id'])
            return [key for key in keys if key.rsplit('/',1)[-1]==args.only_object] if args.only_object else keys

        def needs_idle_clear(r):
            return idle_clear_due(pose(r['base'])[0],r['home'],r.get('carrying',False),
                                  r.get('idle_clear_attempt',-math.inf),clock[0]/60)

        def run_one_task(r):
            try:
                if cleanup is not None:
                    r['task']='Selecting next feasible task'
                    r['selected_props']=[]
                event(r,'reserve_existing_environment_object')
                available=available_cleanup(r) if cleanup is not None else []
                if cleanup is not None and not args.fast_cleanup:
                    own={str(group[0].GetPath()) for group in candidate_groups[r['id']]}
                    preferred=[key for key in available if key in own]
                    available=preferred or available
                groups=([cleanup_by_id[key]['prims'] for key in available] if cleanup is not None else
                        [g for g in candidate_groups[r['id']] if not any(str(p.GetPath()) in reserved for p in g)])
                try:
                    _,group,dock,grip,carry,yaw,rack_slot=yield from choose(r,groups)
                except RuntimeError as exc:
                    if cleanup is not None and str(exc).startswith('No reachable floor object'):
                        for key in r.get('searched_keys',[]):
                            cleanup.reject(r['id'],[key],r.get('placement_rejections',{}).get(key,
                                'No currently feasible route, grasp and unoccupied rack slot'))
                            cleanup.items[key].setdefault('planning_diagnostics',{})[str(r['id'])]=r.get('search_diagnostics',{}).get(key,{})
                            print('TASK REJECTED',r['id'],key,cleanup.items[key]['reason'],flush=True)
                        r['cleanup_waiting']=True
                        event(r,'cleanup_waiting')
                        return
                    if not r['completed_jobs'] or not str(exc).startswith('No reachable floor object'):raise
                    r['no_more_reachable_props']=True
                    r['success']=True
                    event(r,'idle_no_more_reachable_original_props')
                    return
                r['selected_props']=[str(p.GetPath()) for p in group]
                if cleanup is not None:
                    key=str(group[0].GetPath())
                    r['cleanup_lease']=cleanup.claim(key,r['id'],rack_slot)
                    r['cleanup_id']=key
                    r['cleanup_slot']=rack_slot
                    r['cleanup_manipulated']=False
                    r['task']='Arrange '+cleanup.items[key]['kind']+': '+group[0].GetName()
                reserved.update(r['selected_props'])
                print('Selected ORIGINAL props:',r['selected_props'])
                if cleanup is not None and args.serial_cleanup:
                    cleanup_transit.request(r['id'])
                    event(r,'yield_cleanup_aisle_turn')
                    while not cleanup_transit.enter(r['id']):yield
                    event(r,'cleanup_aisle_trip_reserved')
                needs_rear_zone=cleanup is None and r['id'] in (4,5)
                if needs_rear_zone:
                    if rear_zone_owner[0] is not None:
                        event(r,'navigate_to_clear_rear_waiting_point')
                        holding=r['home'][:2].copy() if cleanup is not None else np.array([2.0,7.0])
                        yield from navigate(r,holding,yaw=0.)
                        event(r,'yield_rear_pickup_zone_to_other_robot')
                        while rear_zone_owner[0] is not None:yield
                    rear_zone_owner[0]=r['id']
                event(r,'navigate_to_original_prop')
                yield from navigate(r,dock,yaw)
                if r['id']==1 and r['task_index']==0 and left_corridor_owner[0]==1:
                    left_corridor_owner[0]=None
                yield from surface_contact(r,group,grip)
                source_hand=pose(r['hand'])[0].copy()
                original_center=sum(bounds(group))/2
                # Stop commanding penetration into the static source while
                # its collision representation changes for one physics frame.
                # Hold measured joints; visible contact is rechecked below.
                measured=r['robot'].get_joint_positions()
                r['robot'].apply_action(ArticulationAction(joint_positions=measured[r['arm']].copy(),joint_indices=r['arm']))
                r['robot'].apply_action(ArticulationAction(joint_positions=measured[r['fingers']].copy(),joint_indices=r['fingers']))
                if cleanup is not None:r['cleanup_manipulated']=True
                item=yield from mobilize(r,group)
                # Compound conversion must preserve the exact visible source pose.
                if np.linalg.norm(pose(item)[0]-original_center)>.002 or np.linalg.norm(pose(r['hand'])[0]-source_hand)>.003:
                    raise RuntimeError('Source pose changed before assisted grasp')
                attach_gaps=visible_gaps(r,group)
                if min(attach_gaps)>=.003:
                    raise RuntimeError('Visible contact was lost during compound conversion')
                r['contact_check']['attachment_gaps_m']=attach_gaps.tolist()
                event(r,'contact_verified_assisted_grasp')
                attach(task_root+'/Grasp_%02d'%r['id'],item,r['hand'])
                UsdPhysics.RigidBodyAPI(item).CreateKinematicEnabledAttr(False)
                r['carrying']=True
                if args.inject_pickup_failure and not fault_injection['used']:
                    fault_injection['used']=True
                    r['injected_pickup_failure']=True
                    raise RuntimeError('Injected pickup failure for verified recovery test')
                before=pose(item)[0].copy()
                relative=matrix(item)*matrix(r['hand']).GetInverse()
                yield from arm_to(r,grip+[0,0,.07],closed=True)
                after=pose(item)[0]
                drift=float(np.linalg.norm(np.array((matrix(item)*matrix(r['hand']).GetInverse()).ExtractTranslation())-np.array(relative.ExtractTranslation())))
                if after[2]-before[2]<.04 or drift>.005:
                    raise RuntimeError('Assisted contact lift did not verify')
                r['lift_check']=dict(rise_m=float(after[2]-before[2]),relative_drift_m=drift)
                event(r,'lift_verified_carry_to_existing_rack')
                yield from arm_to(r,carry,closed=True)
                pallet_path,rack_yaw=((r['cleanup_slot']['path'],r['cleanup_slot']['yaw']) if cleanup is not None else
                                     (rack_specs if r['task_index']==0 else next_rack_specs)[r['id']])
                pallet=stage.GetPrimAtPath(pallet_path)
                if not pallet:raise RuntimeError('Existing rack pallet is missing: '+pallet_path)
                lo,hi=bounds([pallet])
                rack_y=r['cleanup_slot']['y'] if cleanup is not None else (lo[1]+hi[1])/2
                if cleanup is None and r['id'] in (2,3):rack_y+=(-.30 if r['task_index']==0 else .30)
                rack=stage.GetPrimAtPath(pallet_path.split('/SM_')[0])
                rack_lo,rack_hi=bounds([rack])
                r['destination_shelf']=str(rack.GetPath())
                if cleanup is None and r['id']==1:
                    yield from reserve_left_corridor(r,[-3.,7.])
                toward=np.array([math.cos(rack_yaw),math.sin(rack_yaw)])
                edge=rack_hi[0] if rack_yaw==math.pi else rack_lo[0]
                depot=np.array([edge-toward[0]*.70,rack_y])
                r['rack_path']=pallet_path
                # Withdraw into the clear front apron before turning a long load.
                front_nav=planner(r)
                front_options=[np.array([dock[0]+dx,y]) for y in (-5.7,-6.6,-8.7)
                               for dx in (0.,-.8,.8,-1.6,1.6,-2.4,2.4)
                               if front_nav.free([dock[0]+dx,y])]
                direct_route=False
                if args.fast_cleanup:
                    try:
                        front_nav.plan(pose(r['base'])[0][:2],depot)
                        direct_route=True
                    except ValueError:pass
                if not front_options and not direct_route:raise RuntimeError('No clear front transit waypoint for this original prop')
                current=pose(r['base'])[0]
                if not direct_route:
                    front=min(front_options,key=lambda p:np.linalg.norm(p-current[:2]))
                    yield from navigate(r,front)
                if cleanup is None and rear_zone_owner[0]==r['id']:
                    rear_zone_owner[0]=None
                if '/Shelf_0/' in pallet_path and not concurrent_cleanup:
                    yield from reserve_left_corridor(r,[r['home'][0],-8.7])
                event(r,'navigate_to_assigned_existing_rack')
                yield from navigate(r,depot,yaw=rack_yaw)
                r['solver'].set_robot_base_pose(*pose(r['shoulder']))
                tcp=r['ik'].compute_end_effector_pose()[0]
                body_lo,body_hi=exact_payload_bounds(item) if cleanup is not None else bounds([item])
                body_center=pose(item)[0]
                # Align the existing object's near edge with the pallet edge.
                near=body_hi[0] if rack_yaw==math.pi else body_lo[0]
                support_edge=hi[0] if rack_yaw==math.pi else lo[0]
                dx=support_edge-toward[0]*.015-near
                if cleanup is not None:
                    dx=r['cleanup_slot']['centre_x']-(body_lo[0]+body_hi[0])/2
                elif r['id']==4:
                    # Support a long beam near its centre of mass, rather
                    # than balancing it at the pallet's far edge.
                    dx=(lo[0]+hi[0])/2+.15-body_center[0]
                dy=rack_y-(body_lo[1]+body_hi[1])/2
                support_z=r['cleanup_slot']['support_z'] if cleanup is not None else float(hi[2])
                dz=support_z+.003-body_lo[2]
                shift=np.array([dx,dy,dz])
                destination=tcp+shift
                r['drop_goal']=(body_center+shift).tolist()
                if cleanup is not None:
                    destination,support_z,rack_y=yield from fit_actual_placement(r,item,lo,hi,rack_yaw)
                    pallet_path=r['cleanup_slot']['path']
                    lo,hi=bounds([stage.GetPrimAtPath(pallet_path)])
                event(r,'place_on_existing_rack_pallet')
                yield from arm_to(r,destination+[0,0,.06],closed=True)
                yield from arm_to(r,destination,closed=True)
                stage.RemovePrim(task_root+'/Grasp_%02d'%r['id'])
                UsdPhysics.FilteredPairsAPI(item).GetFilteredPairsRel().SetTargets(r['robot_body_filters'])
                r['carrying']=False
                r['robot'].apply_action(ArticulationAction(joint_positions=np.array([.04,.04]),joint_indices=r['fingers']))
                event(r,'release_on_rack')
                for _ in range(30):yield
                try:
                    yield from arm_to(r,destination+[0,0,.10])
                except RuntimeError:
                    if cleanup is None:raise
                    yield from arm_to(r,destination+[0,0,.06])
                r['robot'].apply_action(ArticulationAction(joint_positions=folded,joint_indices=r['arm']))
                for _ in range(90):yield
                UsdPhysics.FilteredPairsAPI(item).GetFilteredPairsRel().SetTargets([])
                final,_=pose(item)
                r['placement_error']=float(np.linalg.norm(final-np.array(r['drop_goal'])))
                print('RACK_SETTLE',r['id'],'actual',final,'expected',r['drop_goal'],'error',r['placement_error'],
                      'velocity',live[str(item.GetPath())].get_linear_velocity())
                supported=True
                if cleanup is not None or r['id']==4:
                    final_lo,final_hi=exact_payload_bounds(item) if cleanup is not None else bounds([item])
                    supported=bool(np.all(final[:2]>=lo[:2]-.015) and np.all(final[:2]<=hi[:2]+.015)
                                   and abs(final_lo[2]-support_z)<.025
                                   and np.linalg.norm(final[:2]-np.array(r['drop_goal'])[:2])<.35)
                elif r['placement_error']>.06:
                    supported=False
                if not supported or np.linalg.norm(live[str(item.GetPath())].get_linear_velocity())>.05:
                    raise RuntimeError('Original prop did not settle on assigned rack pallet')
                if cleanup is not None:
                    exact_lo,exact_hi=exact_payload_bounds(item)
                    cleanup.complete(r['cleanup_id'],True,(float(exact_lo[0]),float(exact_lo[1]),float(exact_hi[0]),float(exact_hi[1])))
                r['success']=True
                stored_props.append(item)
                r['completed_jobs'].append(dict(source_props=r['selected_props'],rack_path=pallet_path,
                    rack_slot_y=float(rack_y),
                    support_z=support_z,
                    cleanup_object_id=r.get('cleanup_id'),
                    object_kind=cleanup.items[r['cleanup_id']]['kind'] if cleanup is not None else None,
                    expected_position=r['drop_goal'],
                    payload_path=str(item.GetPath()),contact_check=r['contact_check'],lift_check=r['lift_check'],
                    placement_error_m=r['placement_error']))
                event(r,'completed_original_prop_on_rack')
                if cleanup is not None:
                    print('CLEANUP PROGRESS:',cleanup.summary()['counts'],flush=True)
            except Exception as exc:
                if cleanup is not None and r.get('cleanup_manipulated',False):cleanup_transit.release(r['id'])
                if cleanup is not None and r.get('cleanup_id'):
                    cleanup.fail(r['cleanup_id'],str(exc),before_manipulation=not r.get('cleanup_manipulated',False))
                    if not r.get('cleanup_manipulated',False):
                        reserved.difference_update(r.get('selected_props',[]))
                if rear_zone_owner[0]==r['id']:
                    rear_zone_owner[0]=None
                if left_corridor_owner[0]==r['id']:left_corridor_owner[0]=None
                if r['id'] in left_corridor_queue:left_corridor_queue.remove(r['id'])
                r['failure']=str(exc)
                if isinstance(exc,TrafficDeferred) and cleanup is not None:
                    r['traffic_deferrals']=r.get('traffic_deferrals',0)+1
                    r['traffic_deferred']=True
                    event(r,'traffic_deferred_recover_load' if r.get('cleanup_manipulated',False) else 'traffic_deferred_release_unstarted_task')
                    print('TRAFFIC DEFERRED',r['id'],exc,flush=True)
                    return
                event(r,'failed')
                print('FAILED',r['id'],exc)
                traceback.print_exc()
        def recover_floor_payload(r):
            """Physical release, settle and pose-preserving source handoff."""
            item=r.get('item')
            if item is None or not item.IsValid():return False
            path=str(item.GetPath())
            if path not in live:return False
            event(r,'release_failed_load_for_floor_recovery')
            stage.RemovePrim(task_root+'/Grasp_%02d'%r['id'])
            UsdPhysics.RigidBodyAPI(item).CreateKinematicEnabledAttr(False)
            UsdPhysics.FilteredPairsAPI(item).GetFilteredPairsRel().SetTargets([])
            r['carrying']=False
            if item not in stored_props:stored_props.append(item)
            r['robot'].apply_action(ArticulationAction(joint_positions=np.array([.04,.04]),joint_indices=r['fingers']))
            r['robot'].apply_action(ArticulationAction(joint_positions=folded,joint_indices=r['arm']))
            stable=0
            for _ in range(360):
                yield
                speed=float(np.linalg.norm(live[path].get_linear_velocity()))
                stable=stable+1 if speed<.03 else 0
                if stable>=45:break
            low,high=exact_payload_bounds(item)
            slot=r.get('cleanup_slot')
            lift=r.get('lift_check') or {}
            if stable>=45 and slot and lift.get('rise_m',0.)>=.04 and lift.get('relative_drift_m',1.)<=.005:
                pallet=stage.GetPrimAtPath(slot['path'])
                plo,phi=bounds([pallet])
                final=pose(item)[0]
                gap=float(low[2]-slot['support_z'])
                if np.all(final[:2]>=plo[:2]-.015) and np.all(final[:2]<=phi[:2]+.015) and abs(gap)<.025 and abs(final[1]-slot['y'])<.35:
                    box=(float(low[0]),float(low[1]),float(high[0]),float(high[1]))
                    try:cleanup.recovered_storage(r['cleanup_id'],r.get('contact_check',{}).get('verified',False),gap,speed,box)
                    except ValueError:pass
                    else:
                        target=np.array(r.get('drop_goal',[slot['centre_x'],slot['y'],slot['support_z']+(high[2]-low[2])/2]))
                        r['completed_jobs'].append(dict(source_props=r['selected_props'],rack_path=slot['path'],rack_slot_y=slot['y'],
                            support_z=slot['support_z'],cleanup_object_id=r['cleanup_id'],object_kind=cleanup.items[r['cleanup_id']]['kind'],
                            expected_position=target.tolist(),payload_path=path,contact_check=r['contact_check'],lift_check=lift,
                            placement_error_m=float(np.linalg.norm(final-target)),completion_method='measured supported settling after recovery release'))
                        r.setdefault('recoveries',[]).append({'time':clock[0]/60,'object':r['cleanup_id'],'method':'verified rack settling','support_gap_m':gap,'settled_speed_m_s':speed})
                        event(r,'completed_original_prop_on_rack')
                        r.update(item=None,originals=[],carrying=False,destination_shelf=None,cleanup_manipulated=False)
                        return True
            if stable<45 or not -.05<=low[2]<=.15:
                event(r,'recovery_requires_operator_load_not_stable_on_floor')
                return False
            originals=r['originals']
            parts=[stage.GetPrimAtPath(path+'/Part_%02d'%i) for i in range(len(originals))]
            if not originals or not all(parts):return False
            cache=UsdGeom.XformCache()
            transforms=[cache.GetLocalToWorldTransform(part) for part in parts]
            for original,transform in zip(originals,transforms):
                parent=cache.GetLocalToWorldTransform(original.GetParent())
                UsdGeom.Xformable(original).MakeMatrixXform().Set(transform*parent.GetInverse())
            error=0.
            cache=UsdGeom.XformCache()
            for original,part,transform in zip(originals,parts,transforms):
                actual=cache.GetLocalToWorldTransform(original)
                half=float(UsdGeom.Cube(part).GetSizeAttr().Get())/2
                for x in (-half,half):
                    for y in (-half,half):
                        for z in (-half,half):
                            point=Gf.Vec3d(x,y,z)
                            error=max(error,float((actual.Transform(point)-transform.Transform(point)).GetLength()))
            if error>.002:return False
            for original in originals:
                UsdGeom.Imageable(original).MakeVisible()
                UsdPhysics.CollisionAPI.Apply(original).CreateCollisionEnabledAttr(True)
            stored_props.remove(item)
            # Keep the rigid-body/shape prims allocated: deleting them while
            # other workers own tensor views invalidates the whole simulation.
            # This inactive container has no visible or colliding duplicate.
            UsdPhysics.RigidBodyAPI(item).CreateKinematicEnabledAttr(True)
            for part in parts:UsdPhysics.CollisionAPI(part).CreateCollisionEnabledAttr(False)
            UsdGeom.Imageable(item).MakeInvisible()
            pose_cache.clear();bounds_cache.clear()
            # Let PhysX apply the source/temporary collider handoff before
            # exposing the object to another robot's raycast/grasp preflight.
            for _ in range(3):yield
            cleanup.recovered(r['cleanup_id'],error,speed,True)
            reserved.difference_update(r['selected_props'])
            r.setdefault('recoveries',[]).append({'time':clock[0]/60,'object':r['cleanup_id'],
                'pose_handoff_error_m':error,'settled_speed_m_s':speed,'floor_height_m':float(low[2])})
            r.update(item=None,originals=[],carrying=False,destination_shelf=None,cleanup_manipulated=False)
            event(r,'floor_recovery_verified_object_requeued')
            return True

        def run_task(r):
            r['completed_jobs']=[]
            index=0
            while cleanup is not None or index<args.tasks_per_robot:
                if cleanup is not None:
                    if args.only_object and any(item['state']=='stored' and key.rsplit('/',1)[-1]==args.only_object
                                               for key,item in cleanup.items.items()):
                        r['success']=True
                        event(r,'diagnostic_object_stored')
                        return
                    if cleanup.summary()['floor_clean']:
                        r['success']=True
                        event(r,'all_assigned_environment_tasks_complete')
                        return
                    if not available_cleanup(r):
                        if needs_idle_clear(r):
                            r['cleanup_waiting']=False
                            r['idle_clear_attempt']=clock[0]/60
                            before=pose(r['base'])[0][:2].copy()
                            event(r,'clear_idle_robot_from_work_aisle')
                            try:
                                yield from navigate(r,r['home'][:2],yaw=0.)
                            except RuntimeError:
                                event(r,'idle_parking_route_blocked')
                            finally:
                                route_reservations.release(r['id'],clock[0]/60)
                                r['nav_active']=False
                            if float(np.linalg.norm(pose(r['base'])[0][:2]-before))>.1:
                                cleanup.epoch+=1
                                r['idle_relocations']=r.get('idle_relocations',0)+1
                            # Reconsider grasps after parking changes access.
                            yield
                            continue
                        r['cleanup_waiting']=True
                        if r['state']!='cleanup_waiting':event(r,'cleanup_waiting')
                        yield
                        continue
                    r['cleanup_waiting']=False
                    r['cleanup_id']=None
                    r['cleanup_manipulated']=False
                r['task_index']=index
                r['success']=False
                for field in ('lift_check','contact_check','drop_goal','placement_error'):r.pop(field,None)
                yield from run_one_task(r)
                if cleanup is not None and r.get('cleanup_waiting'):
                    yield
                    continue
                if not r['success']:
                    if cleanup is None:return
                    traffic_deferred=r.pop('traffic_deferred',False)
                    if r.get('cleanup_manipulated',False):
                        recovered=yield from recover_floor_payload(r)
                        if not recovered:
                            # A quarantined load need not retire an empty,
                            # physically clear manipulator for the whole run.
                            item=r.get('item')
                            if item is None or not item.IsValid():return
                            low,high=exact_payload_bounds(item)
                            r['solver'].set_robot_base_pose(*pose(r['shoulder']))
                            tip=r['ik'].compute_end_effector_pose()[0]
                            gap=float(np.linalg.norm(np.maximum(0.,np.maximum(low-tip,tip-high))))
                            speed=float(np.linalg.norm(live[str(item.GetPath())].get_linear_velocity()))
                            try:
                                cleanup.quarantine_released(r['cleanup_id'],
                                    not bool(stage.GetPrimAtPath(task_root+'/Grasp_%02d'%r['id'])),speed,gap)
                            except ValueError:
                                event(r,'unsafe_recovery_requires_operator')
                                return
                            r.setdefault('quarantined_loads',[]).append({'object':r['cleanup_id'],'time':clock[0]/60,
                                'speed_m_s':speed,'gripper_gap_m':gap})
                            r.update(item=None,originals=[],carrying=False,destination_shelf=None,cleanup_manipulated=False)
                            event(r,'released_load_blocked_robot_available')
                        index+=1
                        cleanup_transit.release(r['id'])
                        r['failure']=None
                        r['selected_props']=[]
                        r['task']='Selecting replacement task after load recovery'
                        event(r,'request_replacement_task_after_load_recovery')
                        yield
                        continue
                    elif traffic_deferred:
                        # No object was touched. The lease was already released;
                        # do not start another home-route wait on the same peers.
                        r['failure']=None
                        try:yield from backoff_route(r)
                        except RuntimeError:
                            event(r,'traffic_backoff_unavailable_wait_for_peer')
                        for _ in range(30+17*r['id']):yield
                        event(r,'request_alternative_task_after_traffic')
                        continue
                    # Source geometry is still static and unchanged. Return
                    # safely before this robot tries another pending object.
                    recovery_start=pose(r['base'])[0][:2].copy()
                    r['robot'].apply_action(ArticulationAction(joint_positions=folded,joint_indices=r['arm']))
                    for _ in range(90):yield
                    r.update(item=None,originals=[],carrying=False,destination_shelf=None)
                    try:
                        event(r,'recover_from_unstarted_pickup')
                        yield from navigate(r,r['home'][:2],yaw=0.)
                    except Exception:
                        cleanup_transit.release(r['id'])
                        event(r,'home_route_blocked_try_other_available_work')
                    cleanup_transit.release(r['id'])
                    r['failure']=None
                    if float(np.linalg.norm(pose(r['base'])[0][:2]-recovery_start))>.1:cleanup.epoch+=1
                    yield
                    continue
                if r.get('no_more_reachable_props'):return
                try:
                    event(r,'back_out_from_completed_rack_placement')
                    current=pose(r['base'])[0]
                    retreat=current[:2]-.35*np.array([math.cos(r['yaw']),math.sin(r['yaw'])])
                    yield from navigate(r,retreat)
                    r.update(item=None,originals=[],carrying=False,destination_shelf=None)
                    if not args.fast_cleanup:
                        event(r,'return_to_clear_front_apron')
                        yield from navigate(r,r['home'][:2],yaw=0.)
                except Exception as exc:
                    if left_corridor_owner[0]==r['id']:left_corridor_owner[0]=None
                    if rear_zone_owner[0]==r['id']:rear_zone_owner[0]=None
                    cleanup_transit.release(r['id'])
                    r['success']=False
                    r['failure']='Return navigation: '+str(exc)
                    event(r,'failed')
                    if cleanup is None:return
                    # The placement is already verified. A blocked return
                    # route must not permanently disable an empty-handed robot.
                    r.update(item=None,originals=[],carrying=False,destination_shelf=None,success=True,failure=None)
                    event(r,'return_route_blocked_request_alternative_work')
                if left_corridor_owner[0]==r['id']:left_corridor_owner[0]=None
                if rear_zone_owner[0]==r['id']:rear_zone_owner[0]=None
                cleanup_transit.release(r['id'])
                if cleanup is not None:cleanup.epoch+=1
                index+=1
                if cleanup is not None or index<args.tasks_per_robot:
                    event(r,'request_next_available_original_prop')
            event(r,'all_assigned_environment_tasks_complete')
        # Cooperative workers issue commands for the SAME physics frame.
        # Only this scheduler advances time; a worker never steps the world.
        selected=[r for r in records if not args.robot or r['id']==args.robot]
        from environment_cdd_runtime import SceneResearchRuntime
        research=SceneResearchRuntime(stage,selected,args)
        workers={r['id']:run_task(r) for r in selected}
        camera_start=UsdGeom.Xformable(camera).GetLocalTransformation()
        camera_unchanged=True
        cleanup_stop_reason=None
        while workers and clock[0]/60 < args.duration and app.is_running():
            for robot_id,worker in sorted(list(workers.items()),key=lambda pair:-research.priority(pair[0])):
                try:
                    next(worker)
                except StopIteration:
                    del workers[robot_id]
            if cleanup is not None and workers and all(r.get('cleanup_waiting',False) for r in selected if r['id'] in workers):
                # Exhausting a robot's preferred family does not mean the
                # shared pool is exhausted. Let its next turn try other kinds.
                if not any(item['state']=='assigned' for item in cleanup.items.values()) and not any(
                        available_cleanup(r) or needs_idle_clear(r) for r in selected if r['id'] in workers):
                    cleanup_stop_reason='No currently feasible pickup and free rack slot; remaining objects are not clean'
                    break
            if workers:
                tick()
                moving=0
                for r in selected:
                    current=pose(r['base'])[0][:2].copy()
                    previous=r.get('motion_sample',r['home'][:2])
                    if r.get('nav_active',False) and float(np.linalg.norm(current-previous))>.0005:
                        moving+=1
                        r.setdefault('first_motion_time',clock[0]/60)
                    r['motion_sample']=current
                motion_stats['peak_moving_robots']=max(motion_stats['peak_moving_robots'],moving)
                if moving>1:motion_stats['overlap_frames']+=1
                camera_unchanged &= UsdGeom.Xformable(camera).GetLocalTransformation()==camera_start
                if cleanup is not None and clock[0]%60==0:
                    state=cleanup.summary()
                    if ui_cleanup_summary is not None:
                        active=(','.join('%02d'%rid for rid in route_reservations.routes) or 'manipulating / waiting') if concurrent_cleanup else ('Robot %02d'%cleanup_transit.owner if cleanup_transit.owner else 'waiting')
                        ui_cleanup_summary.text='Stored %d/%d | active: %s | pending %d | blocked %d | failed %d'%(state['counts']['stored'],state['total_objects'],active,state['counts']['pending'],state['counts']['blocked'],state['counts']['failed'])
                    if clock[0]%300==0:
                        args.report.parent.mkdir(parents=True,exist_ok=True)
                        args.report.with_suffix('.cleanup.json').write_text(json.dumps(dict(state,simulation_time=clock[0]/60,
                            aisle_owner=cleanup_transit.owner,aisle_waiting=cleanup_transit.queue,active_navigation=list(route_reservations.routes),
                            robot_states=[{'robot':r['id'],'task':r['task'],'state':r['state'],'carrying':r.get('carrying',False),
                                'position':pose(r['base'])[0].tolist(),'stored_jobs':len(r.get('completed_jobs',[])),
                                'recoveries':len(r.get('recoveries',[])),'failure':r.get('failure')} for r in selected]),indent=2),encoding='utf-8')
                if clock[0]%600==0:
                    print('PROGRESS',clock[0]/60,[(r['id'],r['state'],tuple(round(float(v),2) for v in pose(r['base'])[0][:2])) for r in selected])
        for r in selected:
            if r['id'] in workers:
                r['failure']=cleanup_stop_reason or 'Simulation closed or maximum duration exceeded'
                r['success']=False
                if cleanup is not None and r.get('cleanup_id'):
                    cleanup.fail(r['cleanup_id'],r['failure'],before_manipulation=not r.get('cleanup_manipulated',False))
                event(r,'failed')
        for worker in workers.values():
            worker.close()

        # Recheck earlier placements after the other robots have finished.
        for r in selected:
            for job in r.get('completed_jobs',[]):
                prim=stage.GetPrimAtPath(job['payload_path'])
                final=pose(prim)[0]
                velocity=live[job['payload_path']].get_linear_velocity()
                job['final_position']=final.tolist()
                pallet_lo,pallet_hi=bounds([stage.GetPrimAtPath(job['rack_path'])])
                prop_lo,prop_hi=exact_payload_bounds(prim)
                slot_tolerance=.15 if r['id'] in (2,3) else .35
                support_z=job.get('support_z',float(pallet_hi[2]))
                stored=(np.all(final[:2]>=pallet_lo[:2]-.015) and np.all(final[:2]<=pallet_hi[:2]+.015)
                        and abs(prop_lo[2]-support_z)<.025
                        and abs(final[1]-job['rack_slot_y'])<slot_tolerance)
                job['support_gap_m']=float(prop_lo[2]-support_z)
                job['final_speed_m_s']=float(np.linalg.norm(velocity))
                job['still_stored']=bool(stored and np.linalg.norm(velocity)<.05)
                if not job['still_stored']:
                    r['success']=False
                    r['failure']='A previously placed original prop left its rack location'
                    if cleanup is not None and job.get('cleanup_object_id'):
                        cleanup.items[job['cleanup_object_id']].update(state='failed',reason=r['failure'])

        report=dict(scene=str(args.scene.resolve()),mode='mobile planar-drive physics; original floor props; assisted grasp',
                    reserved_original_paths=sorted(reserved),requested_max_tasks_per_robot=args.tasks_per_robot,
                    simulated_seconds=round(clock[0]/60,3),camera='fixed 28 mm perspective',camera_unchanged=bool(camera_unchanged),
                    execution='concurrent shared physics frames',robots=[])
        for r in records:
            if args.robot and r['id']!=args.robot:
                continue
            report['robots'].append(dict(robot='Robot_%02d'%r['id'],task=r['task'],success=r['success'],failure=r['failure'],
                                         started_at_s=r['transitions'][0]['time'] if r['transitions'] else None,
                                         first_motion_at_s=r.get('first_motion_time'),
                                         traffic_wait_frames=r.get('traffic_wait_frames',0),traffic_replans=r.get('traffic_replans',0),
                                         travel_distance_m=round(r['distance'],3),source_props=r.get('selected_props',[]),
                                         placement_error_m=r.get('placement_error'),rack_path=r.get('rack_path'),contact_check=r.get('contact_check'),lift_check=r.get('lift_check'),
                                         completed_jobs=r.get('completed_jobs',[]),
                                         placement_adjustments=r.get('placement_adjustments',[]),
                                         recoveries=r.get('recoveries',[]),route_backoffs=r.get('route_backoffs',0),
                                         traffic_deferrals=r.get('traffic_deferrals',0),
                                         idle_relocations=r.get('idle_relocations',0),
                                         quarantined_loads=r.get('quarantined_loads',[]),
                                         no_more_reachable_props=r.get('no_more_reachable_props',False),
                                         transitions=r['transitions'],routes=r['route']))
        report['research_pipeline']=research.summary()
        report['navigation']={'mode':'concurrent spatial route reservations and live local avoidance' if concurrent_cleanup else 'original/serial navigation',
                              **motion_stats,'peak_reserved_routes':route_reservations.peak_active,
                              'route_conflict_yields':route_reservations.denials,'route_events':route_reservations.events}
        if cleanup is not None:
            report['cleanup']=cleanup.summary()
            report['cleanup']['diagnostic_object_filter']=args.only_object
            report['cleanup']['stop_reason']=cleanup_stop_reason or ('All registered objects stored' if report['cleanup']['floor_clean'] else
                'Simulation limit, failed controller or unresolved objects')
            print('FLOOR CLEAN' if report['cleanup']['floor_clean'] else 'FLOOR NOT YET CLEAN',report['cleanup']['counts'],flush=True)
            if ui_cleanup_summary is not None:
                ui_cleanup_summary.text=('FLOOR CLEAN' if report['cleanup']['floor_clean'] else 'CLEANUP STOPPED: floor not yet clean')+' | stored %d/%d | see cleanup report'%(report['cleanup']['counts']['stored'],report['cleanup']['total_objects'])
            args.report.with_suffix('.cleanup.json').write_text(json.dumps(report['cleanup'],indent=2),encoding='utf-8')
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2),encoding='utf-8')
        for result in report['robots']:
            print(result['robot'],result['task'],'PASS' if result['success'] else 'FAIL','travel:',result['travel_distance_m'],'m',result['failure'] or '')
        print('TASK REPORT:',args.report.resolve())
        if args.screenshot:
            from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
            capture=capture_viewport_to_file(get_active_viewport(),str(args.screenshot.resolve()))
            for _ in range(60):world.render()
        if not args.headless and not args.exit_on_complete:
            print('Tasks ended. Close the window to exit.')
            while app.is_running():world.step(render=True)
        if cleanup is not None and args.only_object:
            return 0 if any(item['state']=='stored' and key.rsplit('/',1)[-1]==args.only_object for key,item in cleanup.items.items()) else 1
        return (0 if report['cleanup']['floor_clean'] else 1) if cleanup is not None else (0 if all(r['success'] for r in selected) else 1)
    except Exception:
        traceback.print_exc()
        sys.stderr.flush()
        return 1
    finally:
        if research is not None:
            research.close()
        app.close()


if __name__=='__main__':
    sys.exit(main())


