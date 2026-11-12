"""Five actual Ridgeback-Franka USD assets with ROS 2 camera/state interfaces.

Base movement is an explicitly kinematic cmd_vel adapter for an open-floor
integration benchmark. Arms remain parked; external task executors are separate.
The viewport camera is fixed. Mounted RGB cameras do not control the viewport.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import uuid


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robots',type=int,default=1,choices=range(1,6))
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--seconds',type=float,default=0.,help='Wall seconds after starting playback; zero runs until closed')
    parser.add_argument('--camera-hz',type=float,default=5.)
    parser.add_argument('--robot-assets-root')
    parser.add_argument('--fail-robot',default='')
    parser.add_argument('--fail-at',type=float,default=5.)
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'scenes'/'ros2_ridgeback_swarm.usda')
    args=parser.parse_args()
    if args.fail_robot and args.fail_robot not in [f'robot_{i+1}' for i in range(args.robots)]:
        parser.error('--fail-robot must identify one of the selected robots')
    if not math.isfinite(args.fail_at) or args.fail_at<0:
        parser.error('--fail-at must be nonnegative and finite')
    if not math.isfinite(args.camera_hz) or not 0<args.camera_hz<=30 or not math.isfinite(args.seconds) or args.seconds<0:
        parser.error('Camera rate must be 0..30 Hz and duration nonnegative, both finite')
    root=Path(os.environ.get('ISAAC_PATH',str(Path(sys.executable).parents[2])))
    os.environ.setdefault('ROS_DISTRO','humble')
    os.environ.setdefault('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
    os.environ.setdefault('ROS_DOMAIN_ID','42')
    libraries=str(root/'exts'/'isaacsim.ros2.bridge'/'humble'/'lib')
    if libraries not in os.environ.get('PATH','').split(os.pathsep):
        os.environ['PATH']=os.environ.get('PATH','')+os.pathsep+libraries
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':args.headless,'width':960,'height':640})
    ros_node=None
    try:
        import numpy as np
        import omni.usd
        import omni.timeline
        import omni.graph.core as og
        import omni.replicator.core as rep
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.core.utils.viewports import set_camera_view
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        from compat import cube,camera
        from asset_robots import resolve_assets_root
        enable_extension('isaacsim.ros2.bridge')
        app.update()
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String, Bool
        from nav_msgs.msg import Odometry
        rclpy.init()
        omni.usd.get_context().new_stage()
        stage=omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage,1.)
        cube(stage,'/World/Floor',(0,0,-.05),(14,14,.1),(.18,.22,.24))
        cube(stage,'/World/RearWall',(0,5,1.5),(14,.12,3),(.43,.47,.49))
        from pxr import UsdLux
        UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1500)
        assets_root=resolve_assets_root(args.robot_assets_root)
        asset=assets_root+'/Isaac/Robots/Clearpath/RidgebackFranka/ridgeback_franka.usd'
        robots={}
        camera_graphs={}
        boot=uuid.uuid4().hex
        capabilities=[['camera'],['inspection'],['delivery'],['inspection'],['camera']]
        render_products=[]
        for i in range(args.robots):
            rid=f'robot_{i+1}'
            path='/World/Robots/'+rid
            wrapper=UsdGeom.Xform.Define(stage,path)
            translation=wrapper.AddTranslateOp()
            rotation=wrapper.AddRotateZOp()
            translation.Set(Gf.Vec3d((i-2)*2,-3,.05))
            rotation.Set(90)
            model=UsdGeom.Xform.Define(stage,path+'/model').GetPrim()
            model.GetReferences().AddReference(asset)
            prims=list(Usd.PrimRange(model))
            if len(prims)<3:
                raise RuntimeError(f'Ridgeback-Franka USD reference failed: {asset}')
            for prim in prims:
                if prim.GetTypeName()=='OmniGraph':
                    prim.SetActive(False)
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
                if prim.IsA(UsdPhysics.Joint):
                    UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
                for name in ('drawLines','drawPoints'):
                    attr=prim.GetAttribute(name)
                    if attr:
                        attr.Set(False)
            camera_path=path+'/RGB'
            camera(stage,camera_path,(.3,0,1.2))
            product=rep.create.render_product(camera_path,(320,240))
            render_products.append(product)
            graph_path='/ROS/'+rid
            keys=og.Controller.Keys
            og.Controller.edit({'graph_path':graph_path,'evaluator_name':'execution'}, {
                keys.CREATE_NODES:[('Tick','omni.graph.action.OnPlaybackTick'),
                    ('Context','isaacsim.ros2.bridge.ROS2Context'),
                    ('RGB','isaacsim.ros2.bridge.ROS2CameraHelper'),
                    ('Info','isaacsim.ros2.bridge.ROS2CameraInfoHelper')],
                keys.CONNECT:[('Tick.outputs:tick','RGB.inputs:execIn'),('Tick.outputs:tick','Info.inputs:execIn'),
                              ('Context.outputs:context','RGB.inputs:context'),('Context.outputs:context','Info.inputs:context')],
                keys.SET_VALUES:[('Context.inputs:domain_id',int(os.environ['ROS_DOMAIN_ID'])),
                    ('RGB.inputs:renderProductPath',product.path),('Info.inputs:renderProductPath',product.path),
                    ('RGB.inputs:topicName',f'/{rid}/camera/rgb'),('Info.inputs:topicName',f'/{rid}/camera/camera_info'),
                    ('RGB.inputs:type','rgb'),('RGB.inputs:frameId',rid+'/camera_optical'),
                    ('Info.inputs:frameId',rid+'/camera_optical'),
                    ('RGB.inputs:frameSkipCount',max(0,round(60/args.camera_hz)-1)),
                    ('Info.inputs:frameSkipCount',max(0,round(60/args.camera_hz)-1)),
                    ('RGB.inputs:useSystemTime',True),('Info.inputs:useSystemTime',True)]})
            camera_graphs[rid]=graph_path
            robots[rid]={'x':(i-2)*2.,'y':-3.,'yaw':math.pi/2,'battery':100.,'capabilities':capabilities[i],
                         'translation':translation,'rotation':rotation,'cmd':(0.,0.,0.),'last_cmd':0.,'sequence':0,
                         'failed':False,'assignment':None}
            print(f'Loaded Ridgeback-Franka {rid}; camera -> /{rid}/camera/rgb',flush=True)

        lock=threading.RLock()
        state={'playing':False}
        class BridgeNode(Node):
            def __init__(self):
                super().__init__('isaac_ridgeback_interface')
                self.pub={rid:(self.create_publisher(String,f'/{rid}/status',10),
                               self.create_publisher(Odometry,f'/{rid}/odom',10)) for rid in robots}
                self.playback_pub=self.create_publisher(Bool,'/simulation/playback',10)
                self.keep=[]
                self.pending_failures=[]
                for rid in robots:
                    self.keep.append(self.create_subscription(Twist,f'/{rid}/cmd_vel',lambda m,r=rid:self.command(r,m),10))
                    self.keep.append(self.create_subscription(String,f'/{rid}/task_assignment',lambda m,r=rid:self.assignment(r,m),10))
                self.create_subscription(String,'/fleet/inject_failure',self.inject,10)
                self.create_subscription(String,'/tasks/feedback',self.feedback,20)
                self.create_timer(.05,self.publish)

            def command(self,rid,msg):
                values=(msg.linear.x,msg.linear.y,msg.angular.z)
                if not all(math.isfinite(v) for v in values):
                    return
                with lock:
                    r=robots[rid]
                    r['cmd']=tuple(max(-1.,min(1.,v)) for v in values)
                    r['last_cmd']=time.monotonic()

            def assignment(self,rid,msg):
                data=json.loads(msg.data)
                with lock:
                    if data.get('action')=='cancel':
                        old=robots[rid]['assignment']
                        if old and old['assignment_id']==data.get('assignment_id'):
                            robots[rid]['assignment']=None
                            robots[rid]['cmd']=(0.,0.,0.)
                    else:
                        robots[rid]['assignment']=data

            def feedback(self,msg):
                data=json.loads(msg.data)
                rid=data['robot_id']
                with lock:
                    if rid in robots:
                        old=robots[rid]['assignment']
                        if old and old['assignment_id']==data.get('assignment_id') and data['status'] in ('completed','failed'):
                            robots[rid]['assignment']=None

            def inject(self,msg):
                data=json.loads(msg.data)
                if data.get('robot_id') not in robots:
                    return
                with lock:
                    self.pending_failures.append((data['robot_id'],bool(data['failed'])))

            def publish(self):
                with lock:
                    self.playback_pub.publish(Bool(data=state['playing']))
                    if not state['playing']:
                        return
                    for rid,r in robots.items():
                        if r['failed']:
                            continue
                        r['sequence']+=1
                        data={'robot_id':rid,'boot_id':boot,'sequence':r['sequence'],'x':r['x'],'y':r['y'],
                              'battery':r['battery'],'capabilities':r['capabilities'],'healthy':True,
                              'communication_available':True,'current_task':r['assignment']['task_id'] if r['assignment'] else None,
                              'control_mode':'kinematic cmd_vel demo; arms parked','energy_model':'0.1 battery percent per metre'}
                        self.pub[rid][0].publish(String(data=json.dumps(data)))
                        odom=Odometry()
                        odom.header.stamp=self.get_clock().now().to_msg()
                        odom.header.frame_id='map'
                        odom.child_frame_id=rid+'/base_link'
                        odom.pose.pose.position.x=float(r['x'])
                        odom.pose.pose.position.y=float(r['y'])
                        odom.pose.pose.orientation.z=math.sin(r['yaw']/2)
                        odom.pose.pose.orientation.w=math.cos(r['yaw']/2)
                        self.pub[rid][1].publish(odom)
        ros_node=BridgeNode()
        executor=rclpy.executors.SingleThreadedExecutor()
        executor.add_node(ros_node)
        worker=threading.Thread(target=executor.spin,daemon=True)
        worker.start()
        fixed=UsdGeom.Camera.Define(stage,'/World/FixedOverview')
        from omni.kit.viewport.utility import get_active_viewport
        viewport=get_active_viewport()
        if viewport:
            viewport.camera_path='/World/FixedOverview'
        set_camera_view(eye=[10,-14,11],target=[0,0,.5],camera_prim_path='/World/FixedOverview')
        args.output.parent.mkdir(parents=True,exist_ok=True)
        stage.GetRootLayer().Export(str(args.output.resolve()))
        timeline=omni.timeline.get_timeline_interface()
        timeline.play()
        start=previous=time.monotonic()
        injected=False
        while app.is_running() and (not args.seconds or time.monotonic()-start<args.seconds):
            app.update()
            now=time.monotonic()
            dt=min(.05,max(0.,now-previous))
            previous=now
            with lock:
                state['playing']=timeline.is_playing()
                if args.fail_robot and not injected and now-start>=args.fail_at:
                    ros_node.pending_failures.append((args.fail_robot,True))
                    injected=True
                for rid,failed in ros_node.pending_failures:
                    if rid not in robots:
                        continue
                    robots[rid]['failed']=failed
                    robots[rid]['cmd']=(0.,0.,0.)
                    for kind in ('RGB','Info'):
                        og.Controller.attribute(camera_graphs[rid]+'/'+kind+'.inputs:enabled').set(not failed)
                ros_node.pending_failures.clear()
                if not state['playing']:
                    continue
                for r in robots.values():
                    vx,vy,w=r['cmd'] if not r['failed'] and now-r['last_cmd']<.3 else (0.,0.,0.)
                    wx=math.cos(r['yaw'])*vx-math.sin(r['yaw'])*vy
                    wy=math.sin(r['yaw'])*vx+math.cos(r['yaw'])*vy
                    r['x']+=wx*dt
                    r['y']+=wy*dt
                    r['yaw']+=w*dt
                    r['battery']=max(0.,r['battery']-math.hypot(wx,wy)*dt*.1)
                    r['translation'].Set(Gf.Vec3d(r['x'],r['y'],.05))
                    r['rotation'].Set(math.degrees(r['yaw']))
        with lock:
            state['playing']=False
        ros_node.playback_pub.publish(Bool(data=False))
        time.sleep(.06)
        timeline.stop()
        print('ROS scene finished: '+json.dumps({rid:{k:r[k] for k in ('x','y','battery','failed')} for rid,r in robots.items()}),flush=True)
        executor.shutdown()
        worker.join(timeout=2)
        ros_node.destroy_node()
        ros_node=None
        rclpy.shutdown()
    except Exception:
        import traceback
        traceback.print_exc()
        app.app.post_quit(1)
        raise
    finally:
        if ros_node:
            try:
                ros_node.destroy_node()
            except Exception:
                pass
        app.close()


if __name__=='__main__':
    main()
