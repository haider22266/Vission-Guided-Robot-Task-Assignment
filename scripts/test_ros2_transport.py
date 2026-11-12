"""Real Humble transport test with separate node processes and mock robot I/O.

No trained-model, Isaac rendering or physical execution claim is made.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('ROS_DOMAIN_ID','43')
os.environ.setdefault('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
sys.path.insert(0,str(ROOT/'src'/'disaster_mrta'))


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String, Bool
    from sensor_msgs.msg import Image
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    directory=ROOT/'results'/'ros2_transport_test'
    directory.mkdir(parents=True,exist_ok=True)
    tasks=directory/'tasks.json'
    tasks.write_text(json.dumps([dict(task_id='0',task_type='area_search',x=1.,y=0.,priority=.9,
                                    required_capability='camera',estimated_cost=.2)]))
    common=['--ros-args','-p','robot_count:=2','-p','allocator_backend:=baseline','-p','perception_backend:=mock',
            '-p','execution_backend:=timed_demo','-p',f'tasks_file:={tasks}',
            '-p',f'metrics_directory:={directory}']
    processes=[]
    files=[]
    env=os.environ.copy()
    env['PYTHONPATH']=str(ROOT/'src'/'disaster_mrta')+os.pathsep+env.get('PYTHONPATH','')
    try:
        for label,entry,extra in [('allocator','allocator_main',[]),('monitor','failure_main',[]),
                                  ('perception','perception_main',[]),('metrics','metrics_main',[]),
                                  ('controller_1','controller_main',['-p','robot_id:=robot_1']),
                                  ('controller_2','controller_main',['-p','robot_id:=robot_2'])]:
            log=(directory/(label+'.log')).open('w')
            files.append(log)
            processes.append(subprocess.Popen([sys.executable,'-c',f'from disaster_mrta.ros_nodes import {entry}; {entry}()']+common+extra,
                                               stdout=log,stderr=subprocess.STDOUT,env=env))
        rclpy.init()
        node=Node('transport_test_fixture')
        robots={rid:{'x':0.,'y':i*.5,'cmd':(0.,0.,0.),'sequence':0} for i,rid in enumerate(('robot_1','robot_2'))}
        publishers={rid:(node.create_publisher(String,f'/{rid}/status',10),
                         node.create_publisher(Odometry,f'/{rid}/odom',10),
                         node.create_publisher(Image,f'/{rid}/camera/rgb',qos_profile_sensor_data)) for rid in robots}
        playback=node.create_publisher(Bool,'/simulation/playback',10)
        assignments=[]
        feedback=[]
        visions=[]
        failures=[]
        keep=[]
        for rid in robots:
            keep.append(node.create_subscription(Twist,f'/{rid}/cmd_vel',lambda m,r=rid:robots[r].__setitem__('cmd',(m.linear.x,m.linear.y,m.angular.z)),10))
            keep.append(node.create_subscription(String,f'/{rid}/task_assignment',lambda m:assignments.append((time.monotonic(),json.loads(m.data))),10))
        keep.append(node.create_subscription(String,'/tasks/feedback',lambda m:feedback.append(json.loads(m.data)),20))
        keep.append(node.create_subscription(String,'/tasks/vision_result',lambda m:visions.append(json.loads(m.data)),10))
        keep.append(node.create_subscription(String,'/fleet/failure',lambda m:failures.append(json.loads(m.data)),10))
        begin=last=last_publish=time.monotonic()
        failed=None
        active_since=None
        ready=False
        pause_start=None
        pause_done=False
        failure_after=None
        success=False
        while time.monotonic()-begin<35:
            rclpy.spin_once(node,timeout_sec=.01)
            now=time.monotonic()
            dt=now-last
            last=now
            if any(p.poll() is not None for p in processes):
                raise RuntimeError(f'A ROS node exited; inspect {directory}')
            if not ready:
                ready=all(p[0].get_subscription_count()>=3 for p in publishers.values())
                if not ready:
                    continue
            accepted=[(t,a) for t,a in assignments if a.get('action')!='cancel']
            if accepted and pause_start is None:
                pause_start=now
            if pause_start is not None and not pause_done:
                if now-pause_start<.5:
                    playback.publish(Bool(data=False))
                    continue
                assert not failures, 'An intentional playback pause was treated as robot failure'
                pause_done=True
                failure_after=now+.2
                playback.publish(Bool(data=True))
            if accepted and failed is None:
                active_since,first=accepted[0]
                if failure_after is not None and now>failure_after:
                    failed=first['robot_id']
            if now-last_publish<.05:
                continue
            last_publish=now
            for rid,r in robots.items():
                if rid==failed:
                    continue
                vx,vy,_=r['cmd']
                r['x']+=vx*dt*5
                r['y']+=vy*dt*5
                r['sequence']+=1
                status={'robot_id':rid,'boot_id':'fixture','sequence':r['sequence'],'x':r['x'],'y':r['y'],
                        'battery':100.,'capabilities':['camera'],'healthy':True,'communication_available':True}
                publishers[rid][0].publish(String(data=json.dumps(status)))
                odom=Odometry()
                odom.pose.pose.position.x=r['x']
                odom.pose.pose.position.y=r['y']
                odom.pose.pose.orientation.w=1.
                publishers[rid][1].publish(odom)
                image=Image()
                image.header.stamp=node.get_clock().now().to_msg()
                image.height=image.width=8
                image.encoding='rgb8'
                image.step=24
                image.data=bytes([120]*8*8*3)
                publishers[rid][2].publish(image)
            done=[f for f in feedback if f['status']=='completed']
            reassigned=[a for _,a in accepted if a['robot_id']!=failed]
            if failed and reassigned and done and visions and failures:
                assert done[-1]['assignment_id']==reassigned[-1]['assignment_id']
                assert visions[-1]['source']=='mock'
                success=True
                print('PASS: ROS image transport, perception, unique assignment, pause handling, failure detection, reassignment and timed-demo completion',flush=True)
                break
        node.destroy_node()
        rclpy.shutdown()
        if not success:
            raise RuntimeError(f'Transport integration test timed out; inspect {directory}')
    finally:
        for process in processes:
            process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for file in files:
            file.close()


if __name__=='__main__':
    main()
