"""ROS 2 adapters. Each entry point runs in its own process."""
from dataclasses import asdict
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from .pipeline_core import FleetState, FailureTracker, baseline_actions


def parameter(node,name,default):
    if not node.has_parameter(name):
        node.declare_parameter(name,default)
    return node.get_parameter(name).value


def send(publisher,data):
    publisher.publish(String(data=json.dumps(data,allow_nan=False)))


def ids(node):
    count=int(parameter(node,'robot_count',5))
    if not 1<=count<=5:
        raise ValueError('robot_count must be between one and five')
    return [f'robot_{i+1}' for i in range(count)]


def default_tasks():
    caps=['camera','inspection','delivery','inspection','camera']
    kinds=['area_search','hazard_inspection','supply_delivery','hazard_inspection','area_search']
    return [dict(task_id=str(i),task_type=kinds[i],x=(i-2)*2.,y=2.,priority=.9-i*.05,
                 required_capability=caps[i],estimated_cost=2.,route_safe=(i==2)) for i in range(5)]


class AllocatorNode(Node):
    def __init__(self):
        super().__init__('qmix_allocator',automatically_declare_parameters_from_overrides=True)
        robot_ids=ids(self)
        tasks_path=parameter(self,'tasks_file','')
        tasks=json.loads(Path(tasks_path).read_text()) if tasks_path else default_tasks()[:len(robot_ids)]
        self.fleet=FleetState(robot_ids,tasks,float(parameter(self,'min_battery',20.)),float(parameter(self,'max_distance',50.)))
        backend=parameter(self,'allocator_backend','qmix')
        self.backend=backend
        if backend=='qmix':
            from .qmix_policy import QmixPolicy
            self.policy=QmixPolicy(parameter(self,'qmix_checkpoint',''),len(robot_ids),int(parameter(self,'max_tasks',8)))
        elif backend!='baseline':
            raise ValueError('allocator_backend must be qmix or baseline')
        self.assignment_pubs={rid:self.create_publisher(String,f'/{rid}/task_assignment',10) for rid in robot_ids}
        self.events=self.create_publisher(String,'/pipeline/events',100)
        self.state_pub=self.create_publisher(String,'/tasks/state',10)
        self.subscriptions_keep=[]
        self.failed_at={}
        self.playing=True
        self.create_subscription(Bool,'/simulation/playback',lambda m:setattr(self,'playing',m.data),10)
        for rid in robot_ids:
            self.subscriptions_keep.append(self.create_subscription(String,f'/{rid}/status',self.on_status,10))
        self.create_subscription(String,'/tasks/vision_result',self.on_vision,10)
        self.create_subscription(String,'/tasks/feedback',self.on_feedback,50)
        self.create_subscription(String,'/fleet/failure',self.on_failure,10)
        self.create_subscription(String,'/tasks/route_status',self.on_route,10)
        self.create_timer(.1,self.allocate)
        self.get_logger().info(f'Allocator backend={backend}; baseline is not a trained QMIX policy')

    def on_status(self,msg):
        try:
            data=json.loads(msg.data)
            old_boot=self.fleet.robots[data['robot_id']].boot_id
            accepted=self.fleet.status(data)
            if accepted and old_boot and old_boot!=self.fleet.robots[data['robot_id']].boot_id:
                self.on_failure(String(data=json.dumps({'robot_id':data['robot_id'],'reason':'robot process restarted'})))
        except (ValueError,KeyError,TypeError) as exc:
            self.get_logger().error(f'Invalid status: {exc}')

    def on_vision(self,msg):
        try:
            self.fleet.vision(json.loads(msg.data))
        except (ValueError,KeyError,TypeError) as exc:
            self.get_logger().error(f'Invalid vision result: {exc}')

    def on_route(self,msg):
        try:
            data=json.loads(msg.data)
            task=self.fleet.tasks[str(data['task_id'])]
            task.route_safe=data['safe'] is True
            if not task.route_safe and task.task_type=='supply_delivery':
                cancel=self.fleet.cancel_task(task.task_id,'Planner marked delivery route unsafe')
                if cancel:
                    send(self.assignment_pubs[cancel['robot_id']],cancel)
                    send(self.events,dict(cancel,event='task_cancelled'))
        except (ValueError,KeyError,TypeError) as exc:
            self.get_logger().error(f'Invalid route status: {exc}')

    def on_feedback(self,msg):
        try:
            data=json.loads(msg.data)
            if self.fleet.feedback(data):
                send(self.events,dict(data,event='task_'+data['status'],allocator_backend=self.backend))
        except (ValueError,KeyError,TypeError) as exc:
            self.get_logger().error(f'Invalid controller feedback: {exc}')

    def on_failure(self,msg):
        try:
            data=json.loads(msg.data)
            now=time.monotonic()
            cancel=self.fleet.release_robot(data['robot_id'],data['reason'],now)
            if cancel:
                self.failed_at[cancel['task_id']]=now
                send(self.assignment_pubs[data['robot_id']],cancel)
            send(self.events,dict(data,event='failure_detected',task_id=cancel['task_id'] if cancel else None))
            self.allocate()
        except (ValueError,KeyError,TypeError) as exc:
            self.get_logger().error(f'Invalid failure event: {exc}')

    def allocate(self):
        if not self.playing:
            return
        actions=self.policy.actions(self.fleet) if self.backend=='qmix' else baseline_actions(self.fleet)
        tasks=list(self.fleet.tasks.values())
        for rid,action in zip(self.fleet.robots,actions):
            if not action:
                continue
            assignment=self.fleet.commit(rid,tasks[action-1].task_id)
            if assignment:
                send(self.assignment_pubs[rid],assignment)
                event=dict(assignment,event='task_assigned',allocator_backend=self.backend)
                if assignment['task_id'] in self.failed_at:
                    event['reassignment_seconds']=time.monotonic()-self.failed_at.pop(assignment['task_id'])
                send(self.events,event)
        send(self.state_pub,{'backend':self.backend,'tasks':[asdict(t) for t in tasks],
                            'robots':[asdict(r) for r in self.fleet.robots.values()]})


class FailureMonitorNode(Node):
    def __init__(self):
        super().__init__('failure_monitor',automatically_declare_parameters_from_overrides=True)
        robot_ids=ids(self)
        self.fleet=FleetState(robot_ids,[])
        self.tracker=FailureTracker(robot_ids,int(parameter(self,'missed_updates',3)))
        self.publisher=self.create_publisher(String,'/fleet/failure',10)
        self.playing=True
        self.create_subscription(Bool,'/simulation/playback',lambda m:setattr(self,'playing',m.data),10)
        self.subscriptions_keep=[self.create_subscription(String,f'/{rid}/status',self.status,10) for rid in robot_ids]
        self.create_timer(float(parameter(self,'monitor_period',.1)),self.tick)

    def status(self,msg):
        try:
            self.fleet.status(json.loads(msg.data))
        except (ValueError,KeyError,TypeError) as exc:
            self.get_logger().error(str(exc))

    def tick(self):
        if not self.playing:
            for rid in self.tracker.missed:
                self.tracker.missed[rid]=0
            return
        for rid in self.tracker.tick(self.fleet.robots):
            send(self.publisher,{'robot_id':rid,'reason':'three missed status updates or explicit health/communication failure',
                                 'missed_updates':self.tracker.missed[rid]})


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('image_perception',automatically_declare_parameters_from_overrides=True)
        from cv_bridge import CvBridge
        from .perception_model import EfficientNetModel, CLASSES
        self.bridge=CvBridge()
        self.backend=parameter(self,'perception_backend','efficientnet')
        self.classes=list(parameter(self,'class_names',list(CLASSES)))
        self.model=None
        if self.backend=='efficientnet':
            self.model=EfficientNetModel(parameter(self,'efficientnet_model',''),self.classes,
                                        parameter(self,'input_range','0_255'),parameter(self,'model_output','probabilities'))
        elif self.backend!='mock':
            raise ValueError('perception_backend must be efficientnet or mock')
        robot_ids=ids(self)
        areas=list(parameter(self,'camera_area_map',[str(i) for i in range(len(robot_ids))]))
        if len(areas)<len(robot_ids):
            raise ValueError('camera_area_map needs one calibrated area ID per camera')
        self.areas=dict(zip(robot_ids,areas))
        self.latest={}
        self.future=None
        self.inference_pool=ThreadPoolExecutor(max_workers=1)
        self.publisher=self.create_publisher(String,'/tasks/vision_result',10)
        self.subscriptions_keep=[self.create_subscription(Image,f'/{rid}/camera/rgb',lambda msg,r=rid:self.latest.__setitem__(r,msg),qos_profile_sensor_data) for rid in robot_ids]
        self.create_timer(.1,self.tick)
        self.get_logger().info(f'Perception backend={self.backend}; whole-image area clues only')

    def infer(self,rid,msg):
        from .perception_model import vision_result
        if self.backend=='mock':
            values=[float(name=='Non-Damage') for name in self.classes]
        else:
            rgb=self.bridge.imgmsg_to_cv2(msg,desired_encoding='rgb8')
            values=self.model.predict(rgb)
        result=vision_result(values,self.classes,self.areas[rid],rid,self.backend)
        result['image_stamp']={'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec}
        result['area_mapping']='configured camera-to-area association, not image localization'
        return result

    def tick(self):
        if self.future:
            if not self.future.done():
                return
            try:
                send(self.publisher,self.future.result())
            except Exception as exc:
                self.get_logger().error(f'Inference failed: {exc}')
            self.future=None
        if self.latest:
            rid=next(iter(self.latest))
            message=self.latest.pop(rid)
            self.future=self.inference_pool.submit(self.infer,rid,message)

    def destroy_node(self):
        self.inference_pool.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


class TaskControllerNode(Node):
    def __init__(self):
        super().__init__('task_controller',automatically_declare_parameters_from_overrides=True)
        self.rid=parameter(self,'robot_id','robot_1')
        self.backend=parameter(self,'execution_backend','external')
        if self.backend not in ('external','timed_demo'):
            raise ValueError('execution_backend must be external or timed_demo')
        self.speed=float(parameter(self,'max_speed',.5))
        self.tolerance=float(parameter(self,'goal_tolerance',.12))
        self.timeout=float(parameter(self,'task_timeout',120.))
        self.publisher=self.create_publisher(Twist,f'/{self.rid}/cmd_vel',10)
        self.feedback_pub=self.create_publisher(String,'/tasks/feedback',20)
        self.execute_pub=self.create_publisher(String,f'/{self.rid}/execute_task',10)
        self.create_subscription(String,f'/{self.rid}/task_assignment',self.assignment,10)
        self.create_subscription(Odometry,f'/{self.rid}/odom',self.odom,10)
        self.create_subscription(String,f'/{self.rid}/status',self.status,10)
        self.create_subscription(String,f'/{self.rid}/task_result',self.external_result,10)
        self.playing=True
        self.paused_at=None
        self.create_subscription(Bool,'/simulation/playback',self.playback,10)
        self.task=None
        self.position=None
        self.last_state=0.
        self.healthy=False
        self.execute_at=None
        self.create_timer(.1,self.tick)

    def stop(self):
        self.publisher.publish(Twist())

    def playback(self,msg):
        now=time.monotonic()
        if not msg.data and self.playing:
            self.paused_at=now
            self.stop()
        elif msg.data and self.paused_at is not None:
            duration=now-self.paused_at
            if self.task:
                self.assigned_at+=duration
                if self.execute_at is not None:
                    self.execute_at+=duration
            self.paused_at=None
        self.playing=msg.data

    def assignment(self,msg):
        data=json.loads(msg.data)
        if data.get('action')=='cancel':
            if self.task and self.task['assignment_id']==data['assignment_id']:
                send(self.execute_pub,dict(self.task,action='cancel'))
                self.task=None
                self.stop()
            return
        if self.task and self.task['assignment_id']==data['assignment_id']:
            return
        if self.task:
            self.stop()
            send(self.execute_pub,dict(self.task,action='cancel'))
        self.task=data
        self.assigned_at=time.monotonic()
        self.execute_at=None

    def status(self,msg):
        data=json.loads(msg.data)
        self.last_state=time.monotonic()
        self.healthy=data['healthy'] and data['communication_available'] and data['battery']>=20.

    def odom(self,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        self.position=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

    def feedback(self,status,reason=''):
        if self.task:
            send(self.feedback_pub,{k:self.task[k] for k in ('robot_id','task_id','assignment_id')}|
                 {'status':status,'reason':reason,'execution_backend':self.backend})

    def external_result(self,msg):
        data=json.loads(msg.data)
        if (self.task and data.get('assignment_id')==self.task['assignment_id']
                and data.get('status') in ('completed','failed')
                and (data['status']=='failed' or self.execute_at is not None)):
            self.feedback(data['status'],data.get('reason','external executor result'))
            self.task=None
            self.stop()

    def tick(self):
        now=time.monotonic()
        if not self.playing:
            self.stop()
            return
        if not self.task:
            self.stop()
            return
        if now-self.last_state>.3:
            # The failure monitor owns reassignment after its three missed ticks.
            self.stop()
            return
        if not self.healthy or now-self.assigned_at>self.timeout:
            self.stop()
            self.feedback('failed','status watchdog, low battery, health failure or task timeout')
            send(self.execute_pub,dict(self.task,action='cancel'))
            self.task=None
            return
        if self.position is None:
            self.stop()
            return
        x,y,yaw=self.position
        dx,dy=self.task['x']-x,self.task['y']-y
        distance=math.hypot(dx,dy)
        if distance>self.tolerance:
            cmd=Twist()
            scale=min(self.speed,distance)/distance
            cmd.linear.x=(math.cos(yaw)*dx+math.sin(yaw)*dy)*scale
            cmd.linear.y=(-math.sin(yaw)*dx+math.cos(yaw)*dy)*scale
            heading=math.atan2(dy,dx)-yaw
            cmd.angular.z=max(-.7,min(.7,math.atan2(math.sin(heading),math.cos(heading))))
            self.publisher.publish(cmd)
            return
        self.stop()
        if self.execute_at is None:
            self.execute_at=now
            self.feedback('in_progress')
            send(self.execute_pub,dict(self.task,action='start'))
        if self.backend=='timed_demo' and now-self.execute_at>=self.task['estimated_cost']:
            self.feedback('completed','Explicit timed integration demo, not physical task execution')
            self.task=None


class MetricsNode(Node):
    def __init__(self):
        super().__init__('evaluation_recorder',automatically_declare_parameters_from_overrides=True)
        directory=Path(parameter(self,'metrics_directory','results/ros2'))
        directory.mkdir(parents=True,exist_ok=True)
        self.file=(directory/f'events_{time.time_ns()}.jsonl').open('w',encoding='utf-8')
        self.summary_file=directory/'latest_summary.json'
        self.tasks={}
        self.completed=set()
        self.started={}
        self.durations={}
        self.battery={}
        self.energy=0.
        self.reassignments=[]
        self.correct=self.labeled=0
        self.create_subscription(String,'/pipeline/events',self.event,100)
        self.create_subscription(String,'/tasks/state',self.state,10)
        self.create_subscription(String,'/tasks/vision_result',self.vision,10)
        self.subscriptions_keep=[self.create_subscription(String,f'/{rid}/status',self.status,10) for rid in ids(self)]
        self.create_timer(1.,self.flush_summary)

    def record(self,kind,data):
        self.file.write(json.dumps({'received_wall_time':time.time(),'kind':kind,'data':data},allow_nan=False)+'\n')
        self.file.flush()

    def event(self,msg):
        data=json.loads(msg.data)
        self.record('event',data)
        task=data.get('task_id')
        if data['event']=='task_assigned':
            self.started.setdefault(task,time.monotonic())
            if 'reassignment_seconds' in data:
                self.reassignments.append(data['reassignment_seconds'])
        elif data['event']=='task_completed':
            self.completed.add(task)
            if task in self.started:
                self.durations[task]=time.monotonic()-self.started[task]

    def state(self,msg):
        data=json.loads(msg.data)
        self.tasks={t['task_id']:t for t in data['tasks']}

    def vision(self,msg):
        data=json.loads(msg.data)
        self.record('vision',data)
        if data.get('ground_truth') is not None and data.get('source')!='mock':
            self.labeled+=1
            self.correct+=data['predicted_class']==data['ground_truth']

    def status(self,msg):
        data=json.loads(msg.data)
        rid=data['robot_id']
        value=float(data['battery'])
        if rid in self.battery:
            self.energy+=max(0.,self.battery[rid]-value)
        self.battery[rid]=value

    def flush_summary(self):
        summary={'task_completion_rate':len(self.completed)/len(self.tasks) if self.tasks else None,
                 'task_completion_seconds':self.durations,'estimated_battery_percent_used':self.energy,
                 'detection_accuracy':self.correct/self.labeled if self.labeled else None,
                 'labeled_predictions':self.labeled,'reassignment_seconds':self.reassignments,
                 'note':'Timing uses recorder/allocator wall clocks; battery is simulator-estimated. Detection accuracy needs labeled real predictions.'}
        self.summary_file.write_text(json.dumps(summary,indent=2),encoding='utf-8')

    def destroy_node(self):
        self.flush_summary()
        self.file.close()
        return super().destroy_node()


def run(node_class):
    rclpy.init()
    node=None
    try:
        node=node_class()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node:
            if isinstance(node,TaskControllerNode) and rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def allocator_main(): run(AllocatorNode)
def perception_main(): run(PerceptionNode)
def failure_main(): run(FailureMonitorNode)
def controller_main(): run(TaskControllerNode)
def metrics_main(): run(MetricsNode)
