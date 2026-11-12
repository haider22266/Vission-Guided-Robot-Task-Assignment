"""Verify real Isaac camera/status transport and cmd_vel motion from WSL Humble."""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node=Node('isaac_camera_transport_probe')
    counts={'rgb':0,'camera_info':0,'odom':0,'status':0}
    poses=[]
    def count(kind,msg):
        counts[kind]+=1
        if kind=='odom':
            poses.append((msg.pose.pose.position.x,msg.pose.pose.position.y))
    keep=[node.create_subscription(Image,'/robot_1/camera/rgb',lambda m:count('rgb',m),qos_profile_sensor_data),
          node.create_subscription(CameraInfo,'/robot_1/camera/camera_info',lambda m:count('camera_info',m),qos_profile_sensor_data),
          node.create_subscription(Odometry,'/robot_1/odom',lambda m:count('odom',m),10),
          node.create_subscription(String,'/robot_1/status',lambda m:count('status',m),10)]
    publisher=node.create_publisher(Twist,'/robot_1/cmd_vel',10)
    start=last=time.monotonic()
    success=False
    try:
        while time.monotonic()-start<50:
            rclpy.spin_once(node,timeout_sec=.02)
            now=time.monotonic()
            if now-last>.08 and poses:
                cmd=Twist()
                cmd.linear.x=.2
                publisher.publish(cmd)
                last=now
            if min(counts.values())>=2 and len(poses)>2 and math.dist(poses[0],poses[-1])>.08:
                success=True
                break
        result={'counts':counts,'displacement_m':math.dist(poses[0],poses[-1]) if len(poses)>1 else 0.,'success':success}
        path=Path(__file__).resolve().parents[1]/'results'/'isaac_ros_transport.json'
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(result,indent=2))
        print(json.dumps(result),flush=True)
        if not success:
            raise RuntimeError('Windows/WSL DDS transport did not meet camera/state/motion checks')
    finally:
        publisher.publish(Twist())
        rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
