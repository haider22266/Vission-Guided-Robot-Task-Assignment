from glob import glob
from setuptools import find_packages, setup

setup(name='disaster_mrta', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/disaster_mrta']),
                  ('share/disaster_mrta', ['package.xml']),
                  ('share/disaster_mrta/config', glob('config/*.yaml')),
                  ('share/disaster_mrta/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools', 'PyYAML'], zip_safe=True,
      maintainer='Research team', maintainer_email='maintainer@example.com',
      description='Vision-guided fault-tolerant MRTA framework', license='Apache-2.0',
      entry_points={'console_scripts': ['validate_config = disaster_mrta.config_loader:main',
          'image_perception = disaster_mrta.ros_nodes:perception_main',
          'qmix_allocator = disaster_mrta.ros_nodes:allocator_main',
          'failure_monitor = disaster_mrta.ros_nodes:failure_main',
          'robot_task_controller = disaster_mrta.ros_nodes:controller_main',
          'evaluation_recorder = disaster_mrta.ros_nodes:metrics_main']})
