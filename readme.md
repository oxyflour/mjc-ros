```bash
# setup
bash /scripts/ubuntu-setup.sh

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

git clone https://github.com/google-deepmind/mujoco_menagerie data/mujoco_menagerie-main
# do not use `ros2 run` because it can not find mujoco
mjc/node.py

wget --content-disposition 'https://api.ngc.nvidia.com/v2/resources/org/nvidia/team/isaac/r2bdataset2024/1/files?redirect=true&path=r2b_robotarm/r2b_robotarm_0.mcap' --output-document 'data/r2b_robotarm/r2b_robotarm_0.mcap'
# in another terminal
ros2 bag play data/r2b_robotarm/r2b_robotarm_0.mcap
```

