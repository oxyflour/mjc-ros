#!/usr/bin/env python3
import threading
import time
from typing import Dict

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import mujoco
from mujoco.glfw import glfw

MODEL_XML_PATH = "data/mujoco_menagerie-main/universal_robots_ur5e/ur5e.xml"  # TODO: 改成你的模型路径

class MujocoRosBridge(Node):
    def __init__(self, model, data, joint_name_to_qpos: Dict[str, int]):
        super().__init__("mujoco_ros_bridge")

        self.model = model
        self.data = data
        self.joint_name_to_qpos = joint_name_to_qpos

        # 存储最新目标位置
        self.target_qpos = np.copy(data.qpos)
        self.lock = threading.Lock()

        # 订阅 ROS2 话题
        self.sub = self.create_subscription(
            JointState,
            "/joint_states",   # <-- 改成你的话题名
            self.joint_targets_callback,
            10,
        )

        self.get_logger().info("MujocoRosBridge node initialized.")

    def joint_targets_callback(self, msg: JointState):
        """把 JointState 的目标位置写入 target_qpos"""
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self.joint_name_to_qpos:
                    idx = self.joint_name_to_qpos[name]
                    self.target_qpos[idx] = pos
                else:
                    # 第一次发现未映射的关节，可以打印一下
                    self.get_logger().warn(f"Unknown joint name from ROS: {name}")

    def step_simulation(self, dt: float = 0.002):
        """每个 step，把 target_qpos 写入 data.qpos，然后 mj_step"""
        with self.lock:
            # 简单直接：把 qpos = target_qpos
            # 你也可以做插值或 PD 控制
            self.data.qpos[:] = self.target_qpos[:]

        mujoco.mj_step(self.model, self.data)

def build_joint_name_map(model) -> Dict[str, int]:
    """把关节名映射到 qpos index（只考虑有自由度的关节）"""
    joint_name_to_qpos = {}

    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if name is None:
            continue

        qpos_adr = model.jnt_qposadr[j]
        # 对于 1 自由度关节，nv=1，qpos 就一个 index
        # 对于球关节等多自由度，你可以扩展映射逻辑
        joint_name_to_qpos[name] = qpos_adr

    return joint_name_to_qpos

def main():
    # 1. 加载 MuJoCo 模型
    model = mujoco.MjModel.from_xml_path(MODEL_XML_PATH)
    data = mujoco.MjData(model)

    joint_name_to_qpos = build_joint_name_map(model)
    print("Joint map:", joint_name_to_qpos)

    # 2. 初始化 ROS2
    rclpy.init()
    node = MujocoRosBridge(model, data, joint_name_to_qpos)

    # 3. 用线程跑 ROS2 spin，主线程跑 MuJoCo 仿真
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # 4. 创建一个简单的可视化窗口（不想要 GUI 的话可以去掉这块）
    glfw.init()
    window = glfw.create_window(1200, 900, "MuJoCo + ROS2", None, None)
    glfw.make_context_current(window)

    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    scn = mujoco.MjvScene(model, maxgeom=10000)
    con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

    mujoco.mjv_defaultCamera(cam)
    mujoco.mjv_defaultOption(opt)

    last_time = time.time()
    dt = 0.002  # 500Hz 仿真

    try:
        while not glfw.window_should_close(window):
            now = time.time()
            if now - last_time >= dt:
                last_time = now
                node.step_simulation(dt)

            # 渲染
            viewport_width, viewport_height = glfw.get_framebuffer_size(window)
            viewport = mujoco.MjrRect(0, 0, viewport_width, viewport_height)

            mujoco.mjv_updateScene(
                model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn
            )
            mujoco.mjr_render(viewport, scn, con)

            glfw.swap_buffers(window)
            glfw.poll_events()

    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()
        glfw.terminate()

if __name__ == "__main__":
    main()
