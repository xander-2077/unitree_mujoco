import mujoco
import numpy as np
import sys, os, time

mj_model = mujoco.MjModel.from_xml_path("/home/xander/Codes/MuJoCo/unitree_mujoco/unitree_robots/go1/scene_ball.xml")
mj_data = mujoco.MjData(mj_model)


# 获取关节名称和顺序
joint_names = []
for joint_id in range(mj_model.njnt):
    name_address = mj_model.name_jntadr[joint_id]
    joint_name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
    joint_names.append(joint_name)

# 打印关节名称和顺序
for i, joint_name in enumerate(joint_names):
    print(f"Joint {i}: {joint_name}")

breakpoint()

while True:
    step_start = time.perf_counter()
    
    mujoco.mj_step(mj_model, mj_data)
    
    base_link_pos = np.array(mj_data.xpos[1])
    base_link_quat = np.array(mj_data.xquat[1])
    ball_pos = np.array(mj_data.xpos[-1])

    time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)

    breakpoint()

