import mujoco
import mujoco.viewer
import time

mj_model = mujoco.MjModel.from_xml_path("/home/xander/Codes/MuJoCo/unitree_mujoco/unitree_robots/go1/scene.xml")
mj_data = mujoco.MjData(mj_model)


viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = 0.005

while viewer.is_running():
    mujoco.mj_step(mj_model, mj_data)
    viewer.sync()