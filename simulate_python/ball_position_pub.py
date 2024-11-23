import numpy as np
from scipy.spatial.transform import Rotation as R

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point_
from unitree_sdk2py.idl.default import geometry_msgs_msg_dds__Point_ as Point_default


TOPIC_BALL_POSITION = "/ball_position"

class BallPositionPub():
    def __init__(self, mj_model, mj_data):
        
        self.mj_model = mj_model
        self.mj_data = mj_data
        
        self.ball_position = Point_default()
        self.ball_pos_puber = ChannelPublisher(TOPIC_BALL_POSITION, Point_)
        self.ball_pos_puber.Init()
        self.ballPosThread = RecurrentThread(
            interval=0.02,
            target=self.publish_ball_position,
            name="sim_ballpos"
        )
        self.ballPosThread.Start()

    def publish_ball_position(self):
        if self.mj_data != None:
            ball_pos = self.calc_ball_position()
            self.ball_position.x = ball_pos[0]
            self.ball_position.y = ball_pos[1]
            self.ball_position.z = 0.0
            self.ball_pos_puber.Write(self.ball_position)
            # print(self.ball_position)
        
    def calc_ball_position(self):
        base_link_pos = np.array(self.mj_data.xpos[1])
        base_link_quat = np.array(self.mj_data.xquat[1])
        ball_pos = np.array(self.mj_data.xpos[-1])
        
        rotation_matrix = R.from_quat(base_link_quat, scalar_first=True).as_matrix()
        relative_pos_world = ball_pos - base_link_pos
        ball_pos_robot = rotation_matrix.T @ relative_pos_world

        return ball_pos_robot.tolist()
