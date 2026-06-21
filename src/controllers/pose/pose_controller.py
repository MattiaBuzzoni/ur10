import numpy as np
from src.controllers.controller import Controller
from src.controllers.pose.kinematics import Kinematics
from src.model.robots import motor


class PoseController(Controller):
    MOTOR_CONTROL_MODE = motor.MOTOR_CONTROL_POSITION

    def __init__(self, robot, get_time_since_reset):
        super().__init__(robot, get_time_since_reset)
        self._constants = robot.get_ctrl_constants()
        self._kinematics = Kinematics(robot)
        
        self._base_position    = np.array(self._robot.ee_effector_position(6))
        self._base_orientation = np.array(self._robot.ee_effector_orientation(6))
        self._position         = self._base_position.copy()
        self._orientation      = self._base_orientation.copy()

    def update_controller_params(self, params):
        self._position, self._orientation = params

    def setup_ui_params(self, pybullet_client):
        base_x = pybullet_client.addUserDebugParameter("delta_x", -.99, .99, 0.)
        base_y = pybullet_client.addUserDebugParameter("delta_y", -.99, .99, 0.)
        base_z = pybullet_client.addUserDebugParameter("delta_z", -.99, .99, 0.)
        roll   = pybullet_client.addUserDebugParameter("roll",  -np.pi / 2, np.pi / 2, 0)
        pitch  = pybullet_client.addUserDebugParameter("pitch", -np.pi / 2, np.pi / 2, 0)
        yaw    = pybullet_client.addUserDebugParameter("yaw",   -np.pi / 2, np.pi / 2, 0)
        return base_x, base_y, base_z, roll, pitch, yaw

    def read_ui_params(self, pybullet_client, ui):
        base_x, base_y, base_z, roll, pitch, yaw = ui
        
        delta = np.array([
            pybullet_client.readUserDebugParameter(base_x),
            pybullet_client.readUserDebugParameter(base_y),
            pybullet_client.readUserDebugParameter(base_z),
        ])
        position = self._base_position + delta

        delta_rpy = np.array([
            pybullet_client.readUserDebugParameter(roll),
            pybullet_client.readUserDebugParameter(pitch),
            pybullet_client.readUserDebugParameter(yaw),
        ])
        
        base_rpy = np.array(pybullet_client.getEulerFromQuaternion(
            self._base_orientation
        ))
        orientation = base_rpy + delta_rpy   

        return position, orientation

    def reset(self):
        pass

    def get_action(self):

        target_quat = self._robot.pybullet_client.getQuaternionFromEuler(
            self._orientation
        )

        joint_angles = self._kinematics.solve_IK(self._position, target_quat, 6)
        
        return np.array(joint_angles[:6])