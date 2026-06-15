"""Generic class for wheeled mobile robots."""

from __future__ import annotations

import numpy as np

from src.utils import pybullet_data

class Robot:

    def __init__(
        self, 
        pybullet_client,
        mark,
        simulation,
        z_offset=0.  
    ):
        
        self._pybullet_client = pybullet_client
        self._simulation = simulation
        self._z_offset = z_offset
        self._mark = mark
        self._marks = self.get_marks()
        self._constants = self.get_constants()
        # ...

        # Load robot URDF
        self._robotic_arm = self._load_urdf()
        self._dh_params = self._get_dh_params()
        self._build_joint_name_to_dict()
        # ...

        self.reset_pose()

        # ...


    @property 
    def pybullet_client(self):
        return self._pybullet_client
    
    @property 
    def get_robot_id(self):
        return self._robotic_arm
    
    @property
    def dh_params(self):
        return self._dh_params
    
    
    def set_up_discrete_action_space(self):
        pass

    def set_up_continuous_action_space(self):
        pass

    def get_base_position(self):
        """Get the position of the robot's base."""
        position, _ = (self._pybullet_client.getBasePositionAndOrientation(self._robotic_arm))

        return position
    
    def get_base_roll_pitch_yaw(self):
        """Get the orientation of the robot's base."""
        _, orient = (self._pybullet_client.getBasePositionAndOrientation(self._robotic_arm))
        orient = self._pybullet_client.getEulerFromQuaternion(orient)

        return orient
    
    def get_base_roll_pitch_yaw_rate(self):
        """Get the rate of orientation change of the minitaur's base in euler angle."""
        angular_velocity = self._pybullet_client.getBaseVelocity(self._robotic_arm)[1]
        orientation = self.get_base_orientation()

        return self.transform_angular_velocity_to_local_frame(angular_velocity, orientation)
    
    
    def get_base_orientation(self):
        pos, orn = self._pybullet_client.getBasePositionAndOrientation(
            self._robotic_arm
        )

        return orn
    
    
    def _load_urdf(self):
        x, y, z = self._constants.START_POS
        start_position = [x, y, z]
        start_orientation = self._pybullet_client.getQuaternionFromEuler(
        self._constants.INIT_ORIENTATION,
    )

        return self._pybullet_client.loadURDF(
            f"{pybullet_data.getDataPath()}/{self._marks.MARK_PARAMS[self._mark]['urdf_name']}", 
            start_position, 
            start_orientation,
            useFixedBase=True,
        )

    def _build_joint_name_to_dict(self):
        num_joints = self._pybullet_client.getNumJoints(self._robotic_arm)
        self._joint_name_to_id = {}
        for i in range(num_joints):
            joint_info = self._pybullet_client.getJointInfo(self._robotic_arm, i)
            self._joint_name_to_id[joint_info[1].decode("UTF-8").replace("${namespace}", "")] = joint_info[0]

    def _get_dh_params(self):

        from prettytable import PrettyTable
        
        table = PrettyTable()
        table.title = "Denavit–Hartenberg parameters"
        table.field_names = ["Kinematics", "a[m]", "d[m]", "alpha[rad]", "Dynamics"]
        for i in range(1, 7):
            table.add_row(["Joint " + str(i), 
                           self._constants.a[i-1], 
                           self._constants.d[i-1], 
                           self._constants.alpha[i-1],
                           "Link " + str(i-1)])

        return table
    
    def get_joint_info(self):
        lower_limits, upper_limits, joint_ranges = [], [], []
        for j in range(self._pybullet_client.getNumJoints(self._robotic_arm)):
            info = self._pybullet_client.getJointInfo(self._robotic_arm, j)
            if info[2] != self._pybullet_client.JOINT_FIXED:
                lower_limits.append(info[8])
                upper_limits.append(info[9])
                joint_ranges.append(info[9] - info[8])

        return lower_limits, upper_limits, joint_ranges

    def get_flange_state(self):
        return self._pybullet_client.getLinkState(self._robotic_arm, 6)

    
    def reset_pose(self):
        q_start = self._constants.RESET_POS

        q_index = 0

        for _, joint_id in self._joint_name_to_id.items():

            joint_info = self.pybullet_client.getJointInfo(self._robotic_arm, joint_id)
            joint_type = joint_info[2]

            if joint_type == self.pybullet_client.JOINT_FIXED:
                continue

            if q_index >= len(q_start):
                break

            self.pybullet_client.resetJointState(
                self._robotic_arm,
                joint_id,
                q_start[q_index]
            )

            q_index += 1

    def terminate(self):
        pass
