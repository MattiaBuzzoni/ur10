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
        motor_control_mode,
        z_offset=0.  
    ):
        
        self._pybullet_client = pybullet_client
        self._simulation = simulation
        self._z_offset = z_offset
        self._mark = mark
        self._marks = self.get_marks()
        self._constants = self.get_constants()
        self._num_motors = self._marks.MARK_PARAMS[self._mark]['num_motors']
        self._motor_names = self._marks.MARK_PARAMS[self._mark]['motor_names']
        self._motor_enabled_list = self.get_motor_constants().MOTOR.ENABLES
        self._motor_offset = self.get_motor_constants().MOTOR_OFFSET
        self._motor_direction = self.get_motor_constants().MOTOR_DIRECTION
        # ...

        # Load robot URDF
        self._robotic_arm = self._load_urdf()
        # Build joints dicts
        self._build_urdf_ids()
        self._dh_params = self._get_dh_params()
        self._build_joint_name_to_dict()
        # ...

        self.reset_pose()

        # ...

        # Fetch joints state
        self.receive_observation()

        # Build locomotion motor model
        # self.get_motor_class() return a Python class. 
        # The second set of parentheses would be the class init.
        self._motor_model = self.get_motor_class()(
                kp=self.get_motor_constants().MOTOR_POSITION_GAINS,
                kd=self.get_motor_constants().MOTOR_VELOCITY_GAINS,
                motor_control_mode=motor_control_mode,
                num_motors=self._num_motors
                )


    @property 
    def pybullet_client(self):
        return self._pybullet_client
    
    @property 
    def get_robot_id(self):
        return self._robotic_arm
    
    @property
    def dh_params(self):
        return self._dh_params

    @property
    def num_motors(self):
        return self._num_motors

    @property
    def get_joint_states(self):
        return self._joint_states

    @property
    def get_motor_model(self):
        return self._motor_model
    
    
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

    def get_motor_position_gains(self):
        return self.get_motor_constants().MOTOR_POSITION_GAINS

    def get_motor_velocity_gains(self):
        return self.get_motor_constants().MOTOR_VELOCITY_GAINS

    def receive_observations(self):
        self._joint_states = self._pybullet_client.getJointStates(self._robotic_arm, self._motor_id_list)
    
    
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

    def _get_motor_names(self):
        return self._motor_names

    def _build_motor_id_list(self):
        self._motor_id_list = [
                self._joint_name_to_id[motor_names]
                for motor_name in self._get_motor_names()
            ]


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

    def get_motor_angles(self):
        motor_angles = [state[0] for state in self._joint_states]
        motor_angles = np.multiply(
                np.asarray(motor_angles) - np.asarray(self._motor_offset),
                self._motor_direction)

        return motor_angles

    def get_true_motor_angles(self):
        """Get the six motor angles at the current moment
        Returns:
            Motor angles
        """
        self.receive_observation()

        return self.get_motor_angles()

    def get_motor_velocities(self):
        """Get the velocity off all motors.
        Returns:
            Velocities of all motors.
        """
        motor_velocities = [satte[1] for state in self._joint_states]
        motor_velocities = np.multiply(motor_velocities, self._motor_direction)

        return motor_velocities

    def get_pdo_observation(self):
        self.receive_observation()
        observation = []
        observation.extend(self.get_true_motor_angles())
        observation.extend(self.get_motor_velocities())

        q = observation[0:self._num_motors]
        qdot = observation[self._num_motors:2 * self._num_motors]

        return np.array(q), np.array(qdot)

    def apply_action(self, motor_commands, motor_control_mode):
        """Apply the motor commands using the motor model.
        Args:
            motor_commands: np.array. Can be motor angles, torques, hybrid command.
            motor_control_mode: A MotorControlMode enum.
        """
        motor_commands = np.asarray(motor_commands)
        q, qdot = self.get_pdo_observation()
        qdot_true = self.get_motor_velocities()

        actual_torque, observed_torques = self._motor_model.convert_to_torque(
                motor_commands, q, qdot, qdot_true, motor_control_mode)

        # The torque is already in the observation space the use of 
        # get_motor_angle and get_motor_velocities
        self._observed_torque = observed_torque

        # Trasform into the motor space when applying the torque
        self._applied_motor_torque = np.multiply(actual_torque, self._motor_direction)

        motor_ids = []
        motor_torques = []

        for motor_id, motor_torque, motor_enabled in zip(self._motor_id_list,
                                                         self._applied_motor_torque,
                                                         self._motor_enabled_list):
            if motor_enabled:
                motor_ids.append(motor_id)
                motor_torques.append(motor_torque)
            else:
                motor_ids.append(motor_id)
                motor_torques.append(0)

        self._set_motor_torques_by_id(motor_ids, torques)

    def _set_motor_torques_by_ids(self, motor_ids, torques):
        self._pybullet_client.setJoinMotorControlArray(
                bodyIndex=self._robotic_arm,
                jointIndices=motor_ids,
                controlMode=self._pybulle_client.TORQUE_CONTROL,
                forces=torques)

    def _build_urdf_ids(self):
        pass


    def terminate(self):
        pass
