import numpy as np

from src.model.robots import motor

NUM_MOTORS = 12

MOTOR_ENABLED = [True] * NUM_MOTORS

MOTOR_OFFSET = np.array([0.] * NUM_MOTORS)

MOTOR_DIRECTION = np.array([1] * NUM_MOTORS)

MOTOR_POSITION_GAINS = [220.] * NUM_MOTORS

MOTOR_VELOCITY_GAINS = 30

MOTOR_CONTROL_CLASS = motor.RobotMotorModel

MOTOR_MODEL = motor
