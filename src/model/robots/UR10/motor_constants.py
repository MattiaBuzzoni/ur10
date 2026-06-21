import numpy as np

from src.model.robots import motor

NUM_MOTORS = 6

MOTOR_ENABLED = [True] * NUM_MOTORS

MOTOR_OFFSET = np.array([0.] * NUM_MOTORS)

MOTOR_DIRECTION = np.array([1] * NUM_MOTORS)

MOTOR_POSITION_GAINS = [2200, 2200, 500, 81, 81, 81]

MOTOR_VELOCITY_GAINS = [24.7, 24.7, 11.2, 4.0, 4.0, .0]

MOTOR_CONTROL_CLASS = motor.RobotMotorModel

MOTOR_MODEL = motor
