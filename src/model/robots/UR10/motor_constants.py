import numpy as np

from src.model.robots import motor

NUM_MOTORS = 12

MOTOR_ENABLED = [True] * NUM_MOTORS

MOTOR_OFFSET = np.array([0.] * NUM_MOTORS)

MOTOR_DIRECTION = np.array([1] * NUM_MOTORS)