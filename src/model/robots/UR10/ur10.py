from src.model.robots.UR10 import constants, marks, motor_constants, ctrl_constants
from src.model.robots.robot import Robot


class UR10(Robot):

    @classmethod
    def get_motor_class(cls):
        del cls
        return motor_constants.MOTOR_CONTROL_CLASS

    @classmethod
    def get_motor_constants(cls):
        del cls
        return motor_constants
    
    @classmethod
    def get_ctrl_constants(cls):
        del cls
        return ctrl_constants

    @classmethod
    def get_constants(cls):
        del cls
        return constants

    @classmethod
    def get_marks(cls):
        del cls
        return marks
