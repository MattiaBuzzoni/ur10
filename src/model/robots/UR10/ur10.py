from src.model.robots.UR10 import constants, marks
from src.model.robots.robot import Robot


class UR10(Robot):

    @classmethod
    def get_constants(cls):
        del cls
        return constants

    @classmethod
    def get_marks(cls):
        del cls
        return marks
