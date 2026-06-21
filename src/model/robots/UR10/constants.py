import re
from math import pi
import numpy as np


START_POS = [0., 0., 0.]
INIT_ORIENTATION = [0., 0., 0.]

RESET_POS = [pi/2, 0, 0, pi/2, -pi/2, 0 ]


d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922]   # UR10 mm
a = [0 ,-0.612 ,-0.5723 ,0 ,0 ,0]   # UR10 mm

alpha = [pi/2, 0, 0, pi/2, -pi/2, 0 ]   # UR10
