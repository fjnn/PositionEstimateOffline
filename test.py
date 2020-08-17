import numpy as np
from geometry_msgs.msg import Quaternion
from lib.kalman import*
import sys

# b = Quaternion(0, 0, 0, 1)
# a = np.array(3*[Quaternion(0, 0, 0, 1)])
#
# print a[0]
# print a[0][0]

acc1_data = np.array(5*[[4, 0, 0], [2, 0, 0]], dtype=np.float32)
acc2_data = np.array(10*[[0, 2, 0]], dtype=np.float32)
acc = [acc1_data, acc2_data]

sys.exit("I am done")
