import numpy as np
from geometry_msgs.msg import Quaternion
b = Quaternion(0, 0, 0, 1)
a = np.array(3*[Quaternion(0, 0, 0, 1)])

print a[0]
