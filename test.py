import numpy as np
from geometry_msgs.msg import Quaternion
# b = Quaternion(0, 0, 0, 1)
# a = np.array(3*[Quaternion(0, 0, 0, 1)])
#
# print a[0]

a = np.empty([5,3,2])
a[0][0] = np.zeros(2)
# print a[0][0]

acc_data = np.array(5*[np.empty()])
print acc_data.shape
