import numpy as np
from geometry_msgs.msg import Quaternion
from lib.kalman import*
# b = Quaternion(0, 0, 0, 1)
# a = np.array(3*[Quaternion(0, 0, 0, 1)])
#
# print a[0]
# print a[0][0]

acc1_data = np.array(10*[[3, 0, 0]], dtype=np.float32)
acc2_data = np.array(10*[[3, 0, 0]], dtype=np.float32)
acc = [acc1_data, acc2_data]

print "acc:", acc[0].shape
quat1_data = np.array(10*[[0, 0, 0, 1]], dtype=np.float32)
quat2_data = np.array(10*[[0, 0, 0, 1]], dtype=np.float32)
b_u = calculate_b_u([acc1_data, acc2_data], [quat1_data, quat2_data])
print b_u.shape
