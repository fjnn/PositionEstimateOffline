import numpy as np
from geometry_msgs.msg import Quaternion
from lib.kalman import*
# b = Quaternion(0, 0, 0, 1)
# a = np.array(3*[Quaternion(0, 0, 0, 1)])
#
# print a[0]
# print a[0][0]

acc1_data = np.array(5*[[4, 0, 0], [2, 0, 0]], dtype=np.float32)
acc2_data = np.array(10*[[0, 2, 0]], dtype=np.float32)
acc = [acc1_data, acc2_data]

# print "acc:", acc[0].shape
# quat1_data = np.array(10*[[0, 0, 0, 1]], dtype=np.float32)
# quat2_data = np.array(10*[[0, 0, 0, 1]], dtype=np.float32)
# b_u = calculate_b_u([acc1_data, acc2_data], [quat1_data, quat2_data])
# plot_subplot(b_u[:,3:], "position", ylim=[-0.1, 0.15])
# plt.show()
# print b_u.shape


#  MEDIAN DATA
# def get_avg(arr):

test_arr = [np.arange(0,3).reshape(1,3),np.arange(4,7).reshape(1,3)]
# print test_arr
a = np.array([[0,1,2],[4,5,6]])
b = np.array(acc1_data)
averaged_array = np.average(acc1_data,axis=0)
print averaged_array
