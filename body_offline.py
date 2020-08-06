'''
    body.py offline version.

    Body measurements, relations of the links with each other, Kalman parameters

    p: positions of joints -> chest, shoulder, elbow, wrist, (hand)
    s: link lengths, between two joints -> chest, lua, lla, hand
    q_SB: sensor to body orientation, constant.
    q_SG: sensor to global orientation, depend on the previous joint. not updated by Kalman. Input
    q_BG: body to global orientation, the target.

'''

# import numpy as np
# from math import acos
# from math import sqrt
# import Kinematics_with_Quaternions as kinematic
# import kalman as kf
# from IMU_subscriber_class_v2 import IMUsubscriber
# from geometry_msgs.msg import Quaternion
# from my_human_pkg.msg import test_msg

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from lib.util import*


def test_data(file_name):
    cur_dir = os.getcwd()
    fs = 512
    cutoff = 10
    cutoff_fs = cutoff / fs
    print "cutoff_fs:", cutoff_fs
    file_path=os.path.join(cur_dir, 'data', file_name)
    data=np.array(read_data_xlsx(file_path))
    print data[0].shape
    median_data=median_filter(data[0], 155)
    comb_data=freq_filter(median_data, 155, 0.01953125)
    plot_subplot(data[0], 'raw data')
    plot_subplot(comb_data, 'filtered data')
    plt.show()


if __name__ == '__main__':
    if len(sys.argv)<2:
        raise ValueError('No file name specified')
    test_data(sys.argv[1])
