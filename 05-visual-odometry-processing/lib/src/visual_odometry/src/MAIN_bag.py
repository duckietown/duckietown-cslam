"""
[2019] Modified by Aleksandar Petrov <alpetrov@ethz.ch> for the purposes of Duckietown cSLAM.

MIT License (MIT)

Copyright (c) SUMMER 2016, Carnegie Mellon University

Author: Jahdiel Alvarez

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Original Code:
https://github.com/uoip/monoVO-python


"""

from time import sleep

from Common_Modules import *
from py_MVO import VisualOdometry
import Trajectory_Tools as TT


import rosbag
from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import numpy as np
import matplotlib.pyplot as plt


BAG_NAME = "onlybot.bag"
K = np.array([218.54617309570312, 0.0, 338.50325139415145, 0.0, 218.64820861816406, 213.44866555069802, 0.0, 0.0, 1.0]).reshape((3,3))
f_detector = "SURF" #SIFT, FAST, SURF, SHI-TOMASI
window_flag = 'WINDOW_YES'

def run():

    bridge = CvBridge()

    print('-- Press ESC key to end program\n')
    #Parse the Command Line/Terminal

    bag = rosbag.Bag(BAG_NAME, 'r')

    # Initializing the Visual Odometry object
    vo = VisualOdometry(K, f_detector, pitch_adjust=np.deg2rad(10.0))

    # Square for the real-time trajectory window
    traj = np.zeros((1200, 1200, 3), dtype=np.uint8)

    img_id=-1

    xs = [0.0]
    ys = [0.0]
    last_t = np.zeros(3)
    last_R = np.eye(3)

    plt.ion()
    for topic, msg, t in bag.read_messages(topics='/poses_acquisition/rectified_video/autobot04/compressed'):

        img_id+=1

        k = cv2.waitKey(10) & 0xFF
        if k == 27:  # Wait for ESC key to exit
            cv2.destroyAllWindows()
            return

        imgKLT = bridge.compressed_imgmsg_to_cv2(msg)
        imgKLT = cv2.cvtColor(imgKLT, cv2.COLOR_GRAY2RGB)
        img = bridge.compressed_imgmsg_to_cv2(msg)

        # Create a CLAHE object (contrast limiting adaptive histogram equalization)
        clahe = cv2.createCLAHE(clipLimit=5.0)
        img = clahe.apply(img)

        if vo.update(img, img_id):  # Updating the vectors in VisualOdometry class
            cur_t = vo.cur_t  # Retrieve the translation vectors
            # ------- Windowed Displays ---------
            if window_flag == 'WINDOW_YES':
                if img_id > 0:  # Set the points for the real-time trajectory window
                    x, y, z = cur_t[0], cur_t[1], cur_t[2]
                    TT.drawOpticalFlowField(imgKLT, vo.OFF_prev, vo.OFF_cur)  # Draw the features that were matched
                else:
                    x, y, z = 0., 0., 0.

                xs.append(xs[-1]+vo.relative_pose_x)
                ys.append(ys[-1]+vo.relative_pose_y)

                s = np.sqrt(vo.relative_pose_y**2 + vo.relative_pose_x**2)

                plt.plot([0, vo.relative_pose_y/s], [0, vo.relative_pose_x/s])

                empirical_angle = np.arctan2(ys[-1]-ys[-2], xs[-1]-xs[-2])

                plt.plot([0, np.cos(vo.relative_pose_theta+0.5*3.1415)], [0, np.sin(vo.relative_pose_theta+0.5*3.1415)], '--')

                plt.xlim(-1.5,1.5)
                plt.ylim(-1.5,1.5)
                plt.grid()

                plt.draw()
                plt.pause(0.1)
                plt.clf()

                traj = TT.RT_trajectory_window(traj, x, y, z, img_id)  # Draw the trajectory window
            # -------------------------------------
        img_id += 1  # Increasing the image id



    return



#################################################################################

if __name__ == '__main__':

    run()
