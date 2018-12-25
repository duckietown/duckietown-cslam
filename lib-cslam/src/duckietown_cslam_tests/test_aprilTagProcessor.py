# coding=utf-8
from comptests import comptest, run_module_tests

from duckietown_cslam import aprilTagProcessor
from duckietown_cslam import ImageRectifier
import cv2

import numpy as np

import os
import logging

import numpy as np

import sys

@comptest
def aprilTag_extraction_test():

    # Load the AprilTag library
    # ACQ_APRILTAG_LIB = os.getenv('ACQ_APRILTAG_LIB', '/media/sf_Duckietown/SLAM/WIP/apriltag/python')
    # ACQ_APRILTAG_SO = os.getenv('ACQ_APRILTAG_SO', '/media/sf_Duckietown/SLAM/WIP/apriltag/build/lib')
    #
    # sys.path.append(ACQ_APRILTAG_LIB)
    # aprilTagSooPath = [ACQ_APRILTAG_SO, ACQ_APRILTAG_SO]
    # import apriltag

    # Load and rectify the test image
    im = cv2.imread("src/duckietown_cslam_tests/test_image_before.png")
    D = np.array([-0.2967039649743125, 0.06795775093662262, 0.0008927768064001824, -0.001327854648098482, 0.0])
    K = np.array([336.7755634193813, 0.0, 333.3575643300718, 0.0, 336.02729840829176, 212.77376312080065, 0.0, 0.0, 1.0]).reshape((3,3))
    imRect = ImageRectifier(im, K, D)
    rectIm, newCameraMatrix = imRect.rectify(im)
    test_image_gray = cv2.cvtColor(rectIm, cv2.COLOR_BGR2GRAY)

    # Setup and execute the AprilTag extraction
    atp = aprilTagProcessor(tag_size=0.065)
    aprilTags = atp.configureDetector(newCameraMatrix).detect(test_image_gray, return_image=False)
    aprilTags = atp.resultsToPose(aprilTags)
    #

    found_tags = list()
    for tag in aprilTags:
        found_tags.append(tag.tag_id)

    for expected_tag in [60, 82, 318, 328, 387]:
        assert(expected_tag in found_tags)

if __name__ == '__main__':
    run_module_tests()
