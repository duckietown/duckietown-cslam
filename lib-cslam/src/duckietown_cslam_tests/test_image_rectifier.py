# coding=utf-8
from comptests import comptest, run_module_tests

from duckietown_cslam import ImageRectifier
import cv2
import numpy as np

import os
import logging

@comptest
def compare_rectified_image():

    logging.basicConfig()
    logger = logging.getLogger('dt-cslam [compare_rectified_image test]')
    logger.setLevel(logging.DEBUG)

    logger.info(os.getcwd())


    im = cv2.imread("src/duckietown_cslam_tests/test_image_before.png")
    print("Before image size", im.shape)
    D = np.array([-0.2967039649743125, 0.06795775093662262, 0.0008927768064001824, -0.001327854648098482, 0.0])
    K = np.array([336.7755634193813, 0.0, 333.3575643300718, 0.0, 336.02729840829176, 212.77376312080065, 0.0, 0.0, 1.0]).reshape((3,3))

    groundTruth_newCameraMatrix = np.array([[227.72740173, 0., 330.57516721], [0., 227.81047058, 213.42367165], [0, 0., 1.]])
    groundTruth_image = cv2.imread("src/duckietown_cslam_tests/test_image_after.png")

    imRect = ImageRectifier(im, K, D)
    rectIm, newCameraMatrix = imRect.rectify(im)

    if np.linalg.norm(np.abs(groundTruth_image-rectIm), axis=2, ord=np.inf).any() > 5.0:
        raise Exception("The rectified image is not the same as the ground truth image")
        logger.error("The rectified image is not the same as the ground truth image")

    if np.linalg.norm(np.abs((groundTruth_newCameraMatrix-newCameraMatrix).flatten()), np.inf) > 0.0001:
        raise Exception("The rectified image new camera matrix is not the same as the ground truth matriximage")
        logger.error("The rectified image new camera matrix is not the same as the ground truth matriximage")


if __name__ == '__main__':
    run_module_tests()
