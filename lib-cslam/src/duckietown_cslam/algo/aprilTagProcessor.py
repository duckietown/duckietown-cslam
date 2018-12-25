#!/usr/bin/env python

__all__ = [
    'aprilTagProcessor',
]

from argparse import ArgumentParser
import numpy as np

import sys
import os

ACQ_APRILTAG_LIB = os.getenv('ACQ_APRILTAG_LIB', '/media/sf_Duckietown/SLAM/WIP/apriltag/python')
ACQ_APRILTAG_SO = os.getenv('ACQ_APRILTAG_SO', '/media/sf_Duckietown/SLAM/WIP/apriltag/build/lib')

sys.path.append(ACQ_APRILTAG_LIB)
aprilTagSooPath = [ACQ_APRILTAG_SO, ACQ_APRILTAG_SO]
import apriltag

class aprilTagProcessor():
    """
    Interface to the necessary AprilTag functionalities.
    """
    def __init__(self, tag_size):

        # The python bindings for the April Tag library require this strange configuration:
        parser = ArgumentParser(description='test apriltag Python bindings')
        parser.families='tag36h11'
        parser.border=1 # how wide (in bit-sizes) is the black border? (usually 1)
        parser.nthreads=4 #(int) number of threads to use
        parser.quad_decimate=1.0 #(float) Decimate input image by this factor
        parser.quad_blur=0.0 #(float) Apply low-pass blur to input; negative sharpens
        parser.refine_edges=True #(bool) Set to True to spend more time to align edges of tags
        parser.refine_decode=True #(bool) Set to True to spend more time to decode tags
        parser.refine_pose=True #(bool) Set to True to spend more time to precisely localize tags
        parser.debug=False #(bool)Enable/Disable debugging output (slow when enabled)
        parser.quad_contours=True
        apriltag.add_arguments(parser)

        self.options = parser.parse_args()
        self.options.tag_size = tag_size

    def configureDetector(self, cameraMatrix):
        """
        Configures the AprilTag detector

        Returns from detector:
        tag_family (string) : Family that the tag belongs to
        tag_id (int) : The decoded ID of the tag
        hamming (int) : How many error bits were corrected? Note: accepting large numbers of corrected errors leads to greatly increased false positive rates.
        goodness (float) : A measure of the quality of tag localization: measures the average contrast of the pixels around the border of the tag. refine_pose must be enabled, or this field will be zero.
        decision_margin(float) : A measure of the quality of the binary decoding process: the average difference between the intensity of a data bit versus the decision threshold. Higher numbers roughly indicate better decodes. This is a reasonable measure of detection accuracy only for very small tags-- not effective for larger tags (where we could have sampled anywhere within a bit cell and still gotten a good detection.)
        homography (3x3 array) : The 3x3 homography matrix describing the projection from an "ideal" tag (with corners at (-1,-1), (1,-1), (1,1), and (-1, 1)) to pixels in the image.
        center (1x2 array) : The center of the detection in image pixel coordinates.
        corners (4x2 array) : The corners of the tag in image pixel coordinates. These always wrap counter-clock wise around the tag. (top left is origin, x axis is horizontal, y axis is vertical)
        """

        camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

        self.options.camera_params = camera_params
        self.detector = apriltag.Detector(self.options, searchpath=aprilTagSooPath)

        return self.detector

    def resultsToPose(self, results):
        """
        Converts the outputs of the AprilTag libary which are in the form of a rotation matrix to quaternions.
        """
        for i, detection in enumerate(results):
            pose, e0, e1 = self.detector.detection_pose(detection,
                                                   self.options.camera_params,
                                                   self.options.tag_size)

            qvec = self.mat2quat(pose[:3, :3])
            tvec = pose[:3, 3]
            results[i].qvec = qvec
            results[i].tvec = tvec

        return results

    def mat2quat(self, M):
        """
        Helper function that converts rotation matrices to quaternions.
        """
        Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = M.flat
        K = np.array([[Qxx - Qyy - Qzz, 0, 0, 0],
                      [Qyx + Qxy, Qyy - Qxx - Qzz, 0, 0],
                      [Qzx + Qxz, Qzy + Qyz, Qzz - Qxx - Qyy, 0],
                      [Qyz - Qzy, Qzx - Qxz, Qxy - Qyx, Qxx + Qyy + Qzz]]) / 3.0
        # Use Hermitian eigenvectors, values for speed
        vals, vecs = np.linalg.eigh(K)
        # Select largest eigenvector, reorder to x,y,z,w quaternion
        q = vecs[[0, 1, 2, 3], np.argmax(vals)]
        # Prefer quaternion with positive w
        # (q * -1 corresponds to same rotation as q)
        if q[3] < 0:
            q *= -1
        return q
