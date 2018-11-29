#!/usr/bin/env python

from image_rectifier import ImageRectifier
from argparse import ArgumentParser
import sys
import os
import numpy as np
import cv2

ACQ_APRILTAG_LIB = os.getenv('ACQ_APRILTAG_LIB', '/media/sf_Duckietown/SLAM/WIP/apriltag/python')
ACQ_APRILTAG_SO = os.getenv('ACQ_APRILTAG_SO', '/media/sf_Duckietown/SLAM/WIP/apriltag/build/lib')

sys.path.append(ACQ_APRILTAG_LIB)
aprilTagSooPath = [ACQ_APRILTAG_SO, ACQ_APRILTAG_SO]
import apriltag

class aprilTagProcessor():
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

        ### configures the dector
        ### returns from detector
        ##tag_family (string)
        #Family that the tag belongs to
        ##tag_id (int)
        #The decoded ID of the tag
        ##hamming (int)
        #How many error bits were corrected? Note: accepting large numbers of corrected errors leads to greatly increased false positive rates.
        ##goodness (float)
        #A measure of the quality of tag localization: measures the average contrast of the pixels around the border of the tag. refine_pose must be enabled, or this field will be zero.
        ##decision_margin(float)
        #A measure of the quality of the binary decoding process: the average difference between the intensity of a data bit versus the decision threshold. Higher numbers roughly indicate better decodes. This is a reasonable measure of detection accuracy only for very small tags-- not effective for larger tags (where we could have sampled anywhere within a bit cell and still gotten a good detection.)
        ##homography (3x3 array)
        #The 3x3 homography matrix describing the projection from an "ideal" tag (with corners at (-1,-1), (1,-1), (1,1), and (-1, 1)) to pixels in the image.
        ##center (1x2 array)
        #The center of the detection in image pixel coordinates.
        ##corners (4x2 array)
        #The corners of the tag in image pixel coordinates. These always wrap counter-clock wise around the tag. (top left is origin, x axis is horizontal, y axis is vertical)

        camera_params = ( cameraMatrix[0,0], cameraMatrix[0,2], cameraMatrix[1,1], cameraMatrix[1,2] )

        self.options.camera_params = camera_params
        self.detector = apriltag.Detector(self.options, searchpath=aprilTagSooPath)
        return self.detector

    def resultsToPose(self, results):
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



class deviceSideProcessor():
    def __init__(self, options):
        self.ImageRectifier = ImageRectifier()
        self.opt_beautify = options.get("beautify", False)
        self.aprilTagProcessor = aprilTagProcessor(options.get("tag_size", 0.065))

    def process(self, raw_image, cameraMatrix, distCoeffs):

        # 0. Beautify if wanted:
        if self.opt_beautify:
            raw_image = self.ImageRectifier.beautify(raw_image)

        # 1. Rectify the raw image and get the new camera cameraMatrix
        rect_image, newCameraMatrix = self.ImageRectifier.rectify(raw_image, cameraMatrix, distCoeffs)

        # 2. Extract april tags data
        tect_image_gray = cv2.cvtColor(rect_image, cv2.COLOR_BGR2GRAY)
        aprilTags = self.aprilTagProcessor.configureDetector(newCameraMatrix).detect(tect_image_gray, return_image=False)

        # 3. Extract poses from april tags data
        aprilTags = self.aprilTagProcessor.resultsToPose(aprilTags)

        # 4. Package output
        outputDict = []
        for atag in aprilTags:
            outputDict.append({ 'rect_image': rect_image,
                                'new_camera_matrix': newCameraMatrix,
                                'tag_id': atag.tag_id,
                                'goodness': atag.goodness,
                                'corners': atag.corners,
                                'qvec': atag.qvec,
                                'tvec': atag.tvec })
        return outputDict


if __name__ == "__main__":
    import cv2
    im = cv2.imread("wt10_sample.png")
    D = np.array([-0.2967039649743125, 0.06795775093662262, 0.0008927768064001824, -0.001327854648098482, 0.0])
    K = np.array([336.7755634193813, 0.0, 333.3575643300718, 0.0, 336.02729840829176, 212.77376312080065, 0.0, 0.0, 1.0]).reshape((3,3))

    dsp_options={"beautify": False, "tag_size": 0.065}
    dsp = deviceSideProcessor(dsp_options)
    output = dsp.process(im,  K, D)

    import matplotlib.pyplot as plt
    plt.imshow(output[0]['rect_image'])
    for tag in output:
        plt.plot(tag['corners'][:, 0], tag['corners'][:, 1] )
    plt.show()
