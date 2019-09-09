#!/usr/bin/env python

__all__ = [
    'ImageRectifier',
]

import cv2
import numpy as np

class ImageRectifier():
    def __init__(self, image, cameraMatrix, distCoeffs):
        """
        Rectifies an image with a particular cameraMatrix(K) and distCoeffs(D); based on opencv undistort function
        see: https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html?highlight=initundistort#undistort
        see: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
        """
        # Validate the inputs
        if len(np.shape(cameraMatrix)) !=2 or np.shape(cameraMatrix)[0]!=3 or np.shape(cameraMatrix)[1]!=3:
            raise ValueError("Camera matrix is not a 3x3 matix! It is of shape %s." % str(np.shape(cameraMatrix)))
        if len(np.shape(distCoeffs)) !=1 or np.shape(distCoeffs)[0]!=5:
            raise ValueError("distCoeffs should be a vector of length 5! It is of shape %s." % str(np.shape(cameraMatrix)))

        # Calculate the new camera matrix
        self.newCameraMatrix, self.validPixROI = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (image.shape[1], image.shape[0]), 1.0)

        # Calculate the undistorting maps
        self.map1, self.map2 = cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs, np.eye(3), self.newCameraMatrix, (image.shape[1], image.shape[0]), cv2.CV_32FC1)

        self.rectify(image)

    def rectify(self, image):

        # Undistort the image
        remappedIm = cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)

        return remappedIm, self.newCameraMatrix

    def beautify(self, image):
        """
        Applies CLAHE (Contrast Limited Adaptive Histogram Equalization) to a RGB or grayscale image in order to improve the contrast
        """

        # Process separately for colour and grayscale image:
        if len(image.shape) == 2:
            # CLAHE (Contrast Limited Adaptive Histogram Equalization)
            clahe = cv2.createCLAHE(clipLimit=3., tileGridSize=(8,8))
            img2 = clahe.apply(image)  # apply CLAHE to the L-channel
        elif image.shape[2] == 3:
            clahe = cv2.createCLAHE(clipLimit=3., tileGridSize=(8,8))

            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)  # convert from BGR to LAB color space
            l, a, b = cv2.split(lab)  # split on 3 different channels

            l2 = clahe.apply(l)  # apply CLAHE to the L-channel

            lab = cv2.merge((l2,a,b))  # merge channels
            img2 = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)  # convert from LAB to BGR
        else:
            raise Exception("Unsupported image dimensions: ", image.shape)

        return img2

if __name__ == "__main__":
    # Example usage
    import cv2
    im = cv2.imread("test_image_rectifier_img_before.png")
    D = np.array([-0.2967039649743125, 0.06795775093662262, 0.0008927768064001824, -0.001327854648098482, 0.0])
    K = np.array([336.7755634193813, 0.0, 333.3575643300718, 0.0, 336.02729840829176, 212.77376312080065, 0.0, 0.0, 1.0]).reshape((3,3))

    imRect = ImageRectifier(im, K, D)
    rectIm, newCameraMatrix = imRect.rectify(im)
    print(newCameraMatrix)
    status = cv2.imwrite('test_image_rectifier_img_after.png',rectIm)

    # write the beautified images
    status = cv2.imwrite('test_image_rectifier_img_after_beautified_color.png',imRect.beautify(rectIm))
    status = cv2.imwrite('test_image_rectifier_img_after_beautified_gray.png',imRect.beautify(cv2.cvtColor(rectIm, cv2.COLOR_BGR2GRAY)))
