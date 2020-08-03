from __future__ import division
from __future__ import print_function

import os
import sys
import numpy

sys.path.append(os.getenv('ACQ_ARUCO_CALLER_DIR'))
from aruco_caller import aruco_init, aruco_detect_and_estimate

class Detection():

    '''Combined pythonic wrapper for apriltag_detection and apriltag_pose'''

    def __init__(self):
        self.tag_family = None
        self.tag_id = None
        self.hamming = None
        self.decision_margin = None
        self.homography = None
        self.center = None
        self.corners = None
        self.pose_R = None
        self.pose_t = None
        self.pose_err = None

    def __str__(self):
        return('Detection object:'+
        '\ntag_family = ' + str(self.tag_family)+
        '\ntag_id = ' + str(self.tag_id)+
        '\nhamming = ' + str(self.hamming)+
        '\ndecision_margin = ' + str(self.decision_margin)+
        '\nhomography = ' + str(self.homography)+
        '\ncenter = ' + str(self.center)+
        '\ncorners = ' + str(self.corners)+
        '\npose_R = ' + str(self.pose_R)+
        '\npose_t = ' + str(self.pose_t)+
        '\npose_err = ' + str(self.pose_err)+'\n')

    def __repr__(self):
        return self.__str__()


######################################################################

class Detector(object):

    def __init__(self, marker_size=0.065, config_file="config.yml"):
        aruco_init(marker_size, config_file)

    def detect(self, image, width, height, cameraMatrix, distCoeffs):

        '''Run detectons on the provided image. The image must be a grayscale
image of type numpy.uint8.'''

        assert len(image.shape) == 2
        assert image.dtype == numpy.uint8

        calib_data = {
            "width": width,
            "height": height,
            "cameraMatrix00": cameraMatrix[0][0],
            "cameraMatrix02": cameraMatrix[0][2],
            "cameraMatrix11": cameraMatrix[1][1],
            "cameraMatrix12": cameraMatrix[1][2],
            "distortion0": distCoeffs[0],
            "distortion1": distCoeffs[1],
            "distortion2": distCoeffs[2],
            "distortion3": distCoeffs[3]
        }

        #detect apriltags in the image
        detections = aruco_detect_and_estimate(calib_data, image)

        return_info = []

        for marker in detections:
            detection = Detection()
            detection.tag_family = marker["tag_family"]
            detection.tag_id = marker["tag_id"]
            detection.hamming = -1
            detection.decision_margin = -1
            detection.homography = numpy.zeros((3, 3))
            detection.center = numpy.zeros((2, 1))
            detection.corners = numpy.array(marker["corners"])
            if len(marker["rvec"]) == 3:
                detection.pose_R = numpy.array(marker["rvec"])
                detection.pose_t = numpy.array(marker["tvec"])
            else:
                detection.pose_R = numpy.zeros(3)
                detection.pose_t = numpy.zeros(3)
            detection.pose_err = -1

            return_info.append(detection)

        return return_info

# inconsistent now
if __name__ == '__main__':

    test_images_path = 'test'

    visualization = True
    try:
        import cv2
    except:
        raise Exception('You need cv2 in order to run the demo. However, you can still use the library without it.')

    try:
        from cv2 import imshow
    except:
        print("The function imshow was not implemented in this installation. Rebuild OpenCV from source to use it")
        print("VIsualization will be disabled.")
        visualization = False

    try:
        import yaml
    except:
        raise Exception('You need yaml in order to run the tests. However, you can still use the library without it.')

    at_detector = Detector(searchpath=['apriltags/lib', 'apriltags/lib64'],
                           families='tag36h11',
                           nthreads=1,
                           quad_decimate=1.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)

    with open(test_images_path + '/test_info.yaml', 'r') as stream:
        parameters = yaml.load(stream)

    #### TEST WITH THE SAMPLE IMAGE ####

    print("\n\nTESTING WITH A SAMPLE IMAGE")

    img = cv2.imread(test_images_path+'/'+parameters['sample_test']['file'], cv2.IMREAD_GRAYSCALE)
    cameraMatrix = numpy.array(parameters['sample_test']['K']).reshape((3,3))
    camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

    if visualization:
        cv2.imshow('Original image',img)

    tags = at_detector.detect(img, True, camera_params, parameters['sample_test']['tag_size'])
    print(tags)

    color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    for tag in tags:
        for idx in range(len(tag.corners)):
            cv2.line(color_img, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

        cv2.putText(color_img, str(tag.tag_id),
                    org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.8,
                    color=(0, 0, 255))

    if visualization:
        cv2.imshow('Detected tags', color_img)

        k = cv2.waitKey(0)
        if k == 27:         # wait for ESC key to exit
            cv2.destroyAllWindows()


    #### TEST WITH THE ROTATION IMAGES ####

    import time

    print("\n\nTESTING WITH ROTATION IMAGES")

    time_num = 0
    time_sum = 0

    test_images_path = 'test'
    image_names = parameters['rotation_test']['files']

    for image_name in image_names:
        print("Testing image ", image_name)
        ab_path = test_images_path + '/' + image_name
        if(not os.path.isfile(ab_path)):
            continue
        groundtruth = float(image_name.split('_')[-1].split('.')[0])  # name of test image should be set to its groundtruth

        parameters['rotation_test']['rotz'] = groundtruth
        cameraMatrix = numpy.array(parameters['rotation_test']['K']).reshape((3,3))
        camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

        img = cv2.imread(ab_path, cv2.IMREAD_GRAYSCALE)

        start = time.time()
        tags = at_detector.detect(img, True, camera_params, parameters['rotation_test']['tag_size'])

        time_sum+=time.time()-start
        time_num+=1

        print(tags[0].pose_t, parameters['rotation_test']['posx'], parameters['rotation_test']['posy'], parameters['rotation_test']['posz'])
        print(tags[0].pose_R, parameters['rotation_test']['rotx'], parameters['rotation_test']['roty'], parameters['rotation_test']['rotz'])

    print("AVG time per detection: ", time_sum/time_num)

    #### TEST WITH MULTIPLE TAGS IMAGES ####

    print("\n\nTESTING WITH MULTIPLE TAGS IMAGES")

    time_num = 0
    time_sum = 0

    image_names = parameters['multiple_tags_test']['files']

    for image_name in image_names:
        print("Testing image ", image_name)
        ab_path = test_images_path + '/' + image_name
        if(not os.path.isfile(ab_path)):
            continue

        cameraMatrix = numpy.array(parameters['multiple_tags_test']['K']).reshape((3,3))
        camera_params = ( cameraMatrix[0,0], cameraMatrix[1,1], cameraMatrix[0,2], cameraMatrix[1,2] )

        img = cv2.imread(ab_path, cv2.IMREAD_GRAYSCALE)

        start = time.time()
        tags = at_detector.detect(img, True, camera_params, parameters['multiple_tags_test']['tag_size'])
        time_sum+=time.time()-start
        time_num+=1

        tag_ids = [tag.tag_id for tag in tags]
        print(len(tags), " tags found: ", tag_ids)


        color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(color_img, tuple(tag.corners[idx-1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

            cv2.putText(color_img, str(tag.tag_id),
                        org=(tag.corners[0, 0].astype(int)+10,tag.corners[0, 1].astype(int)+10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.8,
                        color=(0, 0, 255))

        if visualization:
            cv2.imshow('Detected tags for ' + image_name    , color_img)

            k = cv2.waitKey(0)
            if k == 27:         # wait for ESC key to exit
                cv2.destroyAllWindows()

    print("AVG time per detection: ", time_sum/time_num)
