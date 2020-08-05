#include <iostream>
#include <opencv2/core.hpp>
#include <boost/python.hpp>
#include <boost/numpy/ndarray.hpp>
#include <boost/numpy.hpp>
#include "aruco.h"
#include "dcf/dcfmarkertracker.h"

using namespace std;
using namespace aruco;
using namespace boost::python;


DFCMarkerTracker marker_tracker;
bool was_init = false;
float marker_size;

void aruco_init(float marker_size_, const str &config_file_str) {
    string config_file_string = extract<string>(config_file_str);
    marker_size = marker_size_;
    marker_tracker.loadParamsFromFile(config_file_string);
    was_init = true;
}

list aruco_detect_and_estimate(const dict &calib_dict, const boost::numpy::ndarray &img_data) {
    if (!was_init) {
        printf("Must call apriltag_detector_create first\n");
        return list();
    }

    cv::Mat cameraMatrix(3, 3, CV_32F);
    cameraMatrix.at<float>(0, 0) = extract<float>(calib_dict["cameraMatrix00"]);
    cameraMatrix.at<float>(0, 2) = extract<float>(calib_dict["cameraMatrix02"]);
    cameraMatrix.at<float>(1, 1) = extract<float>(calib_dict["cameraMatrix11"]);
    cameraMatrix.at<float>(1, 2) = extract<float>(calib_dict["cameraMatrix12"]);

    cv::Mat distortion(4, 1, CV_32F);
    distortion.at<float>(0, 0) = extract<float>(calib_dict["distortion0"]);
    distortion.at<float>(1, 0) = extract<float>(calib_dict["distortion1"]);
    distortion.at<float>(2, 0) = extract<float>(calib_dict["distortion2"]);
    distortion.at<float>(3, 0) = extract<float>(calib_dict["distortion3"]);

    cv::Size image_size;
    image_size.height = extract<int>(calib_dict["height"]);
    image_size.width = extract<int>(calib_dict["width"]);

    try {
        aruco::CameraParameters cam_params(cameraMatrix, distortion, image_size);
        marker_tracker.setParams(cam_params, marker_size);
    } catch (cv::Exception &e) {
        printf("\nFailed: [%s]\n", e.msg.c_str());
    } catch (std::exception &e) {
        printf("\nFailed: [%s]\n", e.what());
    }

    cv::Mat image(image_size.height, image_size.width, CV_8U, img_data.get_data());
    map<int, cv::Ptr<TrackerImpl>> set_trackers = marker_tracker.track(image, 0.1);
    marker_tracker.estimatePose();

    list markers_list;

    for (const auto &t : set_trackers) {
        const Marker &marker = t.second->getMarker();

        dict marker_dict;

        marker_dict["tag_id"] = marker.id;
        marker_dict["tag_family"] = marker.dict_info;


        list marker_corners;
        for (int i = 0; i < 4; i++) {
            marker_corners.append(marker[i].x);
            marker_corners.append(marker[i].y);
        }

        marker_dict["corners"] = marker_corners;


        list marker_rvec;
        list marker_tvec;
        if (!marker.Rvec.empty() && !marker.Tvec.empty()) {
            for (int i = 0; i < 3; i++) {
                marker_rvec.append(marker.Rvec.at<float>(i));
                marker_tvec.append(marker.Tvec.at<float>(i));
            }
        }
        marker_dict["rvec"] = marker_rvec;
        marker_dict["tvec"] = marker_tvec;


        markers_list.append(marker_dict);
    }

    return markers_list;
}

BOOST_PYTHON_MODULE(aruco_caller) {
    boost::numpy::initialize();
    def("aruco_init", aruco_init);
    def("aruco_detect_and_estimate", aruco_detect_and_estimate);
}