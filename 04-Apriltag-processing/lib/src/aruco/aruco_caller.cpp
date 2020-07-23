#include <iostream>
#include <opencv2/core.hpp>
#include "aruco.h"
#include "dcf/dcfmarkertracker.h"

#define __PYTHON2__

#ifdef __PYTHON2__
#include "python2.7/Python.h"
#else
#include "python3.6/Python.h"
#endif

using namespace std;
using namespace aruco;

DFCMarkerTracker marker_tracker;
bool was_init = false;
float marker_size;

extern "C" void aruco_init(float marker_size_, char *tag_family) {
    marker_size = marker_size_;
    marker_tracker.setDictionary(tag_family);
    was_init = true;
}

extern "C" PyObject *aruco_detect_and_estimate(PyObject *calib_dict, unsigned char *img_data) {
    PyObject *markers_list = PyList_New(0);
    if (!was_init) {
        printf("Must call apriltag_detector_create first\n");
        return markers_list;
    }

    cv::Mat cameraMatrix(3, 3, CV_32FC1);
    cameraMatrix.at<float>(0, 0) = PyFloat_AsDouble(PyDict_GetItemString(calib_dict, "cameraMatrix00"));
    cameraMatrix.at<float>(0, 2) = PyFloat_AsDouble(PyDict_GetItemString(calib_dict, "cameraMatrix02"));
    cameraMatrix.at<float>(1, 1) = PyFloat_AsDouble(PyDict_GetItemString(calib_dict, "cameraMatrix11"));
    cameraMatrix.at<float>(1, 2) = PyFloat_AsDouble(PyDict_GetItemString(calib_dict, "cameraMatrix12"));

    cv::Mat distorsion(4, 1, CV_32FC1);
    distorsion.at<float>(0, 0) = PyFloat_AsDouble(PyDict_GetItemString(calib_dict, "distorsion0"));
    distorsion.at<float>(1, 0) = PyFloat_AsDouble(PyDict_GetItemString(calib_dict, "distorsion1"));
    distorsion.at<float>(2, 0) = PyFloat_AsDouble(PyDict_GetItemString(calib_dict, "distorsion2"));
    distorsion.at<float>(3, 0) = PyFloat_AsDouble(PyDict_GetItemString(calib_dict, "distorsion3"));

    cv::Size image_size;
    image_size.height = PyFloat_AsDouble(PyDict_GetItemString(calib_dict, "height"));
    image_size.width = PyFloat_AsDouble(PyDict_GetItemString(calib_dict, "width"));

    try {
        aruco::CameraParameters cam_params(cameraMatrix, distorsion, image_size);
        marker_tracker.setParams(cam_params, marker_size);
    } catch (cv::Exception &e) {
        printf("\nFailed: [%s]\n", e.msg.c_str());
    } catch (std::exception &e) {
        printf("\nFailed: [%s]\n", e.what());
    }

    cv::Mat image(image_size.height, image_size.width, CV_8UC1, img_data);
    map<int, cv::Ptr<TrackerImpl>> set_trackers = marker_tracker.track(image);
    marker_tracker.estimatePose();

    for (const auto &t : set_trackers) {
        const Marker &marker = t.second->getMarker();

        PyObject *marker_dict = PyDict_New();

#ifdef __PYTHON2__
        PyDict_SetItemString(marker_dict, "tag_id", PyLong_FromLong(marker.id));
        PyDict_SetItemString(marker_dict, "tag_family", PyString_FromString(marker.dict_info.c_str()));
#else
        PyDict_SetItem(marker_dict, PyUnicode_FromString("tag_id"), PyLong_FromLong(marker.id));
        PyDict_SetItem(marker_dict, PyUnicode_FromString("tag_family"), PyUnicode_FromString(marker.dict_info.c_str()));
#endif

        // these points are first 4 points in contour, not corners
        PyObject *marker_corners = PyList_New(0);
        for (int i = 0; i < 4; i++) {
            PyObject *marker_corner = PyList_New(0);
            PyList_Append(marker_corner, PyLong_FromLong(marker.contourPoints[i].x));
            PyList_Append(marker_corner, PyLong_FromLong(marker.contourPoints[i].y));
            PyList_Append(marker_corners, marker_corner);
        }
#ifdef __PYTHON2__
        PyDict_SetItemString(marker_dict, "corners", marker_corners);
#else
        PyDict_SetItem(marker_dict, PyUnicode_FromString("corners"), marker_corners);
#endif

        PyObject *marker_rvec = PyList_New(0);
        PyObject *marker_tvec = PyList_New(0);
        if (!marker.Rvec.empty() && !marker.Tvec.empty()) {
            for (int i = 0; i < 3; i++) {
                PyList_Append(marker_rvec, PyFloat_FromDouble(marker.Rvec.at<float>(i)));
                PyList_Append(marker_tvec, PyFloat_FromDouble(marker.Tvec.at<float>(i)));
            }
        }
#ifdef __PYTHON2__
        PyDict_SetItemString(marker_dict, "rvec", marker_rvec);
        PyDict_SetItemString(marker_dict, "tvec", marker_tvec);
#else
        PyDict_SetItem(marker_dict, PyUnicode_FromString("rvec"), marker_rvec);
        PyDict_SetItem(marker_dict, PyUnicode_FromString("tvec"), marker_tvec);
#endif

        PyList_Append(markers_list, marker_dict);
    }

    return markers_list;
}