#!/usr/bin/env python

"""
WesQuackerson (named after Wes Anderson due to his love of overhead scenes) combines all the
Watchtower footage from an experiment run and creates a nice video out of it.
"""

import os
import numpy as np
import rosbag
from duckietown_msgs.msg import AprilTagDetection
from sensor_msgs.msg import CompressedImage, Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
import rospy
import copy
import sys

OUTPUT_FRAMERATE = int(os.getenv('OUTPUT_FRAMERATE', 12))
MIN_SHOT_LENGTH = int(os.getenv('MIN_SHOT_LENGTH', 6))

ATMSGS_BAG = os.getenv('ATMSGS_BAG', "/bags/processed.bag")
VIDEO_BAGS = os.getenv('VIDEO_BAGS', "/bags")
POSES_TOPIC = os.getenv('POSES_TOPIC', "/poses_acquisition/poses")
VIDEO_TOPIC = os.getenv('VIDEO_TOPIC', "/camera_node/image/compressed")
OUTPUT_FILE = os.getenv('OUTPUT_FILE', "/bags/overheadVideo.mp4")

TRACKED_AT_ID = int(os.getenv('TRACKED_AT_ID',410))
CAMERA_RESOLUTION = (int(os.getenv('CAMERA_RESOLUTION_HEIGHT',1296)),int(os.getenv('CAMERA_RESOLUTION_WIDTH',972)))
TITLE_CARD = int(os.getenv('TITLE_CARD',1))

print("Starting the awesome WesQuackerson movie generation process.")
print("We are reading the AT messages from %s and video bags from %s." % (ATMSGS_BAG, VIDEO_BAGS))
print("The particular topics are: %s and %s." % (POSES_TOPIC, VIDEO_TOPIC))
print("The final video of how we track AT %d with resolution %s will be saved at %s." % (TRACKED_AT_ID, str(CAMERA_RESOLUTION), OUTPUT_FILE))
print("We will make it with frame rate %d and the minimum shot length wil be %d.\n" % (OUTPUT_FRAMERATE, MIN_SHOT_LENGTH))

def makeCut():
    """
    The cutMap specifies which camera stream is to be used for every frame of the final video
    """
    bag = rosbag.Bag(ATMSGS_BAG, 'r')


    # Extract the cameras and the end times of the bag
    cameras = list()
    cameras_dict = dict()
    bag_start_time = np.inf
    bag_end_time = -np.inf
    i=0
    print("Extracting the cameras in the bag")
    for topic, msg, t in bag.read_messages(topics=POSES_TOPIC):
        if str(msg.header.frame_id) not in cameras:
            cameras.append(str(msg.header.frame_id))
            cameras_dict[str(msg.header.frame_id)] = i
            i+=1
        if msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs < bag_start_time:
            bag_start_time = msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs
        if msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs > bag_end_time:
            bag_end_time = msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs

    print("The following cameras were found in the bag: %s" % str(cameras))
    print("Bag start time: %.8f, Bag end time: %.8f, Total seconds: %.8f" % (bag_start_time, bag_end_time, bag_end_time-bag_start_time))

    # Initialize the timestamps
    timestamps = np.arange(bag_start_time, bag_end_time, 1.0/OUTPUT_FRAMERATE)

    # Initialize the cutMap
    print("Processing the AT messages")
    cutMap = np.ones((len(cameras), len(timestamps))) * np.inf
    for topic, msg, t in bag.read_messages(topics=POSES_TOPIC):

        # Only if the observation is of the AT we are tracking
        if int(msg.tag_id) == int(TRACKED_AT_ID):
            cam = msg.header.frame_id
            center = msg.center
            #THE ORDER HERE COULD BE WRONG!!!
            dist_to_img_center = (center[0]-CAMERA_RESOLUTION[0])**2 + (center[1]-CAMERA_RESOLUTION[1])**2

            ts = msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs
            closest_earlier_timestamp_idx = np.where(timestamps<ts)[0][-1]
            cutMap[cameras_dict[cam], closest_earlier_timestamp_idx] = dist_to_img_center

    bag.close()

    print("Processing the AT messages finished")
    print("Starting with the director cut")

    # Find for each frame find the camera that had the most centered image,
    # if no camera saw the tracked tag, set it to -1
    cutTimeline = np.argmin(cutMap, axis=0)
    cutTimeline[np.all(np.isinf(cutMap), axis=0)] = -1


    # Replace the -1 values in the begining and in the end with the first non -1 value
    if cutTimeline[0] == -1:
        for i in range(len(cutTimeline)):
            if not cutTimeline[i] == -1:
                cutTimeline[:i] = cutTimeline[i]
                break
    # print("B1", np.array(np.unique(cutTimeline, return_counts=True)).T)

    reversed_cutTimeline = cutTimeline[::-1]
    if reversed_cutTimeline[0] == -1:
        for i in range(len(reversed_cutTimeline)):
            if not reversed_cutTimeline[i] == -1:
                reversed_cutTimeline[:i] = reversed_cutTimeline[i]
                break


    # Replace the -1 values in the middle by splitting the -1 range in half
    for i in range(len(cutTimeline)):
        #If you encounter an -1, find the last -1 from this sequence
        if cutTimeline[i] == -1:
            start_idx = i
            for j in range(i, len(cutTimeline)):
                if not cutTimeline[j] == -1:
                    end_idx = j-1
                    break
            # If only one, substitute with the previous
            if start_idx == end_idx:
                cutTimeline[start_idx]=cutTimeline[start_idx-1]
            # If only two, substitute with the previous and the next
            elif start_idx + 1 == end_idx:
                cutTimeline[start_idx]=cutTimeline[start_idx-1]
                cutTimeline[end_idx]=cutTimeline[end_idx+1]
            # Otherwise, split approx. in half
            else:
                midpoint = start_idx+int((end_idx-start_idx)/2)+1
                cutTimeline[start_idx:midpoint] = [cutTimeline[start_idx-1]] * (midpoint-start_idx)
                cutTimeline[midpoint:end_idx+1] = [cutTimeline[end_idx+1]] * (end_idx+1-midpoint)


    # Remove too quick shots
    groups = list()
    group_counts = list()
    # First find all consequitive groups of the same camera, as well as counts of how many frames each group has

    last_cam = None
    for i in range(len(cutTimeline)):
        curr_cam = cutTimeline[i]
        if curr_cam == last_cam:
            group_counts[-1]+=1
        else:
            last_cam = curr_cam
            groups.append(curr_cam)
            group_counts.append(1)

    # If the first one is too short, find the first long enough and use that camera
    if group_counts[0] < MIN_SHOT_LENGTH:
        for j, count in enumerate(group_counts[1:]):
            if group_counts >= MIN_SHOT_LENGTH:
                groups[:j] = groups[j]
                break

    # If the last one is too short, find the last long enough and use that camera
    reversed_group_counts = group_counts[::-1]
    reversed_groups = groups[::-1]
    if reversed_group_counts[0] < MIN_SHOT_LENGTH:
        for j, count in enumerate(reversed_group_counts[1:]):
            if reversed_group_counts >= MIN_SHOT_LENGTH:
                reversed_groups[:j] = reversed_groups[j]
                break

    # If there is a short cut somewhere in the middle, split it and assign it to the closest long enough cut
    changed = True
    while changed:
        changed = False
        for count_idx, count in enumerate(group_counts):
            if count < MIN_SHOT_LENGTH:
                # If only one frame, go with the previous
                if count == 1:
                    groups[count_idx] = groups[count_idx-1]

                # If two, then one to the previous one to the next
                elif count == 2:
                    groups[count_idx] = groups[count_idx-1]
                    group_counts[count_idx] = 1
                    group_counts[count_idx+1] += 1

                # If more, split approximately in half
                else:
                    first_half = int(count/2)
                    second_half = count - first_half
                    groups[count_idx] = groups[count_idx-1]
                    group_counts[count_idx] = first_half
                    group_counts[count_idx+1] += second_half

                changed = True

        #Combine consecutives with the same group
        new_groups = list()
        new_counts = list()
        last_group = None
        for group_idx, group in enumerate(groups):
            if last_group == group:
                new_counts[-1]+=group_counts[group_idx]
            else:
                new_groups.append(group)
                new_counts.append(group_counts[group_idx])
                last_group = group
        groups = copy.deepcopy(new_groups)
        group_counts = copy.deepcopy(new_counts)

    # Reconstruct the cutTimeline
    cutTimeline = list()
    for idx, group in enumerate(groups):
        for j in range(group_counts[idx]):
            cutTimeline.append(group)

    print("Director cut finished.")

    return cutTimeline, cameras, timestamps

def getBagFilename(device):
    for fname in os.listdir(VIDEO_BAGS):
        if device in fname:
            return VIDEO_BAGS+"/"+fname
    raise Exception("There is no bag with name containing %s in folder %s!" % ( device, VIDEO_BAGS ))

def makeVideo(cutTimeline, cameras, timestamps):
    bridge = CvBridge()

    # Setup the video recording
    fourcc = cv2.VideoWriter_fourcc(*'X264')
    video = cv2.VideoWriter(OUTPUT_FILE,fourcc,OUTPUT_FRAMERATE,CAMERA_RESOLUTION)

    # Add the titlecard if requested`
    if TITLE_CARD == 1:
        img = cv2.imread('/titlecard.png',cv2.IMREAD_COLOR)
        for i in range(2*OUTPUT_FRAMERATE):
            video.write(img)

    # Add the frame one by one
    last_camera = None
    bag = None

    selected_msg = None

    print("Rendering your video:")

    for i in range(len(cutTimeline)):
        camera = cameras[cutTimeline[i]]
        ts = timestamps[i]
        # Load the bag file if the last one was not the same
        if last_camera != camera:
            #First close the other one
            if bag is not None:
                bag.close()
            bag = rosbag.Bag(getBagFilename(camera))
        # Get the first message after the current time stamp
        last_time = 0
        for topic, msg, t in bag.read_messages(topics="/"+camera+VIDEO_TOPIC, start_time=rospy.Time.from_sec(ts-0.25)):
            selected_msg = msg
            if msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs > ts:
                break

        if selected_msg is not None:
            img = bridge.compressed_imgmsg_to_cv2(selected_msg, desired_encoding='bgr8')
        else:
            img = np.zeros((CAMERA_RESOLUTION[0], CAMERA_RESOLUTION[1], 3), np.uint8)
            print("msg detected as None, will show a black frame")
        # Add the frame to the video
        video.write(img)

        if i%OUTPUT_FRAMERATE == 0:
            print("%d\%d seconds processed" % (i/OUTPUT_FRAMERATE, len(cutTimeline)/OUTPUT_FRAMERATE))
            sys.stdout.flush()

    video.release()
    bag.close()

    print("Render to %s complete!" % OUTPUT_FILE)


cutTimeline, cameras, timestamps = makeCut()
makeVideo(cutTimeline, cameras, timestamps)
