devices=(autobot03 watchtower01 watchtower02 watchtower03 watchtower04 watchtower05 watchtower06 watchtower07 watchtower08 watchtower09 watchtower10 watchtower11 watchtower12 watchtower13 watchtower14 watchtower15 watchtower16)

FOLDER_PATH="/home/amaury/AIDO2_experiment_data/submission_3219/attempt_01"

BAGS_PATH="${FOLDER_PATH}/bags"
OUTPUT_BAG_PATH="${FOLDER_PATH}/bags/processed.bag"
STATISTICS_PATH="${FOLDER_PATH}/bags/statistics.yaml"
ACQ_TEST_STREAM=0

printf "Setting up the acquisition batch bag processing. Sit back and enjoy.\n\n"

OUTPUT_BAG_DIR=$(dirname "${OUTPUT_BAG_PATH}")
OUTPUT_BAG_FILE=$(basename "${OUTPUT_BAG_PATH}")
STATISTICS_DIR=$(dirname "${STATISTICS_PATH}")
STATISTICS_FILE=$(basename "${STATISTICS_PATH}")

# # Trying to remove existing output bag and statistics files
# printf "Trying to remove existing output bag and statistics files\n"
# rm ${OUTPUT_BAG_PATH}
# rm ${STATISTICS_PATH}

# # Figure out which are the bags
# BAG_LIST=()
# for index in ${!devices[*]}
# do
#     device=${devices[$index]}
#     file=$(find "${BAGS_PATH}" -name "*$device*")
#     printf "Device ${device} has a bagfile ${file}\n"
# done

# printf "\n\n"

# for index in ${!devices[*]}
# do
#     device=${devices[$index]}
#     file=$(find "${BAGS_PATH}" -name "*$device*")
#     filename=$(basename "${file}")

#     printf "##########################################################################\n"
#     printf "STARTING PROCESSING ${devices[$index]}\n\n"

#     # If it is an autobot, toggle visual odometry
#     if [[ $device == *"autobot"* ]]; then
#       VO_FLAG=1
#       printf "AUTOBOT DETECTED. WILL PROCESS WITH VISUAL ODOMETRY!\n"
#     else
#       VO_FLAG=0
#       printf "NOT AUTOBOT. Will process without visual odometry!\n"
#     fi

#     docker run  --rm \
#                 --network=host \
#                 -e ACQ_DEVICE_MODE=postprocessing \
#                 -e ACQ_DEVICE_THREADS=6 \
#                 -e ACQ_DEVICE_NAME="${device}" \
#                 -e ACQ_DEVICE_BAG="/bags/${filename}" \
#                 -e ACQ_SERVER_MODE=postprocessing \
#                 -e ACQ_OUTPUT_BAG="/outputbag/${OUTPUT_BAG_FILE}" \
#                 -e ACQ_OBSERVATIONS_STATISTICS_OUTPUT="/statistics/${STATISTICS_FILE}" \
#                 -e ACQ_TEST_STREAM=${ACQ_TEST_STREAM} \
#                 -e ACQ_TOPIC_RAW=camera_node/image/compressed \
#                 -e ACQ_TOPIC_VELOCITY_TO_POSE=velocity_to_pose_node/pose \
#                 -e ACQ_ODOMETRY_POST_VISUAL_ODOMETRY=$VO_FLAG \
#                 -e ACQ_ODOMETRY_POST_VISUAL_ODOMETRY_FEATURES=SURF \
#                 -v "${BAGS_PATH}":"/bags" \
#                 -v "${OUTPUT_BAG_DIR}":"/outputbag" \
#                 -v "${STATISTICS_DIR}":"/statistics" \
#                 duckietown/cslam-acquisition:x86-doubletrouble

#     printf "FINISHED PROCESSING ${devices[$index]}\n\n"
# done

printf "##########################################################################\n"
printf "WES ANDERSON TAKES THE SCENE\n\n"

docker run  --rm \
            --network=host \
            -e OUTPUT_FRAMERATE=12 \
            -e MIN_SHOT_LENGTH=12 \
            -e ATMSGS_BAG=/bags/processed.bag \
            -e VIDEO_BAGS=/bags \
            -e POSES_TOPIC=/poses_acquisition/poses \
            -e VIDEO_TOPIC=/camera_node/image/compressed \
            -e OUTPUT_FILE=/bags/overheadVideo.mp4 \
            -e TRACKED_AT_ID=403 \
            -e CAMERA_RESOLUTION_HEIGHT=1296 \
            -e CAMERA_RESOLUTION_WIDTH=972 \
            -e TITLE_CARD=1 \
            -v "${BAGS_PATH}":"/bags" \
            duckietown/cslam-wes-quackerson

printf "##########################################################################\n"
printf "WES ANDERSON FINISHED HIS AWESOME MOVIE. Check it out! \n\n"
