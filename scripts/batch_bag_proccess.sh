# PARAMETERS TO SETUP
devices=(autobot04 watchtower01 watchtower02 watchtower03 watchtower04 watchtower05 watchtower06 watchtower07 watchtower08 watchtower09 watchtower10 watchtower11 watchtower12 watchtower13 watchtower14 watchtower15 watchtower16)

BAGS_PATH="/FILES/Documents/ETH_Zurich/Duckietown/SLAM/bags/19-04-18-experiment/"
OUTPUT_BAG_PATH="/FILES/Documents/ETH_Zurich/Duckietown/SLAM/bags/19-04-18-experiment/processed.bag"
STATISTICS_PATH="/FILES/Documents/ETH_Zurich/Duckietown/SLAM/bags/19-04-18-experiment/statistics.yaml"
ACQ_TEST_STREAM=0

printf "Setting up the acquisition batch bag processing. Sit back and enjoy.\n\n"

OUTPUT_BAG_DIR=$(dirname "${OUTPUT_BAG_PATH}")
OUTPUT_BAG_FILE=$(basename "${OUTPUT_BAG_PATH}")
STATISTICS_DIR=$(dirname "${STATISTICS_PATH}")
STATISTICS_FILE=$(basename "${STATISTICS_PATH}")

# Trying to remove existing output bag and statistics files
printf "Trying to remove existing output bag and statistics files\n"
rm ${OUTPUT_BAG_PATH}
rm ${STATISTICS_PATH}

# Figure out which are the bags
BAG_LIST=()
for index in ${!devices[*]}
do
    device=${devices[$index]}
    file=$(find "${BAGS_PATH}" -name "*$device*")
    printf "Device ${device} has a bagfile ${file}\n"
done

printf "\n\n"

for index in ${!devices[*]}
do
    device=${devices[$index]}
    file=$(find "${BAGS_PATH}" -name "*$device*")
    filename=$(basename "${file}")

    printf "##########################################################################\n"
    printf "STARTING PROCESSING ${devices[$index]}\n\n"

    docker run  --rm \
                --network=host \
                -e ACQ_DEVICE_MODE=postprocessing \
                -e ACQ_DEVICE_THREADS=6 \
                -e ACQ_DEVICE_NAME="${device}" \
                -e ACQ_DEVICE_BAG="/bags/${filename}" \
                -e ACQ_SERVER_MODE=postprocessing \
                -e ACQ_OUTPUT_BAG="/outputbag/${OUTPUT_BAG_FILE}" \
                -e ACQ_OBSERVATIONS_STATISTICS_OUTPUT="/statistics/${STATISTICS_FILE}" \
                -e ACQ_TEST_STREAM=${ACQ_TEST_STREAM} \
                -e ACQ_TOPIC_RAW=camera_node/image/compressed \
                -e ACQ_TOPIC_VELOCITY_TO_POSE=velocity_to_pose_node/pose \
                -v "${BAGS_PATH}":"/bags" \
                -v "${OUTPUT_BAG_DIR}":"/outputbag" \
                -v "${STATISTICS_DIR}":"/statistics" \
                duckietown/cslam-acquisition:x86-post

    printf "FINISHED PROCESSING ${devices[$index]}\n\n"
done
