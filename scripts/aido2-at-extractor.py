import os
import re
import subprocess
import shutil
import tempfile

import logging

logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

# Get a list of suitable bag files
# And copy the bags to a tmp folder (to improve ipfs performance)

tmpdir = tempfile.gettempdir()
bags_folder = os.getenv('BAGS_PATH')
bag_files = list()

logger.info("Looking for bags and copying them to a temporary directory:")
logger.info("BAGS FOUND:")
for fname in os.listdir(bags_folder):
    if ('autobot' in fname or 'watchtower' in fname or 'aidobot' in fname) and '.bag' in fname:
        orig_bag_name = os.path.join(bags_folder, fname)
        logger.info("   - %s" % orig_bag_name)

        tmp_bag_name = os.path.join(tmpdir, fname)
        shutil.copy(orig_bag_name, tmp_bag_name)
        logger.info("   - COPIED TO: %s" % tmp_bag_name)

        bag_files.append(tmp_bag_name)

# Extract the device names:
logger.info("DEVICES FOUND:")
device_names = list()
for bag_name in bag_files:
    device_names.append(re.search('watchtower\d{1,2}|autobot\d{1,2}|aidobot\d{1,2}', bag_name).group(0))
    logger.info("   - %s" % device_names[-1])

# Delete old statistics and processed files if exist:
if os.path.isfile(os.getenv("ACQ_OUTPUT_BAG")):
    logger.warning("[!!!] OLD OUTPUT BAG FOUND AT %s! ATTEMPTING DELETION." % os.getenv("ACQ_OUTPUT_BAG"))
    try:
        logger.info(os.getenv("ACQ_OUTPUT_BAG"))
        print("[---] REMOVAL SUCCESSFUL")
    except:
        logger.error("[!!!] REMOVAL FAILED. WE WILL NOT PROCEED!")
        exit(1)

    logger.warning("[!!!] OLD STATISTICS FILE FOUND AT %s! ATTEMPTING DELETION." % os.getenv("ACQ_OBSERVATIONS_STATISTICS_OUTPUT"))
    try:
        os.remove(os.getenv("ACQ_OBSERVATIONS_STATISTICS_OUTPUT"))
        logger.info("[---] REMOVAL SUCCESSFUL")
    except:
        logger.error("[!!!] REMOVAL FAILED. The new statistics will be appended at the end of the file.")

# Run the AT extractor
for bag_idx in range(len(bag_files)):
    bag = bag_files[bag_idx]
    device = device_names[bag_idx]

    logger.info("##########################################################################\n")
    logger.info("STARTING PROCESSING %s\n\n" % device)

    # If it is an autobot, toggle visual odometry
    if "autobot" in device or "aidobot" in device:
        vo_flag=1
        logger.info("AUTOBOT OR AIDOBOT DETECTED. WILL PROCESS WITH VISUAL ODOMETRY!\n")
    else:
        vo_flag=0
        logger.info("NOT AUTOBOT NOR AIDOBOT. Will process without visual odometry!\n")


    # Pass all environment variables
    cmd = ""
    env_dict = dict(os.environ)
    for envvar in env_dict:
        cmd += 'export %s="%s"; ' % (envvar, env_dict[envvar])
    cmd += "export ACQ_DEVICE_NAME=%s; " % device
    cmd += "export ACQ_DEVICE_MODE=postprocessing; "
    cmd += "export ACQ_SERVER_MODE=postprocessing; "
    cmd += "export ACQ_DEVICE_BAG=%s; " % bag
    cmd += "export ACQ_ODOMETRY_POST_VISUAL_ODOMETRY=%d; " % vo_flag
    cmd += "python acquire_and_publish.py"

    logger.info("EXECUTING COMMAND: %s \n\n\n" % cmd)

    subprocess.call(cmd, shell=True, env=dict(os.environ))
