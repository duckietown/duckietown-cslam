import os
import re
import subprocess

# Get a list of suitable bag files
bag_files = list()
print("BAGS FOUND:")
bags_folder = os.getenv('BAGS_PATH')
for fname in os.listdir(bags_folder):
    if ('autobot' in fname or 'watchtower' in fname or 'aidobot' in fname) and '.bag' in fname:
        bag_files.append(os.path.join(bags_folder, fname))
        print("   - %s" % bag_files[-1])

# Extract the device names:
print("DEVICES FOUND:")
device_names = list()
for bag_name in bag_files:
    device_names.append(re.search('watchtower\d{1,2}|autobot\d{1,2}|aidobot\d{1,2}', bag_name).group(0))
    print("   - %s" % device_names[-1])

# Delete old statistics and processed files if exist:
if os.path.isfile(os.getenv("ACQ_OUTPUT_BAG")):
    print("[!!!] OLD OUTPUT BAG FOUND AT %s! ATTEMPTING DELETION." % os.getenv("ACQ_OUTPUT_BAG"))
    try:
        os.remove(os.getenv("ACQ_OUTPUT_BAG"))
        print("[---] REMOVAL SUCCESSFUL")
    except:
        print("[!!!] REMOVAL FAILED. WE WILL NOT PROCEED!")
        exit(1)

    print("[!!!] OLD STATISTICS FILE FOUND AT %s! ATTEMPTING DELETION." % os.getenv("ACQ_OBSERVATIONS_STATISTICS_OUTPUT"))
    try:
        os.remove(os.getenv("ACQ_OBSERVATIONS_STATISTICS_OUTPUT"))
        print("[---] REMOVAL SUCCESSFUL")
    except:
        print("[!!!] REMOVAL FAILED. The new statistics will be appended at the end of the file.")

# Run the AT extractor
for bag_idx in range(len(bag_files)):
    bag = bag_files[bag_idx]
    device = device_names[bag_idx]

    print("##########################################################################\n")
    print("STARTING PROCESSING %s\n\n" % device)

    # If it is an autobot, toggle visual odometry
    if "autobot" in device or "aidobot" in device:
        vo_flag=1
        print("AUTOBOT OR AIDOBOT DETECTED. WILL PROCESS WITH VISUAL ODOMETRY!\n")
    else:
        vo_flag=0
        print("NOT AUTOBOT NOR AIDOBOT. Will process without visual odometry!\n")


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

    print("EXECUTING COMMAND: %s \n\n\n" % cmd)

    subprocess.call(cmd, shell=True, env=dict(os.environ))
