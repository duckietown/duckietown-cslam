for computer_number in 20 21 22 23 8; do
    ssh duckietown@duckietown${computer_number}.local "cd duckietown-cslam; git pull; cd 04-Apriltag-processing; source remove-all-processors.bash; docker pull duckietown/apriltag-processor:master19-amd64; source launch-all-watchtowers.bash"
done
