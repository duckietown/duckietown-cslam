for computer_number in 20 21 22 23 8; do
    ssh duckietown@duckietown${computer_number}.local "cd duckietown-cslam/04-Apriltag-processing; source remove-all-processors.bash; source launch-all-watchtowers.bash"
done
