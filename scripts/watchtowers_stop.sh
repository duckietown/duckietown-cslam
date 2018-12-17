#!/bin/bash

array=(demowatchtower09 demowatchtower10)

echo "We are stopping ${#array[*]} watchtowers"

for index in ${!array[*]}
do
    printf "\nStopping %s\n" ${array[$index]}
    docker -H ${array[$index]}.local stop cslam-aquisition || echo "Didn't stop older cslam-aquisition, probably doesn't exist, so don't worry."
    docker -H ${array[$index]}.local rm cslam-aquisition || echo "Didn't remove older cslam-aquisition, probably doesn't exist, so don't worry."
done
